/*
  MeteoGPS V4 - émission ESP-NOW - version ESP32 WROOM-32 (DevKit 30/38 broches)
  ESP32 WROOM-32 + BME280 + GPS NEO-7M + LCD 2004 I2C
  + anémomètre (0x01) et girouette (0x02) RS485 Modbus RTU

  Câblage ESP32 classique :
    BME280 + LCD (I2C) : SDA -> GPIO 21   SCL -> GPIO 22
    GPS NEO-7M (UART2) : TX GPS -> GPIO 16 (RX2)   RX GPS -> GPIO 17 (TX2)
    Module RS485 (UART1, direction automatique) :
      TXD module -> GPIO 18   RXD module -> GPIO 19   GND commun
      A -> Jaune anémo + girouette   B -> Vert anémo + girouette
    Anémo / girouette : Rouge +12V (10-30V)   Noir GND
    ATTENTION : GPIO 18 n'accepte pas 5V. Alimenter le module RS485 en 3,3V,
    ou mettre un pont diviseur sur sa sortie TXD s'il est alimenté en 5V.
  Broches à éviter : 6 à 11 (flash), 0 / 2 / 12 / 15 (démarrage),
  1 / 3 (USB-série du moniteur), 34 à 39 = entrées seulement.

  Arduino IDE : carte "ESP32 Dev Module".

  Trame envoyée (vent en m/s, dir = indice 0-15 par pas de 22,5°, 0 = Nord ;
  champ vide si le capteur ne répond pas) :
  MTO,cpt,QNH,T,HR,Td,meteo,lat,lon,alt,sats,vent,dir

  Testé pour le core Arduino ESP32 3.x
*/

#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>

/* ---------- CONFIG ---------- */
#define BME_ADDR 0x76

// --- ESP-NOW : ces 3 réglages doivent être IDENTIQUES sur le récepteur ---
// Avec la passerelle Wi-Fi, le canal doit être CELUI DE LA BOX (affiché par
// la passerelle au démarrage). Si BOX_SSID est renseigné, l'émetteur cherche
// la box au démarrage et prend son canal ; sinon (ou si la box n'est pas
// captée), il utilise ESPNOW_CHANNEL.
#define ESPNOW_CHANNEL 1
#define BOX_SSID ""               // ex. "NomDeTaBox" ; "" = canal fixe
#define USE_LONG_RANGE 1          // 1 = mode LR Espressif (plus de portée, débit réduit)
#define TX_POWER WIFI_POWER_19_5dBm   // pleine puissance : OK avec l'antenne PCB du WROOM-32

// --- Broches ESP32 WROOM-32 ---
#define I2C_SDA 21
#define I2C_SCL 22
#define RS485_RX 18
#define RS485_TX 19

// --- Vent : RS485 / Modbus RTU ---
#define RS485_BAUD      9600
#define MODBUS_TIMEOUT  300     // ms ; le GPS reste lu pendant l'attente
#define MODBUS_GAP      50      // ms de silence avant chaque requête
#define ADDR_ANEMOMETRE 0x01
#define ADDR_GIROUETTE  0x02
#define REG_VITESSE     0x0000
#define REG_DIRECTION   0x0000
#define WIND_DIR_OFFSET 1       // calage de la girouette, en pas de 22,5°

// Broadcast : pas besoin de connaître l'adresse MAC du récepteur.
// Pour un envoi ciblé (avec accusé de réception), remplace par la MAC
// affichée par le récepteur au démarrage, ex. {0x34,0x85,0x18,0xAB,0xCD,0xEF}
uint8_t peerMac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

/* ---------- OBJETS ---------- */
Adafruit_BME280 bme;
TinyGPSPlus gps;
hd44780_I2Cexp disp(0x27, 20, 4);

static const int RXPin = 16, TXPin = 17;   // UART2 (RX2/TX2 sur le DevKit)
static const uint32_t GPSBaud = 9600;
HardwareSerial hs(2);
HardwareSerial rs485(1);

bool radioOk = false;
uint16_t txCount = 0;

/* ---------- TIMERS ---------- */
unsigned long tRead = 0;
const long intervalRead = 15000;

/* ---------- DONNÉES ---------- */
int temperature, humidity, pressure, dew, diff;

float windSpeed = 0.0;        // m/s
int   windDir   = 0;          // indice dans CARDINAUX
bool  windSpeedOk = false;
bool  windDirOk   = false;

const char* const CARDINAUX[16] = {
  "N","NNE","NE","ENE","E","ESE","SE","SSE",
  "S","SSO","SO","OSO","O","ONO","NO","NNO"
};

/* ---------- ENUM METEO ---------- */
enum Meteo {
  METEO_CLEAR,
  METEO_FOG,
  METEO_RAIN,
  METEO_FLYABLE,
  METEO_HUMID
};
Meteo currentMeteo;

/* ---------- POINT DE ROSÉE ---------- */
float dewPoint(float temp, float hum) {
  const float a = 17.27;
  const float b = 237.7;
  float alpha = ((a * temp) / (b + temp)) + log(hum / 100.0);
  return (b * alpha) / (a - alpha);
}

/* ---------- LECTURE CAPTEUR ---------- */
void readSensors() {
  temperature = bme.readTemperature() - 8;
  humidity    = round(bme.readHumidity());
  pressure    = round(bme.readPressure() / 100.0) + (gps.altitude.meters() / 8.5) - 3;
  dew         = round(dewPoint(temperature, humidity));
  diff        = temperature - dew;
}

/* ---------- GPS ---------- */
void feedGps() {
  while (hs.available() > 0) gps.encode(hs.read());
}

/* ---------- MODBUS RTU ---------- */
uint16_t modbusCRC(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int b = 0; b < 8; b++) {
      crc = (crc & 0x0001) ? (crc >> 1) ^ 0xA001 : crc >> 1;
    }
  }
  return crc;
}

// Lecture d'un registre (fonction 0x03). Le GPS continue d'être lu pendant
// les attentes : à 9600 bauds, son tampon déborderait si un capteur ne répond pas.
bool modbusRead(uint8_t addr, uint16_t reg, uint16_t& value) {
  unsigned long t = millis();
  while (millis() - t < MODBUS_GAP) feedGps();

  uint8_t req[8] = { addr, 0x03, (uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF), 0x00, 0x01, 0, 0 };
  uint16_t crc = modbusCRC(req, 6);
  req[6] = crc & 0xFF;
  req[7] = crc >> 8;

  while (rs485.available()) rs485.read();
  rs485.write(req, sizeof(req));
  rs485.flush();

  uint8_t resp[7];
  t = millis();
  while (rs485.available() < (int)sizeof(resp)) {
    if (millis() - t > MODBUS_TIMEOUT) return false;
    feedGps();
  }
  rs485.readBytes(resp, sizeof(resp));

  if (resp[0] != addr || resp[1] != 0x03 || resp[2] != 0x02) return false;
  if (modbusCRC(resp, 5) != (uint16_t)(resp[5] | (resp[6] << 8))) return false;

  value = ((uint16_t)resp[3] << 8) | resp[4];
  return true;
}

/* ---------- LECTURE VENT ---------- */
void readWind() {
  uint16_t raw = 0;

  windSpeedOk = modbusRead(ADDR_ANEMOMETRE, REG_VITESSE, raw);
  if (windSpeedOk) windSpeed = raw * 0.1f;
  else Serial.println(F("Anemometre : pas de reponse RS485"));

  windDirOk = modbusRead(ADDR_GIROUETTE, REG_DIRECTION, raw);
  if (windDirOk) windDir = (raw + WIND_DIR_OFFSET) % 16;
  else Serial.println(F("Girouette : pas de reponse RS485"));
}

/* ---------- LOGIQUE METEO ---------- */
Meteo computeMeteo() {
  if (humidity >= 98) return METEO_RAIN;
  if (diff <= 2 && humidity >= 96) return METEO_FOG;
  if (diff > 3 && humidity <= 75) return METEO_FLYABLE;
  if (diff <= 3 && humidity <= 96 && humidity >= 90) return METEO_HUMID;
  return METEO_CLEAR;
}

/* ---------- ESP-NOW ---------- */
void setupEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  uint8_t channel = ESPNOW_CHANNEL;
  if (strlen(BOX_SSID) > 0) {
    // Recherche de la box (avant de passer en mode LR seul)
    int n = WiFi.scanNetworks(false, false, false, 300, 0, BOX_SSID);
    if (n > 0) {
      channel = WiFi.channel(0);
      Serial.print(F("Box trouvee, canal "));
    } else {
      Serial.print(F("Box non captee, canal par defaut "));
    }
    Serial.println(channel);
    WiFi.scanDelete();
  }

#if USE_LONG_RANGE
  esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR);
#endif
  esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
  WiFi.setTxPower(TX_POWER);

  Serial.print(F("MAC emetteur : "));
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println(F("ESP-NOW : init echouee"));
    return;
  }

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, peerMac, 6);
  peer.channel = channel;
  peer.ifidx   = WIFI_IF_STA;
  peer.encrypt = false;

  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println(F("ESP-NOW : ajout du pair echoue"));
    return;
  }
  radioOk = true;
  Serial.println(F("ESP-NOW OK"));
}

// Trame CSV : MTO,cpt,QNH,T,HR,Td,meteo,lat,lon,alt,sats,vent,dir
void sendFrame() {
  if (!radioOk) return;

  char sVent[8] = "";
  char sDir[4]  = "";
  if (windSpeedOk) snprintf(sVent, sizeof(sVent), "%.1f", windSpeed);
  if (windDirOk)   snprintf(sDir, sizeof(sDir), "%d", windDir);

  char buf[96];
  if (gps.location.isValid()) {
    snprintf(buf, sizeof(buf), "MTO,%u,%d,%d,%d,%d,%d,%.5f,%.5f,%.0f,%lu,%s,%s",
             txCount, pressure, temperature, humidity, dew, (int)currentMeteo,
             gps.location.lat(), gps.location.lng(),
             gps.altitude.meters(), (unsigned long)gps.satellites.value(),
             sVent, sDir);
  } else {
    snprintf(buf, sizeof(buf), "MTO,%u,%d,%d,%d,%d,%d,,,,0,%s,%s",
             txCount, pressure, temperature, humidity, dew, (int)currentMeteo,
             sVent, sDir);
  }

  esp_err_t res = esp_now_send(peerMac, (const uint8_t*)buf, strlen(buf) + 1); // +1 : envoie le '\0'
  txCount++;

  Serial.print(res == ESP_OK ? F("TX ESP-NOW : ") : F("TX ECHEC : "));
  Serial.println(buf);
}

/* ---------- SERIAL ---------- */
void printSerial() {
  Serial.print("temperature: "); Serial.print(temperature); Serial.println("°C");
  Serial.print("Pression: ");    Serial.print(pressure);    Serial.println(" hPa");
  Serial.print("Humidité: ");    Serial.print(humidity);    Serial.println(" %");
  Serial.print("Dew point: ");   Serial.print(dew);         Serial.println(" °C");

  Serial.print("Vent: ");
  if (windSpeedOk) { Serial.print(windSpeed, 1); Serial.print(" m/s"); }
  else Serial.print("--");
  Serial.print("  Direction: ");
  Serial.println(windDirOk ? CARDINAUX[windDir] : "--");

  switch (currentMeteo) {
    case METEO_FOG:     Serial.println("Brouillard"); break;
    case METEO_RAIN:    Serial.println("Pluie");      break;
    case METEO_FLYABLE: Serial.println("Volable");    break;
    case METEO_HUMID:   Serial.println("humide++");   break;
    default:            Serial.println();
  }
}

/* -------------AFFICHAGE----------- */
void lcdDisp() {
  char l[32];

  snprintf(l, sizeof(l), "QNH %4d hPa T%3d%cC ", pressure, temperature, (char)223);
  disp.setCursor(0, 0);
  disp.print(l);

  snprintf(l, sizeof(l), "Humid %3d %%        %c", humidity, radioOk ? 'W' : '-');   // témoin radio
  disp.setCursor(0, 1);
  disp.print(l);

  char sVent[8] = " --.-";
  if (windSpeedOk) snprintf(sVent, sizeof(sVent), "%5.1f", windSpeed * 3.6f);
  snprintf(l, sizeof(l), "Vent %s km/h %-3s ", sVent, windDirOk ? CARDINAUX[windDir] : "--");
  disp.setCursor(0, 2);
  disp.print(l);

  disp.setCursor(0, 3);
  disp.print("Alt GPS: " + String(gps.altitude.meters(), 0) + " m   ");
  disp.setCursor(18, 3);
  disp.print(String(gps.satellites.value()));
}

void displayGps() {
  Serial.print(F("Date/Time: "));
  if (gps.date.isValid()) {
    Serial.print(gps.date.day());   Serial.print(F("/"));
    Serial.print(gps.date.month()); Serial.print(F("/"));
    Serial.print(gps.date.year());  Serial.print(F(" - "));
  } else {
    return;
  }

  Serial.print(F(" "));
  if (gps.time.isValid()) {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());   Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute()); Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.println(gps.time.second());
  } else {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("Location: "));
  if (gps.location.isValid()) {
    Serial.print(gps.location.lat(), 6); Serial.print(F(","));
    Serial.println(gps.location.lng(), 6);
  } else {
    Serial.println(F("INVALID"));
  }

  Serial.print(F("Altitude: "));
  if (gps.altitude.isValid()) { Serial.print(gps.altitude.meters()); Serial.println(F("m")); }
  else Serial.println(F("INVALID"));

  Serial.print(F("Nbre Fix: "));
  if (gps.satellites.isValid()) Serial.println(gps.satellites.value());
  else Serial.println(F("INVALID"));

  Serial.print(F("Quality: "));
  if (gps.hdop.isValid()) Serial.println(gps.hdop.hdop());
  else Serial.println(F("INVALID"));

  Serial.println();
}

/* ---------- SETUP ---------- */
void setup() {
  Serial.begin(115200);
  hs.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin);
  rs485.begin(RS485_BAUD, SERIAL_8N1, RS485_RX, RS485_TX);
  Wire.begin(I2C_SDA, I2C_SCL);

  if (!bme.begin(BME_ADDR)) {
    Serial.println(F("BME280 introuvable : verifier SDA/SCL et l'adresse 0x76/0x77"));
    while (1) delay(1000);
  }

  disp.init();
  disp.backlight();

  setupEspNow();

  readSensors();
  readWind();
  currentMeteo = computeMeteo();
  lcdDisp();
  printSerial();
  displayGps();
}

/* ---------- LOOP ---------- */
void loop() {
  feedGps();

  unsigned long now = millis();
  if (now > 5000 && gps.charsProcessed() < 10) {
    Serial.println(F("No GPS detected: check wiring."));
    while (1);
  }

  if (now - tRead >= intervalRead) {
    tRead = now;
    readSensors();
    readWind();
    currentMeteo = computeMeteo();
    printSerial();
    displayGps();
    lcdDisp();
    sendFrame();
  }
}
