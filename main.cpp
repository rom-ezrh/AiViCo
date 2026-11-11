/*
 * Cubicore Devboard (RAK3112) - Multi-Sensor Industrial Telemetry Node
 * DS18B20 temp (GPIO 21) | MPU6050 vibration (I2C SDA 9, SCL 40) | SCT-013 current (GPIO 2)
 * Band: built-in AS923_3 (916.6 / 916.8 MHz, SF10 / DR2), CayenneLPP payload.
 */
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <RadioLib.h>
#include <CayenneLPP.h>
#include <Preferences.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#define PIN_LED_BLUE 45
#define PIN_LED_GREEN 46
#define LORA_NSS 7
#define LORA_SCK 5
#define LORA_MOSI 6
#define LORA_MISO 3
#define LORA_RESET 8
#define LORA_BUSY 48
#define LORA_DIO1 47
#define PIN_DS18B20 21
#define I2C_SDA_PIN 9
#define I2C_SCL_PIN 40
#define PIN_SCT013_ADC 2

#define CURRENT_CALIBRATION 30.0f // SCT-013-030 (30A/1V)
#define ADC_REF_VOLTAGE 3.3f
#define ADC_MAX_COUNTS 4095.0f
#define CURRENT_NOISE_CUTOFF 0.05f
#define UPLINK_INTERVAL_MS 20000

static const uint8_t joinEUIBytes[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static const uint8_t devEUIBytes[8] = {0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x07, 0x9D, 0x24};
static const uint8_t appKeyBytes[16] = {0x6B, 0xF4, 0xD4, 0x98, 0x74, 0x0F, 0xA1, 0x52, 0x8C, 0xF5, 0xBD, 0xC2, 0xC6, 0x8D, 0xD1, 0x46};
static const uint8_t nwkKeyBytes[16] = {0x6B, 0xF4, 0xD4, 0x98, 0x74, 0x0F, 0xA1, 0x52, 0x8C, 0xF5, 0xBD, 0xC2, 0xC6, 0x8D, 0xD1, 0x46};

static inline uint64_t arrayToUint64(const uint8_t *arr)
{
  uint64_t val = 0;
  for (int i = 0; i < 8; i++)
    val = (val << 8) | (uint64_t)arr[i];
  return val;
}

OneWire oneWire(PIN_DS18B20);
DallasTemperature ds18b20(&oneWire);
Adafruit_MPU6050 mpu;
bool mpuAvailable = false;

SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_RESET, LORA_BUSY);
LoRaWANNode node(&radio, &AS923_3); // built-in band (replaces the custom AS923_3_Cubicore)
CayenneLPP lpp(51);
Preferences prefs;
unsigned long lastUplinkTime = 0;

void blinkLED(uint8_t pin, int times, int onMs = 120, int offMs = 120)
{
  for (int i = 0; i < times; i++)
  {
    digitalWrite(pin, HIGH);
    delay(onMs);
    digitalWrite(pin, LOW);
    delay(offMs);
  }
}

float readDS18B20Temperature()
{
  ds18b20.requestTemperatures();
  float temp = ds18b20.getTempCByIndex(0);
  if (temp == DEVICE_DISCONNECTED_C || temp < -55.0f || temp > 125.0f)
  {
    Serial.println("  [DS18B20] Read error. Check the 4.7k pull-up and wiring on GPIO 21.");
    return NAN;
  }
  return temp;
}

struct MotionData
{
  float ax_g, ay_g, az_g, vibration_g;
  bool valid;
};

MotionData readMPU6050Motion()
{
  MotionData md = {0, 0, 0, 0, false};
  if (!mpuAvailable)
    return md;
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  md.ax_g = a.acceleration.x / 9.80665f;
  md.ay_g = a.acceleration.y / 9.80665f;
  md.az_g = a.acceleration.z / 9.80665f;

  float sumSqDiff = 0.0f;
  const int numSamples = 20;
  for (int i = 0; i < numSamples; i++)
  {
    sensors_event_t sa, sg, st;
    mpu.getEvent(&sa, &sg, &st);
    float gx = sa.acceleration.x / 9.80665f, gy = sa.acceleration.y / 9.80665f, gz = sa.acceleration.z / 9.80665f;
    float diff = fabsf(sqrtf(gx * gx + gy * gy + gz * gz) - 1.0f);
    sumSqDiff += diff * diff;
    delay(5);
  }
  md.vibration_g = sqrtf(sumSqDiff / numSamples);
  md.valid = true;
  return md;
}

float readSCT013Current()
{
  const unsigned long sampleDurationMs = 200;
  unsigned long startTime = millis(), numberOfSamples = 0;
  double sumSquaredVoltage = 0.0, offsetV = 2048.0;
  while ((millis() - startTime) < sampleDurationMs)
  {
    int rawSample = analogRead(PIN_SCT013_ADC);
    numberOfSamples++;
    offsetV = offsetV + (rawSample - offsetV) * 0.005;
    double v = ((rawSample - offsetV) / ADC_MAX_COUNTS) * ADC_REF_VOLTAGE;
    sumSquaredVoltage += v * v;
    delayMicroseconds(100);
  }
  if (numberOfSamples == 0)
    return 0.0f;
  double iRMS = sqrt(sumSquaredVoltage / numberOfSamples) * CURRENT_CALIBRATION;
  if (iRMS < CURRENT_NOISE_CUTOFF)
    iRMS = 0.0;
  return (float)iRMS;
}

void setup()
{
  Serial.begin(115200);
  delay(2000);
  pinMode(PIN_LED_BLUE, OUTPUT);
  pinMode(PIN_LED_GREEN, OUTPUT);
  digitalWrite(PIN_LED_BLUE, LOW);
  digitalWrite(PIN_LED_GREEN, LOW);
  Serial.printf("\n== Cubicore industrial telemetry node == build %s %s\n", __DATE__, __TIME__);

  ds18b20.begin();
  Serial.printf("[Sensor] DS18B20 found: %d\n", ds18b20.getDeviceCount());

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  if (mpu.begin(0x68, &Wire))
  {
    mpuAvailable = true;
    mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.println("[Sensor] MPU6050 OK");
  }
  else
  {
    Serial.println("[Sensor] MPU6050 NOT FOUND (check SDA=GPIO9, SCL=GPIO40, 3.3V, GND)");
  }

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);
  int16_t state = radio.begin();
  if (state != RADIOLIB_ERR_NONE)
  {
    Serial.printf("[Radio] SX1262 init FAILED (code %d)\n", state);
    while (true)
    {
      blinkLED(PIN_LED_BLUE, 3, 100, 100);
      delay(500);
    }
  }
  radio.setDio2AsRfSwitch(true);
  radio.setTCXO(1.8);
  delay(100);
  Serial.println("[Radio] SX1262 OK");

  node.beginOTAA(arrayToUint64(joinEUIBytes), arrayToUint64(devEUIBytes), (uint8_t *)nwkKeyBytes, (uint8_t *)appKeyBytes);
  node.setADR(false);  // required by the Cubicore dual-channel gateway
  node.setDatarate(2); // fixed SF10

  // Restore the saved DevNonce so TTN accepts the join after a reboot
  prefs.begin("lorawan", false);
  if (prefs.isKey("nonces"))
  {
    uint8_t buf[RADIOLIB_LORAWAN_NONCES_BUF_SIZE];
    prefs.getBytes("nonces", buf, RADIOLIB_LORAWAN_NONCES_BUF_SIZE);
    node.setBufferNonces(buf);
  }

  int attempt = 1;
  while (!node.isActivated())
  {
    Serial.printf("[LoRaWAN] Join attempt #%d...\n", attempt++);
    digitalWrite(PIN_LED_BLUE, HIGH);
    state = node.activateOTAA();
    digitalWrite(PIN_LED_BLUE, LOW);
    if (state == RADIOLIB_LORAWAN_NEW_SESSION || state == RADIOLIB_ERR_NONE)
    {
      Serial.println("[LoRaWAN] JOIN ACCEPTED");
      prefs.putBytes("nonces", node.getBufferNonces(), RADIOLIB_LORAWAN_NONCES_BUF_SIZE);
      blinkLED(PIN_LED_GREEN, 5, 100, 100);
      break;
    }
    Serial.printf("[LoRaWAN] Join failed (code %d). Retrying in 15 s...\n", state);
    delay(15000);
  }
  prefs.end();
  node.setADR(false);
  node.setDatarate(2);
  node.setDwellTime(false);
  Serial.println("[System] Telemetry loop starting.");
}

void loop()
{
  if (millis() - lastUplinkTime < UPLINK_INTERVAL_MS)
    return;
  lastUplinkTime = millis();

  float tempC = readDS18B20Temperature();
  MotionData motion = readMPU6050Motion();
  float currentA = readSCT013Current();
  Serial.printf("\n[Data] temp=%.2f C | vib=%.3f g | current=%.2f A | X=%.2f Y=%.2f Z=%.2f g\n",
                tempC, motion.vibration_g, currentA, motion.ax_g, motion.ay_g, motion.az_g);

  lpp.reset();
  if (!isnan(tempC))
    lpp.addTemperature(1, tempC);
  if (motion.valid)
  {
    lpp.addAccelerometer(2, motion.ax_g, motion.ay_g, motion.az_g);
    lpp.addAnalogInput(3, motion.vibration_g);
  }
  lpp.addAnalogInput(4, currentA);

  digitalWrite(PIN_LED_BLUE, HIGH);
  int16_t state = node.sendReceive(lpp.getBuffer(), lpp.getSize(), 1);
  digitalWrite(PIN_LED_BLUE, LOW);

  if (state == RADIOLIB_ERR_NONE)
  {
    Serial.printf("  --> [OK] Uplink sent (%d bytes)\n", lpp.getSize());
    blinkLED(PIN_LED_GREEN, 2, 100, 100);
  }
  else
  {
    Serial.printf("  --> [Error] Uplink failed (code: %d)\n", state);
  }
}
