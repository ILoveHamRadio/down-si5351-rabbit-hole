#include <Arduino.h>
#include <AiEsp32RotaryEncoder.h>
#include <si5351.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <RTClib.h>
#include <TimeLib.h>
#include <JTEncode.h>

Si5351 si5351;
// calibration factor for si5351 based on the calibration tool
int32_t cal_factor = 148000;
uint64_t freqMultiplier = 1000ULL;

// global definitions
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Defines for Frequency Display
#define FREQUENCY_DIVISOR_HZ 10
#define FREQUENCY_DIVISOR_KHZ 100
#define FREQUENCY_DIVISOR_MHZ 100000

#define LED_PIN 12

// global variables
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// Rotary Encoder Definitions
#define ROTARY_ENCODER_A_PIN 18
#define ROTARY_ENCODER_B_PIN 5
#define ROTARY_ENCODER_BUTTON_PIN 19
#define ROTARY_ENCODER_VCC_PIN -1 /* 27 put -1 of Rotary encoder Vcc is connected directly to 3,3V; else you can use declared output pin for powering rotary encoder */

// depending on your encoder - try 1,2 or 4 to get expected behaviour
#define ROTARY_ENCODER_STEPS 4
unsigned long shortPressAfterMiliseconds = 50;  // how long a short press shoud be. Do not set too low to avoid bouncing (false press events).
unsigned long longPressAfterMiliseconds = 1000; // how long a long press shoud be.

// instead of changing here, rather change numbers above
AiEsp32RotaryEncoder rotaryEncoder = AiEsp32RotaryEncoder(ROTARY_ENCODER_A_PIN, ROTARY_ENCODER_B_PIN, ROTARY_ENCODER_BUTTON_PIN, ROTARY_ENCODER_VCC_PIN, ROTARY_ENCODER_STEPS);

const int bandswitch[] = {160, 80, 40, 20, 15, 10, 6};
const int bandswitch_max = 6;
const long freqswitch_low[] = {180000, 350000, 700000, 1400000, 2100000, 2800000, 5000000};
const long freqswitch_high[] = {200000, 400000, 730000, 1435000, 2145000, 2970000, 5400000};

const String modes[] = {"USB", "LSB", "CW", "WSPR", "FT8", "FT4"};
int current_mode = 3;

int current_band = 4;
long current_freq[] = {183660, 356860, 703860, 1409705, 2109460, 2812460, 5000100};
long target_freq;

/*
   This sample sketch demonstrates the normal use of a TinyGPSPlus (TinyGPSPlus) object.
   It requires the use of SoftwareSerial, and assumes that you have a
   9600-baud serial GPS device hooked up on pins 17(rx) and 16(tx).
*/
static const int RXPin = 17, TXPin = 16;
static const uint32_t GPSBaud = 9600;

bool gpsLocationValid = false;
bool gpsDateValid = false;
bool gpsTimeValid = false;
String gridSquare = "";

// The TinyGPSPlus object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

// DS1307 RTC
RTC_DS1307 rtc;

// WSPR Definitions
#define WSPR_TONE_SPACING 146 // ~1.46 Hz
#define WSPR_DELAY 683        // Delay value for WSPR
// WSPR Global Variables
JTEncode jtencode;
boolean txEnabled = false;
DateTime dt;
// uint8_t tx_buffer[] = {3, 1, 2, 0, 0, 2, 0, 0, 1, 0, 0, 0, 3, 3, 1, 2, 2, 2, 3, 2, 2, 1, 0, 1, 1, 3, 3, 0, 2, 2, 2, 0, 0, 0, 3, 2, 2, 3, 2, 1, 2, 0, 0, 2, 2, 0, 1, 0, 1, 3, 2, 2, 1, 1, 0, 1, 0, 0, 0, 1, 1, 0, 1, 2, 2, 0, 0, 3, 1, 2, 3, 0, 3, 0, 3, 0, 3, 2, 2, 3, 2, 0, 1, 2, 1, 1, 2, 2, 0, 1, 1, 0, 1, 0, 3, 0, 2, 2, 3, 2, 2, 0, 0, 0, 1, 0, 2, 1, 0, 0, 3, 1, 1, 0, 3, 1, 2, 0, 3, 3, 0, 3, 2, 2, 2, 1, 3, 3, 2, 2, 0, 0, 0, 1, 2, 1, 2, 0, 1, 1, 2, 0, 2, 0, 2, 0, 2, 1, 3, 2, 3, 0, 3, 3, 0, 2, 2, 1, 3, 0, 2, 2};
uint8_t tx_buffer[WSPR_SYMBOL_COUNT];
char call[] = "K5VZ"; // change to your callsign
char loc[] = "EM13";
uint8_t dbm = 3;

// predefined function prototypes
// rotary encoder functions
void init_rotary_encoder();
void on_button_short_click();
void on_button_long_click();
void handle_band_change();
void rotary_loop();
void IRAM_ATTR readEncoderISR();
// si5351 functions
void init_si5351();
void set_si5351_freuency(long target_freq);
// oled display functions
void clearDisplay();
void showDisplay();
void displayFrequency(uint64_t frequency);
void displayBand();
void displayMode();
void displayTime();
void displayInTransmit();
void displayGridSquare(String gridSquare);
void displayInitGPS();
void display_init_gps();
void display_loop();
void display_transmitting();

// GPS routines
void initGPSSync();
void displayGPSInfo();
static char *get_mh(double lat, double lon, int size);

// WSPR routines
void jtTransmitMessage();

void setup()
{
  Serial.begin(115200);
  // connect to GPS
  ss.begin(GPSBaud);
  // set tx led pin mode
  pinMode(LED_PIN, OUTPUT);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;
  }
  // pause to ensure everything initializes
  delay(2000);
  // SETUP RTC MODULE
  if (!rtc.begin())
  {
    Serial.println("RTC module is NOT found");
    Serial.flush();
    while (1)
      ;
  }

  target_freq = current_freq[current_band];

  init_rotary_encoder();
  init_si5351();
  while (gridSquare == "")
  {
    initGPSSync();
  }
}

void loop()
{
  rotary_loop();
  display_loop();
  set_si5351_freuency(target_freq);
  if (txEnabled)
  {
    dt = rtc.now();
    // WSPR TX LOOP
    if (dt.second() == 0 && (dt.minute() % 2 == 0))
    {
      jtTransmitMessage();
    }
  }
  delay(10);
}

#pragma region gps_functions

void initGPSSync()
{
  display_init_gps();
  // This sketch displays information every time a new sentence is correctly encoded.
  while (ss.available() > 0)
  {
    if (gps.encode(ss.read()))
    {
      displayGPSInfo();
    }
    if (gpsLocationValid == true && gpsDateValid == true && gpsTimeValid == true)
    {
      return;
    }
  }

  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    while (true)
      delay(500);
  }
}

void displayGPSInfo()
{
  Serial.print(F("Location: "));
  if (gps.location.isValid())
  {
    gpsLocationValid = true;
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
    Serial.print(F(", Grid Square: "));
    gridSquare = get_mh(gps.location.lat(), gps.location.lng(), 4);
    Serial.println(gridSquare);
    Serial.println(F("GridSqure updated from GPS"));
  }
  else
  {
    gpsLocationValid = false;
    Serial.print(F("INVALID "));
  }

  Serial.print(F("Date/Time: "));
  if (gps.date.isValid())
  {
    gpsDateValid = true;
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else
  {
    gpsDateValid = false;
    Serial.print(F("INVALID "));
  }

  Serial.print(F(" "));
  if (gps.time.isValid())
  {
    gpsTimeValid = true;
    if (gps.time.hour() < 10)
      Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10)
      Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10)
      Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10)
      Serial.print(F("0"));
    Serial.println(gps.time.centisecond());
  }
  else
  {
    gpsTimeValid = false;
    Serial.print(F("INVALID"));
  }

  if (gpsDateValid && gpsTimeValid)
  {
    rtc.adjust(DateTime(gps.date.year(), gps.date.month(), gps.date.day(), gps.time.hour(), gps.time.minute(), gps.time.second()));
    Serial.println(F("RTC Updated with GPS Time"));
  }

  Serial.println();
}

char letterize(int x)
{
  return (char)x + 65;
}

static char *get_mh(double lat, double lon, int size)
{
  static char locator[11];
  double LON_F[] = {20, 2.0, 0.083333, 0.008333, 0.0003472083333333333};
  double LAT_F[] = {10, 1.0, 0.0416665, 0.004166, 0.0001735833333333333};
  int i;
  lon += 180;
  lat += 90;

  if (size <= 0 || size > 10)
    size = 6;
  size /= 2;
  size *= 2;

  for (i = 0; i < size / 2; i++)
  {
    if (i % 2 == 1)
    {
      locator[i * 2] = (char)(lon / LON_F[i] + '0');
      locator[i * 2 + 1] = (char)(lat / LAT_F[i] + '0');
    }
    else
    {
      locator[i * 2] = letterize((int)(lon / LON_F[i]));
      locator[i * 2 + 1] = letterize((int)(lat / LAT_F[i]));
    }
    lon = fmod(lon, LON_F[i]);
    lat = fmod(lat, LAT_F[i]);
  }
  locator[i * 2] = 0;
  return locator;
}

#pragma endregion gps_functions

#pragma region rotary_encoder
// start rotary encoder functions
void init_rotary_encoder()
{
  // we must initialize rotary encoder
  rotaryEncoder.begin();
  rotaryEncoder.setup(readEncoderISR);

  // set boundaries and if values should cycle or not
  // in this example we will set possible values to the band frequencies limits;
  bool circleValues = false;
  rotaryEncoder.setBoundaries(freqswitch_low[current_band], freqswitch_high[current_band], circleValues); // minValue, maxValue, circleValues true|false (when max go to min and vice versa)
  rotaryEncoder.setEncoderValue(current_freq[current_band]);

  rotaryEncoder.disableAcceleration(); // acceleration is now enabled by default - disable if you dont need it
  rotaryEncoder.setAcceleration(250);  // or set the value - larger number = more accelearation; 0 or 1 means disabled acceleration
}

void on_button_short_click()
{
  Serial.println("button SHORT press ");
  handle_band_change();
}

void on_button_long_click()
{
  Serial.println("button LONG press ");
  txEnabled = !txEnabled;
  if (txEnabled)
  {
    Serial.println("Transmit Enabled");
  }
  else
  {
    Serial.println("Transmit Disabled");
  }
}

void handle_rotary_button()
{
  static unsigned long lastTimeButtonDown = 0;
  static bool wasButtonDown = false;

  bool isEncoderButtonDown = rotaryEncoder.isEncoderButtonDown();
  // isEncoderButtonDown = !isEncoderButtonDown; // uncomment this line if your button is reversed

  if (isEncoderButtonDown)
  {
    if (!wasButtonDown)
    {
      // start measuring
      lastTimeButtonDown = millis();
    }
    // else we wait since button is still down
    wasButtonDown = true;
    return;
  }

  // button is up
  if (wasButtonDown)
  {
    // click happened, lets see if it was short click, long click or just too short
    if (millis() - lastTimeButtonDown >= longPressAfterMiliseconds)
    {
      on_button_long_click();
    }
    else if (millis() - lastTimeButtonDown >= shortPressAfterMiliseconds)
    {
      on_button_short_click();
    }
  }

  wasButtonDown = false;
}

void handle_band_change()
{
  current_band++;
  Serial.printf("button clicked, new index = %d\n", current_band);
  if (current_band > bandswitch_max)
  {
    current_band = 0;
    Serial.println("band array limit reached roll over to index 0");
  }
  Serial.print("band changed to ");
  Serial.printf("%d meters\n", bandswitch[current_band]);
  bool circleValues = false;
  rotaryEncoder.setBoundaries(freqswitch_low[current_band], freqswitch_high[current_band], circleValues); // minValue, maxValue, circleValues true|false (when max go to min and vice versa)
  rotaryEncoder.setEncoderValue(current_freq[current_band]);
  target_freq = current_freq[current_band];
}

void rotary_loop()
{
  // dont print anything unless value changed
  if (rotaryEncoder.encoderChanged())
  {
    Serial.print("Encoder Value: ");
    Serial.println(rotaryEncoder.readEncoder());
    current_freq[current_band] = rotaryEncoder.readEncoder();
    target_freq = current_freq[current_band];
  }

  // if (rotaryEncoder.isEncoderButtonClicked())
  // {
  handle_rotary_button();
  //}
}

void IRAM_ATTR readEncoderISR()
{
  rotaryEncoder.readEncoder_ISR();
}
// end rotary encoder functions
#pragma endregion rotary_encoder

#pragma region si5351
// start si5351 functions
void init_si5351()
{
  // The crystal load value needs to match in order to have an accurate calibration
  si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0);

  // Start on target frequency
  si5351.set_correction(cal_factor, SI5351_PLL_INPUT_XO);
  si5351.set_pll(SI5351_PLL_FIXED, SI5351_PLLA);
  uint64_t si5352_freq = target_freq * freqMultiplier;
  si5351.set_freq(si5352_freq, SI5351_CLK0);
  si5351.output_enable(SI5351_CLK0, 0);
}

void set_si5351_freuency(long target_freq)
{
  uint64_t si5352_freq = target_freq * freqMultiplier;
  si5351.set_freq(si5352_freq, SI5351_CLK0);
}
// end si5351 functions
#pragma endregion si5351

#pragma region oled_display
// start oled display functions
void clearDisplay()
{
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
}

void showDisplay()
{
  display.display();
}

void displayFrequency(uint64_t frequency)
{
  // Serial.printf("display frequecy: %d\n", frequency);

  uint64_t freqMhz = frequency / FREQUENCY_DIVISOR_MHZ;
  char megaHz[4];
  sprintf(megaHz, "%d", freqMhz);

  frequency -= freqMhz * FREQUENCY_DIVISOR_MHZ;
  // Serial.printf("%d\n", frequency);

  uint64_t freqKhz = frequency / FREQUENCY_DIVISOR_KHZ;
  char kiloHz[4];
  sprintf(kiloHz, "%03d", freqKhz);

  frequency -= freqKhz * FREQUENCY_DIVISOR_KHZ;
  // Serial.printf("frequency %d\n", frequency);

  uint64_t freqHz = frequency * 10;
  char hertz[4];
  sprintf(hertz, "%03d", freqHz);

  char buffer[40];
  sprintf(buffer, "%s.%s,%s", megaHz, kiloHz, hertz);
  // Serial.println(buffer);

  display.setCursor(0, 5);
  display.setTextSize(2);
  display.printf(buffer);
}

void displayBand()
{
  display.setCursor(0, 23);
  display.setTextSize(1);
  display.printf("Band: %d meters", bandswitch[current_band]);
}

void displayMode()
{
  display.setCursor(0, 33);
  display.setTextSize(1);
  display.printf("Mode: %s", modes[current_mode]);
}

void displayTime()
{
  DateTime now = rtc.now();
  display.setCursor(0, 43);
  display.setTextSize(1);
  display.printf("Time: %02d:%02d:%02d", now.hour(), now.minute(), now.second());
}

void displayGridSquare(String gridSquare)
{
  display.setCursor(0, 53);
  display.setTextSize(1);
  if (txEnabled)
  {
    display.printf("GRID: %s TX", gridSquare);
  }
  else
  {
    display.printf("GRID: %s", gridSquare);
  }
}

void displayInTransmit()
{
  display.setCursor(0, 53);
  display.setTextSize(1);
  display.printf("TRANSMITTING");
}

void displayInitGPS()
{
  display.setCursor(0, 43);
  display.setTextSize(1);
  display.printf("INIT GPS");
}

void display_init_gps()
{
  clearDisplay();
  displayFrequency(target_freq);
  displayBand();
  displayMode();
  displayInitGPS();
  showDisplay();
}

void display_transmitting()
{
  clearDisplay();
  displayFrequency(target_freq);
  displayBand();
  displayMode();
  displayTime();
  displayInTransmit();
  showDisplay();
}

void display_loop()
{
  clearDisplay();
  displayFrequency(target_freq);
  displayBand();
  displayMode();
  displayTime();
  displayGridSquare(gridSquare);
  showDisplay();
}
// end oled display functions
#pragma endregion oled_display

#pragma region wspr_functions

// Loop through the string, transmitting one character at a time.
void jtTransmitMessage()
{
  Serial.println(F("Transmit WSPR Message"));

  uint8_t i;

  jtencode.wspr_encode(call, loc, dbm, tx_buffer);

  // Reset the tone to the base frequency and turn on the output
  si5351.set_clock_pwr(SI5351_CLK0, 1);
  si5351.output_enable(SI5351_CLK0, 1);
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA);
  digitalWrite(LED_PIN, HIGH);
  display_transmitting();

  for (i = 0; i < WSPR_SYMBOL_COUNT; i++)
  {
    // Thanks to https://github.com/W3PM/Auto-Calibrated-GPS-RTC-Si5351A-FST4W-and-WSPR-MEPT/blob/main/w3pm_GPS_FST4W_WSPR_V1_1a.ino
    unsigned long timer = millis();
    si5351.set_freq((target_freq * freqMultiplier) + (tx_buffer[i] * WSPR_TONE_SPACING), SI5351_CLK0);
    while ((millis() - timer) <= WSPR_DELAY)
    {
      __asm__("nop\n\t");
    };
  }

  Serial.println();

  // Turn off the output
  si5351.set_clock_pwr(SI5351_CLK0, 0);
  si5351.output_enable(SI5351_CLK0, 0);
  digitalWrite(LED_PIN, LOW);
  display_loop();
}

#pragma endregion wspr_functions