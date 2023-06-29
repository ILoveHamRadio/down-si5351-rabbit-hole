#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// global definitions
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Defines for Frequency Display
#define FREQUENCY_DIVISOR_HZ 100
#define FREQUENCY_DIVISOR_KHZ 100000
#define FREQUENCY_DIVISOR_MHZ 100000000

// global variables
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

const int bandswitch[] = { 160,80,40,20,15,10 };
const String modes[] = {"USB", "LSB", "CW", "WSPR", "FT8", "FT4"};
const uint64_t freqswitch_low[] = { 180000000, 350000000, 700000000, 1400000000, 2100000000, 2800000000 };
const uint64_t freqswitch_high[] = { 188000000, 380000000, 720000000, 1435000000, 2145000000, 290000000 };

int current_band = 5;
int current_mode = 4;
uint64_t  current_freq[] = { 180000000,350000000,700000000,1400000000,2100000000,2800000000 };
uint64_t target_freq;
uint64_t frequency_step = 100;

// display functions
void clearDisplay();
void showDisplay();
void displayFrequency(uint64_t frequency);
void displayBand();
void displayMode();

void setup() {
  Serial.begin(115200);

  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  delay(2000);
  target_freq = current_freq[current_band];
}

void loop() {
  clearDisplay();
  displayFrequency(target_freq);
  displayBand();
  displayMode();
  showDisplay();
  delay(10000);
}

void clearDisplay(){
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
}

void showDisplay(){
  display.display();
}

void displayFrequency(uint64_t frequency){
  Serial.printf("display frequecy: %d\n", frequency);

  uint64_t freqMhz = frequency / FREQUENCY_DIVISOR_MHZ;
  char megaHz[4];
  sprintf(megaHz, "%d", freqMhz);

  frequency -= freqMhz * FREQUENCY_DIVISOR_MHZ;
  Serial.printf("%d\n", frequency);

  uint64_t freqKhz = frequency / FREQUENCY_DIVISOR_KHZ;
  char kiloHz[4];
  sprintf(kiloHz, "%03d", freqKhz);

  frequency -=  freqKhz * FREQUENCY_DIVISOR_KHZ;
  Serial.printf("frequency %d\n", frequency);

  uint64_t freqHz = frequency / FREQUENCY_DIVISOR_HZ;
  char hertz[4];
  sprintf(hertz, "%03d", freqHz);

  char buffer[40];
  sprintf(buffer, "%s.%s,%s", megaHz, kiloHz, hertz); 
  Serial.println(buffer);

  display.setCursor(0, 10);
  display.setTextSize(2);
  display.printf(buffer);
}

void displayBand(){
  display.setCursor(0, 28);
  display.setTextSize(1);
  display.printf("Band: %d meters",  bandswitch[current_band]);
}

void displayMode(){
  display.setCursor(0, 38);
  display.setTextSize(1);
  display.printf("Mode: %s",  modes[current_mode]);
}