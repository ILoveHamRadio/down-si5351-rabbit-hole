#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
/*
   This sample sketch demonstrates the normal use of a TinyGPSPlus (TinyGPSPlus) object.
   It requires the use of SoftwareSerial, and assumes that you have a
   9600-baud serial GPS device hooked up on pins 17(rx) and 16(tx).
*/
static const int RXPin = 17, TXPin = 16;
static const uint32_t GPSBaud = 9600;

void displayInfo();
static char *get_mh(double lat, double lon, int size);

// The TinyGPSPlus object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

void setup()
{
  Serial.begin(115200);
  ss.begin(GPSBaud);
}

void loop()
{
  // This sketch displays information every time a new sentence is correctly encoded.
  while (ss.available() > 0)
    if (gps.encode(ss.read()))
      displayInfo();

  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    while (true)
      delay(500);
  }
}

void displayInfo()
{
  Serial.print(F("Location: "));
  if (gps.location.isValid())
  {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
    Serial.print(F(", Grid Square: "));
    Serial.print(get_mh(gps.location.lat(), gps.location.lng(), 10));
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (gps.time.isValid())
  {
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
    Serial.print(gps.time.centisecond());
  }
  else
  {
    Serial.print(F("INVALID"));
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