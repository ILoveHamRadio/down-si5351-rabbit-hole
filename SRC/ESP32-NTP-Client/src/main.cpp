#include <WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

#define LEAP_YEAR(Y) ((Y > 0) && !(Y % 4) && ((Y % 100) || !(Y % 400)))

// Replace with your network credentials
const char *ssid = "REPLACE_WITH_YOUR_SSID";
const char *password = "REPLACE_WITH_YOUR_PASSWORD";

// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

// Variables to save date and time
String formattedDate;
String dayStamp;
String timeStamp;

String getFormattedDate(NTPClient client);

void setup()
{
  // Initialize Serial Monitor
  Serial.begin(115200);
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  // Print local IP address and start web server
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  // Initialize a NTPClient to get time
  timeClient.begin();
  // Set offset time in seconds to adjust for your timezone, for example:
  // GMT +1 = 3600
  // GMT +8 = 28800
  // GMT -1 = -3600
  // GMT 0 = 0
  timeClient.setTimeOffset(3600);
}
void loop()
{
  timeClient.update();
  // The formattedDate comes with the following format:
  // 2018-05-28T16:00:13Z
  // We need to extract date and time
  formattedDate = getFormattedDate(timeClient);
  Serial.println(formattedDate);

  delay(1000);
}

// Based on https://github.com/PaulStoffregen/Time/blob/master/Time.cpp
// currently assumes UTC timezone, instead of using this->_timeOffset
String getFormattedDate(NTPClient client)
{
  unsigned long rawTime = (client.getEpochTime()) / 86400L; // in days
  unsigned long days = 0, year = 1970;
  uint8_t month;
  static const uint8_t monthDays[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

  while ((days += (LEAP_YEAR(year) ? 366 : 365)) <= rawTime)
    year++;
  rawTime -= days - (LEAP_YEAR(year) ? 366 : 365); // now it is days in this year, starting at 0
  days = 0;
  for (month = 0; month < 12; month++)
  {
    uint8_t monthLength;
    if (month == 1)
    { // february
      monthLength = LEAP_YEAR(year) ? 29 : 28;
    }
    else
    {
      monthLength = monthDays[month];
    }
    if (rawTime < monthLength)
      break;
    rawTime -= monthLength;
  }
  String monthStr = ++month < 10 ? "0" + String(month) : String(month);     // jan is month 1
  String dayStr = ++rawTime < 10 ? "0" + String(rawTime) : String(rawTime); // day of month
  return String(year) + "-" + monthStr + "-" + dayStr + "T" + client.getFormattedTime() + "Z";
}
