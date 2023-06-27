#include <Arduino.h>

void setup()
{
  Serial.begin(115200);
  Serial.println("");
  Serial.println("Hello from the Setup Function");
}

void loop()
{
  Serial.println("Hello from the Loop");
  // wait for a second
  delay(1000);
}
