#include <QMC5883LCompass.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);
QMC5883LCompass compass;

void setup() {
  Serial.begin(115200);
   lcd.init();
  lcd.backlight();
  compass.init();
}

void loop() {
  // Read compass values
  compass.read();
  int compass_value = 360 - map(compass.getAzimuth(), -180, 180, 0, 360);
  Serial.println(compass_value);

  lcd.setCursor(0, 0);
  lcd.print("compass : ");
  lcd.setCursor(0, 11);
  lcd.print(compass_value);
  delay(250);
}
