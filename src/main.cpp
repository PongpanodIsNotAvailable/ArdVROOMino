
#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <string.h>

LiquidCrystal_I2C lcd(0x27,  16, 4);

const unsigned long SAMPLE_PERIOD_MS = 10;  // 100 Hz
unsigned long lastSample = 0;
unsigned long lastLCDUpdate = 0;
unsigned long lastA0Sample = 0;
unsigned long lastRPMSample = 0;
unsigned long RPM = 0;

unsigned long currentSample;


int a0, a1, a2, a3;
float voltTPS = 0;
float fuelPresBar = 0;
float waterTempC = 0;
float voltWaterTemp = 0;
String samples[5];

void setup() {
  
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0,0);  
  Serial.begin(9600);

  
  pinMode(8, OUTPUT);

  // Wait briefly for USB serial (safe for Leonardo / Micro)
  unsigned long start = millis();
  while (!Serial && millis() - start < 2000) {
    lcd.print("Hello");
  }
  
  lcd.clear();
  
}

void loop() {
  
  digitalWrite(8,HIGH);

  currentSample = millis();
  
  if (currentSample - lastSample >= SAMPLE_PERIOD_MS) {
    lastSample = currentSample;
    
    // Read ADCs
    a0 = analogRead(A0);    // Tacho
    if(a0 - lastA0Sample >= 20){
      if(currentSample - lastRPMSample >= 10){
        RPM = 1000/(currentSample - lastRPMSample)*60;
      }
    }
    lastA0Sample = a0;
    lastRPMSample = currentSample;

    a1 = analogRead(A1);    // TPS
    voltTPS = a1*0.0049;
    
    a2 = analogRead(A2);    // Water Temp
    voltWaterTemp = a2*0.0049;

    // waterTempC = 1/(0.0014237 + 0.00024618*(1) + 0.00000010892 * (1));
    
    a3 = analogRead(A3);    // fuel pressure
    fuelPresBar = (a3 * 0.0049 - 0.5) * 0.65;
    

    // Serial.print(currentSample);
    // Serial.print(",");
    // Serial.print(a0);
    // Serial.print(",");
    // Serial.print(a1);
    // Serial.print(",");
    // Serial.print(",");
    // Serial.print(a2);
    // Serial.println(a3);    
}
  
  
  
  if(currentSample -lastLCDUpdate >= 1000){
    

    lastLCDUpdate = currentSample;
    
    lcd.setCursor(0,0); 
    lcd.print("RPM : "); lcd.print(a0);

    lcd.setCursor(0,1);
    lcd.print("TPS : "); lcd.print(voltTPS); lcd.print(" V ");
    
    lcd.setCursor(0,2);
    lcd.print("Temp : "); lcd.print(voltWaterTemp); lcd.print(" V ");

    lcd.setCursor(0,3);
    if(fuelPresBar >= 0){
      lcd.print("Fuel Pres : "); lcd.print(fuelPresBar); lcd.print(" BAR");
    }
    else{
      lcd.print("Fuel Pres :"); lcd.print(fuelPresBar); lcd.print(" BAR");
    }
  }
}