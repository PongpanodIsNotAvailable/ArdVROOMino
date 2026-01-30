#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>

LiquidCrystal_I2C lcd(0x27, 16, 4);

const unsigned long SAMPLE_PERIOD_MS = 10; // 100 Hz
unsigned long lastSample = 0;
unsigned long lastLCDUpdate = 0;
unsigned long lastA0Sample = 0;
unsigned long lastRPMSample = 0;
unsigned long RPM = 0;
unsigned long currentSample;

// Variables for averaging
long sumA0 = 0, sumA1 = 0, sumA2 = 0, sumA3 = 0;
int sampleCount = 0;

float voltTPS = 0;
float fuelPresBar = 0;
float voltWaterTemp = 0;

void setup() {
  lcd.init();
  lcd.backlight();
  Serial.begin(9600);
  pinMode(8, OUTPUT);

  lcd.setCursor(0, 0);
  lcd.print("Initializing...");
  delay(1000);
  lcd.clear();
}

void loop() {
  digitalWrite(8, HIGH);
  currentSample = millis();

  // --- 1. SENSOR SAMPLING BLOCK (Every 10ms) ---
  if (currentSample - lastSample >= SAMPLE_PERIOD_MS) {
    lastSample = currentSample;

    int a0 = analogRead(A0);
    int a1 = analogRead(A1);
    int a2 = analogRead(A2);
    int a3 = analogRead(A3);

    // Accumulate sums for averaging
    sumA0 += a0;
    sumA1 += a1;
    sumA2 += a2;
    sumA3 += a3;
    sampleCount++;

    // RPM Calculation (keeping your original logic)
    if (a0 - lastA0Sample >= 20) {
      if (currentSample - lastRPMSample >= 10) {
        RPM = 1000 / (currentSample - lastRPMSample) * 60;
      }
    }
    lastA0Sample = a0;
    lastRPMSample = currentSample;

    // Optional: Keep serial fast for debugging
    Serial.print(a0); Serial.print(",");
    Serial.print(a1); Serial.print(",");
    Serial.print(a2); Serial.print(",");
    Serial.println(a3);
  }

  // --- 2. LCD UPDATE & AVERAGING BLOCK (Every 1000ms) ---
  if (currentSample - lastLCDUpdate >= 1000) {
    lastLCDUpdate = currentSample;

    if (sampleCount > 0) {
      // Calculate Means
      float avgA0 = (float)sumA0 / sampleCount;
      float avgA1 = (float)sumA1 / sampleCount;
      float avgA2 = (float)sumA2 / sampleCount;
      float avgA3 = (float)sumA3 / sampleCount;

      // Convert Averaged ADCs to Units
      voltTPS = avgA1 * 0.0049;
      // voltWaterTemp = (avgA2 * 0.0049 - 0.7) * 66.67;
      voltWaterTemp = readTemp(avgA2);
    
      fuelPresBar = (avgA3 * 0.0049) * 4.5;

      // Update LCD
      lcd.setCursor(0, 0);
      lcd.print("RPM : "); lcd.print((int)avgA0); lcd.print("    "); // Using avg ADC for RPM display based on your original code

      lcd.setCursor(0, 1);
      lcd.print("TPS : "); lcd.print(voltTPS, 2); lcd.print(" V  ");

      lcd.setCursor(0, 2);
      lcd.print("Temp: "); lcd.print(voltWaterTemp, 1); lcd.print(" C  ");

      lcd.setCursor(0, 3);
      lcd.print("Fuel: "); lcd.print(fuelPresBar, 2); lcd.print(" BAR ");

      // --- 3. RESET SUMS FOR NEXT AVERAGE ---
      sumA0 = 0; sumA1 = 0; sumA2 = 0; sumA3 = 0;
      sampleCount = 0;
    }
  }
}

float readTemp(int NTCPin){

  float ADCvalue;
  float Resistance;
  ADCvalue = analogRead(NTCPin);
  Serial.print("Analog value ");
  Serial.print(ADCvalue);
  Serial.print(" = ");
  //convert value to resistance
  Resistance = (1023 / ADCvalue) - 1;
  Resistance = 10000 / Resistance;
  Serial.print(Resistance);
  Serial.println(" Ohm");

  float steinhart;
  steinhart = Resistance / 10000; // (R/Ro)
  steinhart = log(steinhart); // ln(R/Ro)
  steinhart /= 3950; // 1/B * ln(R/Ro)
  steinhart += 1.0 / (25 + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart; // Invert
  steinhart -= 273.15; // convert to C

  Serial.print(steinhart);
  return steinhart;
}