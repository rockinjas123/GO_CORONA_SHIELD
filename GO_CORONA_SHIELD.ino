// defining the parameters of the screen
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64// OLED display height, in pixels


const int sensorPin = A0; 
float sensorValue;
float voltageOut;

float temperatureC;
float temperatureF;

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

int pir_pin=7;
int state = LOW;// by default, no motion detected
int val = 0;


int buzz_pin=5;
int led_pin=6;

uint32_t delayMS;

void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);

   if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
   // Address 0x3D for 128x64
   Serial.println(F("SSD1306 allocation failed"));
   for(;;);
  }
  delay(2000);
  display.clearDisplay();

  pinMode(led_pin,OUTPUT);
  pinMode(pir_pin,INPUT);
  pinMode(buzz_pin,OUTPUT);
  pinMode(sensorPin, INPUT);


  // Initialize device.
  display.println(F("WELCOME USER"));
 
  display.display(); 
  delay(500);
  display.clearDisplay();
}

void loop() {
  // put your main code here, to run repeatedly:
  sensorValue = analogRead(sensorPin);
  voltageOut = (sensorValue * 5000) / 1024;
  
  // calculate temperature for LM35 (LM35DZ)
  temperatureC = voltageOut / 10;
  temperatureF = (temperatureC * 1.8) + 32;

  val = digitalRead(pir_pin);   // read sensor value
  if (val == HIGH) {           // check if the sensor is HIGH
    digitalWrite(led_pin, HIGH); // turn LED ON
    digitalWrite(buzz_pin, HIGH);// turn buzzer ON
    delay(100);                // delay 100 milliseconds 
    
    if (state == LOW) {
      display.print("Follow social distancing");
      delay(2000);
      display.display();
      state = HIGH;// update variable state to HIGH

      }
  }
  else {
      digitalWrite(led_pin, LOW); // turn LED OFF
      digitalWrite(buzz_pin, LOW);// turn buzzer OFF
      delay(200);             // delay 200 milliseconds 
      
      if (state == HIGH){
        display.println("Temperature(ºC): ");
        display.print(temperatureC);
        display.println("  Temperature(ºF): ");
        display.print(temperatureF);
        delay(1000);
        display.display();
        state = LOW;       // update variable state to LOW
    }
  }
  
  
  delay(1000);
  display.display(); 
  delay(500);
  display.clearDisplay();
}
