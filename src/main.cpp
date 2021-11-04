#include <Arduino.h>
#include "VescUart.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "ArduinoOTA.h"
#include "WiFi.h"
#include "ESPmDNS.h"
#include "Adafruit_NeoPixel.h"
#include "ESP32Tone.h"

//Notes for playing songs.

const int c = 261;
const int d = 294;
const int e = 329;
const int f = 349;
const int g = 391;
const int gS = 415;
const int a = 440;
const int aS = 455;
const int b = 466;
const int cH = 523;
const int cSH = 554;
const int dH = 587;
const int dSH = 622;
const int eH = 659;
const int fH = 698;
const int fSH = 740;
const int gH = 784;
const int gSH = 830;
const int aH = 880;

//TODO: wifi bootloader

#define REMOTE 17

#define BUZZPIN 18

#define SPEED 12
#define ANGLE 13

#define VESC_2_ID 86

#define RX_PIN 26   //GPIO 26
#define TX_PIN 25   //GPIO 25

Adafruit_NeoPixel strip1 = Adafruit_NeoPixel(13, 19, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip2 = Adafruit_NeoPixel(13, 23, NEO_GRB + NEO_KHZ800);

uint32_t remotevals[4] = {6342651, 3196923, 25217019, 12634107}; 
String buttons[4] = {"Lock", "Unlock", "Bell", "Power"};

enum {
  R_Lock = 0,
  R_Unlock,
  R_Bell,
  R_Power
};

// variables for key protocol interrupt.
uint32_t lastrise;
uint32_t lastfall;

volatile char remote_bit;
volatile uint32_t remote_buff;
volatile uint32_t last_buff;
volatile bool ISRupdate;

// Class for communicating with the VESC
VescUart vesc;

// bool to disable motors
bool VescOn = true;

bool mdns_setup = false;

float speed;
float angle;

// bool to turn off lights
int lightpattern = 1;

// Lock for Serial1, so that only one task uses it at a time.
SemaphoreHandle_t xVescUartLock;

// Task handle for motor control task
TaskHandle_t xMotorControlHandle;

// Task handle for key task
TaskHandle_t xKeyHandle;

// Task handle for key task
TaskHandle_t xLightsHandle;

void vMotorControl( void * pvParameters);
void vKey( void * pvParameters);
void vLights(void * pvParameters);

uint32_t Wheel(byte WheelPos);

void firstSection();
void secondSection();
void beep(int note, int duration);

void IRAM_ATTR remote_ISR(){
  int b = digitalRead(REMOTE);

  // Falling edge
  if(!b){
    // One if time since rising edge > 700 ns.
    if(micros() - lastrise > 700){
      remote_buff |= 1<<remote_bit;
      remote_bit += 1;
      lastfall = micros();
    // 0 if time since rising edge >100ns, <700ns
    } else if (micros() - lastrise > 100){
      remote_bit += 1;
      lastfall = micros();
    }
    
  // Rising edge
  }else{
    // if time since falling edge > 5ms, new message.
    if(micros() - lastfall > 5000){
      if(last_buff != remote_buff){
        for (int i = 0; i < 4; i++)
        {
          if(remote_buff == remotevals[i]){
            last_buff = i;
            ISRupdate = 1;
          }

        }
        
      }
      // reset buffers
      remote_buff = 0;
      remote_bit = 0;
    } else if (micros() - lastfall > 700){
      lastrise = micros();
    }
  }
}

void setup() {

  Serial.begin(115200);
  pinMode(5, OUTPUT);
  pinMode(SPEED, INPUT);
  pinMode(ANGLE, OUTPUT);
  pinMode(REMOTE, INPUT);
  
  pinMode(BUZZPIN, OUTPUT);

  lastrise = micros();
  lastfall = micros();

  /*

  WiFi.begin("CASELAB", "CaseLocalNet");

  delay(1000);

  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";
      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });
  
  ArduinoOTA.begin();

  */

  // Set up serial port for VESC communication
  Serial1.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);

  vesc.setSerialPort(&Serial1);

  xVescUartLock = xSemaphoreCreateMutex();

  // motor control task, 8 kb of ram, priority 10
  xTaskCreate(&vMotorControl, "motorControl", 8000, NULL, 10, &xMotorControlHandle);

  // key task, 1 kb bytes of ram, priority 5
  xTaskCreate(&vKey, "Key", 1000, NULL, 5, &xKeyHandle);

  attachInterrupt(REMOTE, remote_ISR, CHANGE);

  xTaskCreate(&vLights, "Lights", 1000, NULL, 2, &xLightsHandle);

}

void vMotorControl( void * pvParameters){
  while(1){
    speed = (analogRead(SPEED)) / 2048.0 - 0.96;  //scale from -1 to 1
    angle = (analogRead(ANGLE)) / 1024.0 - 1.78;  //scale from -1 to 1

    angle = constrain(angle, -1.5, 1.5);

    // 20% deadzone
    if(abs(speed) < 0.2){
      speed = 0;
    }

    // Throttle curve 
    speed = (speed*speed*speed + speed*0.4)/1.4;

    float leftspeed;
    float rightspeed;

    if (angle > 0) // turning right, slow right motor.
    {

      leftspeed = speed; 
      rightspeed = speed - angle*speed;

    } else {        //turn left, slow left motor.

      leftspeed = speed + angle*speed;;
      rightspeed = speed;

    }

    // make sure that no other task is using Serial1
    xSemaphoreTake(xVescUartLock, portMAX_DELAY);

    
    Serial.print(leftspeed);
    Serial.print("\t , \t");
    Serial.print(rightspeed);
    Serial.print("\t | \t");
    Serial.print(speed);
    Serial.print("\t , \t");
    Serial.println(angle);
    
    

    // Set speed
    
    if(VescOn){
      vesc.setDuty(leftspeed);
      vesc.setDuty(rightspeed, VESC_2_ID);
    }else{
      vesc.setDuty(0);
      vesc.setDuty(0, VESC_2_ID);
    }

    // return lock
    xSemaphoreGive(xVescUartLock);

    vTaskDelay( 50 / portTICK_PERIOD_MS );  
  }
}


void vKey( void * pvParameters){
  while (1)
  {
    if(ISRupdate){
      switch (last_buff)
      {
        case R_Lock:
          VescOn = false;
          digitalWrite(BUZZPIN, HIGH);
          delay(100);
          digitalWrite(BUZZPIN, LOW);
          break;

        case R_Unlock:
          digitalWrite(BUZZPIN, HIGH);
          delay(50);
          digitalWrite(BUZZPIN, LOW);
          delay(50);
          digitalWrite(BUZZPIN, HIGH);
          delay(50);
          digitalWrite(BUZZPIN, LOW);
          VescOn = true;
          break;

        case R_Bell:
          //Play first section
          firstSection();
        
          //Play second section
          secondSection();

          
          digitalWrite(BUZZPIN, HIGH);
          delay(100);
          digitalWrite(BUZZPIN, LOW);
          delay(300);

          digitalWrite(BUZZPIN, HIGH);
          delay(100);
          digitalWrite(BUZZPIN, LOW);
          delay(100);

          digitalWrite(BUZZPIN, HIGH);
          delay(100);
          digitalWrite(BUZZPIN, LOW);
          delay(100);

          digitalWrite(BUZZPIN, HIGH);
          delay(100);
          digitalWrite(BUZZPIN, LOW);
          delay(100);

          digitalWrite(BUZZPIN, HIGH);
          delay(100);
          digitalWrite(BUZZPIN, LOW);
          delay(300);

          digitalWrite(BUZZPIN, HIGH);
          delay(100);
          digitalWrite(BUZZPIN, LOW);
          delay(300);

          digitalWrite(BUZZPIN, HIGH);
          delay(100);
          digitalWrite(BUZZPIN, LOW);
          break;
        
        case R_Power:
          lightpattern++;

          digitalWrite(BUZZPIN, HIGH);
          delay(100);
          digitalWrite(BUZZPIN, LOW);
          delay(100);

          break;

        default:
          break;
      }

      vTaskDelay(500 / portTICK_PERIOD_MS);
      ISRupdate = 0; 
    }
    vTaskDelay(100 / portTICK_PERIOD_MS); 
  }
}


void vLights(void * pvParameters){

  uint8_t j = 0;

  while(1){
    if(lightpattern == 0){
      for(int i=0; i< strip2.numPixels(); i++) {
        strip1.setPixelColor(i, 0);
        strip2.setPixelColor(i, 0);
      }

      strip1.show();
      strip2.show();
    }else if(lightpattern == 1){

      for(int i=0; i< strip2.numPixels(); i++) {
        strip1.setPixelColor(i, Wheel(((i * 256 / strip1.numPixels()) + j) & 255));
        strip2.setPixelColor(i, Wheel(((i * 256 / strip2.numPixels()) + j) & 255));
      }

      strip1.show();
      strip2.show();
      j = j + 20;
    }else if(lightpattern == 2){

      for(int i=0; i< strip2.numPixels(); i++) {
        strip1.setPixelColor(i, Wheel(((i * 128 / strip1.numPixels()) + j) & 255));
        strip2.setPixelColor(i, Wheel(((-i * 128 / strip2.numPixels()) + j) & 255));
      }

      strip1.show();
      strip2.show();
      j = j + 10;

    }else if(lightpattern==3){
      for(int i=0; i< strip2.numPixels(); i++) {
        strip1.setPixelColor(i, Wheel(j));
        strip2.setPixelColor(i, Wheel(j+128));
      }
      strip1.show();
      strip2.show();
      j += 10;
    }else if(lightpattern==4){

      for(int i=0; i< strip2.numPixels(); i++) {
        strip1.setPixelColor(i, strip1.Color(((13-j/16)<i)*200, ((13-j/16)<i)*20, 0));
        strip2.setPixelColor(i, strip1.Color(((13-j/16)<i)*200, ((13-j/16)<i)*20, 0));
      }

      strip1.setPixelColor(13-j/16, strip1.Color(255, 0, 0));
      strip2.setPixelColor(13-j/16, strip1.Color(255, 0, 0));
      
      strip1.setPixelColor(14-j/16, strip1.Color(200, 10, 0));
      strip2.setPixelColor(14-j/16, strip1.Color(200, 10, 0));     

      strip1.show();
      strip2.show();
      vTaskDelay(30);
      j = j + speed*30 + 10;
     }else if(lightpattern == 5){

      uint32_t col = Wheel(random(255));

      for(int i=0; i< strip2.numPixels(); i++) {
        strip1.setPixelColor(i, col);
        strip2.setPixelColor(i, col);
      }

      strip1.show();
      strip2.show();
      vTaskDelay(300-abs(angle)*200);
      j = j + 10;
    }else{
      lightpattern=0;
    }
    vTaskDelay(30);
  }
}

void loop() {
  vTaskDelay(10000);
  /*
  if(WiFi.status() == WL_CONNECTED){
    
    if(!mdns_setup){
      if (!MDNS.begin("bobby") && !mdns_setup) { 
        Serial.println("Error setting up MDNS responder!");
        while (1) {
          delay(1000);
        }
      }
      mdns_setup=true;
    }
    
    ArduinoOTA.handle();
  }
  */

}

uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
    return strip1.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if(WheelPos < 170) {
    WheelPos -= 85;
    return strip1.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return strip1.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}


void beep(int note, int duration)
{
  //Play tone on buzzerPin
  tone(BUZZPIN, note, duration);
 
  vTaskDelay(duration/2);
 
  //Stop tone on buzzerPin
  noTone(BUZZPIN);
 
  vTaskDelay(10);
}


void firstSection()
{
  beep(a, 500);
  beep(a, 500);    
  beep(a, 500);
  beep(f, 350);
  beep(cH, 150);  
  beep(a, 500);
  beep(f, 350);
  beep(cH, 150);
  beep(a, 650);
 
  vTaskDelay(500);
 
  beep(eH, 500);
  beep(eH, 500);
  beep(eH, 500);  
  beep(fH, 350);
  beep(cH, 150);
  beep(gS, 500);
  beep(f, 350);
  beep(cH, 150);
  beep(a, 650);
 
  vTaskDelay(500);
}
 
void secondSection()
{
  beep(aH, 500);
  beep(a, 300);
  beep(a, 150);
  beep(aH, 500);
  beep(gSH, 325);
  beep(gH, 175);
  beep(fSH, 125);
  beep(fH, 125);    
  beep(fSH, 250);
 
  vTaskDelay(325);
 
  beep(aS, 250);
  beep(dSH, 500);
  beep(dH, 325);  
  beep(cSH, 175);  
  beep(cH, 125);  
  beep(b, 125);  
  beep(cH, 250);  
 
  vTaskDelay(350);
}