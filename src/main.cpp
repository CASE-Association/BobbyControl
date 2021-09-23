#include <Arduino.h>
#include "VescUart.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

//TODO: wifi bootloader

#define REMOTE 17

#define BUZZPIN 18

#define SPEED 12
#define ANGLE 13

#define VESC_2_ID 86

#define RX_PIN 26   //GPIO 26
#define TX_PIN 25   //GPIO 25


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

// Lock for Serial1, so that only one task uses it at a time.
SemaphoreHandle_t xVescUartLock;

// Task handle for motor control task
TaskHandle_t xMotorControlHandle;

// Task handle for key task
TaskHandle_t xKeyHandle;

void vMotorControl( void * pvParameters);
void vKey( void * pvParameters);

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

  // Set up serial port for VESC communication
  Serial1.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);

  vesc.setSerialPort(&Serial1);

  xVescUartLock = xSemaphoreCreateMutex();

  // motor control task, 8 kb of ram, priority 10
  xTaskCreate(&vMotorControl, "motorControl", 8000, NULL, 10, &xMotorControlHandle);

  // key task, 1 kb bytes of ram, priority 5
  xTaskCreate(&vKey, "Key", 1000, NULL, 5, &xKeyHandle);

  attachInterrupt(REMOTE, remote_ISR, CHANGE);
}

void vMotorControl( void * pvParameters){
  while(1){
    float speed = (analogRead(SPEED)) / 2048.0 - 0.96;  //scale from -1 to 1
    float angle = (analogRead(ANGLE)) / 1024.0 - 1.78;  //scale from -1 to 1

    angle = constrain(angle, -1.5, 1.5);

    // 10% deadzone
    if(abs(speed) < 0.1){
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

    /*
    Serial.print(leftspeed);
    Serial.print("\t , \t");
    Serial.print(rightspeed);
    Serial.print("\t | \t");
    Serial.print(speed);
    Serial.print("\t , \t");
    Serial.println(angle);
    */
    

    // Set speed
    
    if(VescOn){
      vesc.setCurrent(20*leftspeed);
      vesc.setCurrent(20*rightspeed, VESC_2_ID);
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
          //TODO: Lights
          digitalWrite(BUZZPIN, HIGH);
          delay(500);
          digitalWrite(BUZZPIN, LOW);
          delay(100);

          digitalWrite(BUZZPIN, HIGH);
          delay(500);
          digitalWrite(BUZZPIN, LOW);
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

void loop() {
  vTaskDelay(portMAX_DELAY);
}