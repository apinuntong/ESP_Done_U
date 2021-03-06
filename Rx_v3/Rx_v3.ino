// stable 2.2.0  //

#include <ESP8266WiFi.h>
#include <WiFiUDP.h>
#include <EEPROM.h>
#include <Ticker.h>
#include <Wire.h>
#include <VL53L0X.h>
//kp = 20.3  ki= 1.7  kd = 20.3 
float kpt = 4.0, kit = 0.4, kdt = 4.0;
int zxcv =0;
float Input, Output;
float errSum, dErr, error, lastErr;
float ref_t=0.0;
float duration1, distance1,distance2,distance3;
int sen_b,sen;
#include "wifi_fw.h"
VL53L0X sensor;
float cmdistance,cmdistance_tmp;
 typedef struct
{
    float X2, X1, Y2, Y1, Y0;
} filted_data;
filted_data data1;
float Butterworth_filter_fc5_fs30(filted_data *filted, float x_data)
{
  // lowpass filter butterworth 
  filted->Y2 = filted->Y1;
  filted->Y1 = filted->Y0;
  
  float nume = (x_data + 2.0f * filted->X1 + filted->X2);
  float denom = (-1.4189826522181201f * filted->Y1 +  0.55326988968868296f * filted->Y2);
  filted->Y0 = 0.033571809367640711f * nume - denom;
  
  filted->X2 = filted->X1;
  filted->X1 = x_data;
  

  
  return filted->Y0;
}
float limit(float input, int min_limit, int max_limit);
float smooth(float alfa, float new_data, float old_data);
void tong();
unsigned long previousMillis = 0; 
float EMA_a = 0.3;    //initialization of EMA alpha
int EMA_S = 0;        //initialization of EMA S
int highpass = 0;

void setup()
{
  Wire.begin();
  sensor.init();
  sensor.setTimeout(500);
  sensor.startContinuous();
  init_drone();
  
}
  
void loop()
{
  Read_udp();
  delay(0);
    unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= 33) {
    previousMillis = currentMillis;
    tong();
  }


}
void myPID()
{
  lastErr = error;
  error = ref_t - sen*0.1;
  dErr = (error - lastErr) * 2.0f;
  errSum = limit(errSum + (error * 0.02f), 0, 300);

  Input = limit(((kpt*0.02f) * error) + (kit * errSum) + (kdt * dErr),0,100);
         if(abs(errSum)>10){
    zxcv = 40;
   }else{
    zxcv=0;
   }
   if(ref_t<=5){
    Input = 0;
    zxcv=0;
   }

}
float limit(float input, int min_limit, int max_limit)
{
  if (input > max_limit)input = max_limit;
  if (input < min_limit)input = min_limit;
  return input;
}
float smooth(float alfa, float new_data, float old_data)
{
  return (old_data + alfa * (new_data - old_data));
}
void tong(){
  sen = sensor.readRangeContinuousMillimeters();
  if(sen>1200){
    sen = sen_b;
  }else{
    sen_b = sen;
  }
  Serial.println(sen*0.1f);
  myPID();


}

