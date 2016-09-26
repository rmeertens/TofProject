  /* This example shows how to use continuous mode to take
range measurements with the VL6180X. It is based on
vl53l0x_ContinuousRanging_Example.c from the VL53L0X API.

The range readings are in units of mm. */

#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor;
VL53L0X second_sensor;
int second_sensor_pin = 17;
int led_pin=3;
int second_led_pin=7;
void SendArray(uint8_t *b, uint8_t array_width, uint8_t array_height)
{
  uint8_t code[4];
  code[0] = 0xff;
  code[1] = 0x00;
  code[2] = 0x00;
  code[3] = 0xAF; // 175
  Serial.write(code, 4);
    
  int horizontalLine = 0;
  for (horizontalLine = 0; horizontalLine < array_height; horizontalLine++) {
    code[3] = 0x80;//128
    Serial.write(code, 4);
    
    Serial.write(b + array_width * horizontalLine, array_width);
      

    code[3] = 0xDA;//218
    Serial.write(code, 4);
  }

  code[3] = 0xAB;
  Serial.write(code, 4);
}


void setup()
{
  pinMode(second_sensor_pin, OUTPUT);      // sets the digital pin as output
  pinMode(led_pin,OUTPUT);
  pinMode(second_led_pin,OUTPUT);
  digitalWrite(second_sensor_pin, LOW);  
  digitalWrite(led_pin,LOW);
  
  Serial.begin(115200);
  Wire.begin();
  sensor.init();
  sensor.setAddress(1);
  Serial.println("Just set address to");
  Serial.println(sensor.getAddress());
  sensor.setTimeout(500);

  // Start continuous back-to-back mode (take readings as
  // fast as possible).  To use continuous timed mode
  // instead, provide a desired inter-measurement period in
  // ms (e.g. sensor.startContinuous(100)).
  sensor.startContinuous(33);

  
  digitalWrite(second_sensor_pin, HIGH);   
  second_sensor.init();  
  second_sensor.setAddress(2);
  Serial.println(second_sensor.getAddress());
  second_sensor.setTimeout(500);
  second_sensor.startContinuous(33);
}

void loop()
{
uint16_t w1=sensor.readRangeContinuousMillimeters();
delay(100);
uint16_t w2=second_sensor.readRangeContinuousMillimeters();
int measurement_count=2;
uint16_t measurements[measurement_count];
measurements[0]=w1;
measurements[1]=w2;
uint8_t *ref = (uint8_t*)measurements;
SendArray(ref,measurement_count*sizeof(uint16_t),1);
if(w1 > 1000){
  digitalWrite(led_pin,LOW);
  
}
else{
  digitalWrite(led_pin,HIGH);
}
if(w2>1000){
  digitalWrite(second_led_pin,LOW);
}
else{
  digitalWrite(second_led_pin,HIGH);
}
}
