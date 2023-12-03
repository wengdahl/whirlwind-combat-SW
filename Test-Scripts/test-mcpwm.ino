#include "driver/mcpwm.h"
#include <Adafruit_NeoPixel.h>
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"

#define GPIO_PWM0A_OUT 1   //HV
#define GPIO_PWM1A_OUT 2   //HV

#define PIN_NEOPIXEL   4 // On Trinket or Gemma, suggest changing this to 1

int freq0 = 50; // 50 nominal, 500 theoretical limit
int freq1 = 1000;
float duty0 = 25;
float duty1 = 25;

bool pwm0 = false;
bool pwm1 = false;

// When setting up the NeoPixel library, we tell it how many pixels,
// and which pin to use to send signals. Note that for older NeoPixel
// strips you might need to change the third parameter -- see the
// strandtest example for more information on possible values.
Adafruit_NeoPixel pixels(3, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

void setuppwm(){
   mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_PWM0A_OUT);
   mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, GPIO_PWM1A_OUT);
   
 mcpwm_config_t pwm_config0;
     pwm_config0.frequency = freq0;  //frequency 
     pwm_config0.cmpr_a = 0;      //duty cycle of PWMxA = 50.0%
     pwm_config0.cmpr_b = 0;      //duty cycle of PWMxB = 50.0%
     pwm_config0.counter_mode = MCPWM_UP_COUNTER; // Up-down counter (triangle wave)
     pwm_config0.duty_mode = MCPWM_DUTY_MODE_0; // Active HIGH
     
   mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config0);    //Configure PWM0A & PWM0B with above settings
   mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config0);    //Configure PWM0A & PWM0B with above settings
   
   delay(20);
   mcpwm_set_timer_sync_output(MCPWM_UNIT_0, MCPWM_TIMER_0,MCPWM_SWSYNC_SOURCE_TEZ);
   mcpwm_sync_enable(MCPWM_UNIT_0, MCPWM_TIMER_0,MCPWM_SELECT_TIMER0_SYNC, 0);   
   mcpwm_sync_enable(MCPWM_UNIT_0, MCPWM_TIMER_1,MCPWM_SELECT_TIMER0_SYNC, 500); 

   mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, 1500);
   mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_GEN_A, 1500);

   delay(1000);

 }
void setup() {
 setuppwm();

 pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
}

int duty_us = 1500;

void loop(){
  
  pixels.clear();

  // pixels.Color() takes RGB values, from 0,0,0 up to 255,255,255
  // Here we're using a moderately bright green color:
  pixels.setPixelColor(duty_us%3, pixels.Color(0, 150, 0));

  pixels.show();   // Send the updated pixel colors to the hardware.


  duty_us++;
  if(duty_us > 2000){
    duty_us = 1100;
  }
  mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, duty_us);
  mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_GEN_A, duty_us);
  
  delay(60);

  if(duty_us == 1500){
    delay(10000);
  }
}