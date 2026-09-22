// /**********************************************************************************************/
// /*                                                                                            */
// /*                                                                                            */
// /*        led_rgb.cpp                                   Author  : Luiz Felipe                  */
// /*                                                     Email   :                              */
// /*                                                     address : DF, BRAZIL                   */
// /*        Created: 2023/02/25          by Luiz F.                                             */
// /*        Updated: 202/03/26          by Luiz F.                                              */
// /*                                                                       All rights reserved  */
// /**********************************************************************************************/


// #include "led_rgb.h"
// #include "behavior.h"
// #include <Arduino.h>

// LED_RGB::LED_RGB(int pin_red, int pin_green, int pin_blue) {
//   this->pin_red = pin_red;
//   this->pin_green = pin_green;
//   this->pin_blue = pin_blue;
  
//   ledcSetup(PWM_CHANNEL_LED_RED, PWM_FREQUENCY_LED, PWM_RESOLUTION_LED);
//   ledcSetup(PWM_CHANNEL_LED_GREEN, PWM_FREQUENCY_LED, PWM_RESOLUTION_LED);
//   ledcSetup(PWM_CHANNEL_LED_BLUE, PWM_FREQUENCY_LED, PWM_RESOLUTION_LED);

//   ledcAttachPin(pin_red, PWM_CHANNEL_LED_RED);
//   ledcAttachPin(pin_green, PWM_CHANNEL_LED_GREEN);
//   ledcAttachPin(pin_blue, PWM_CHANNEL_LED_BLUE);
// }

// void LED_RGB::turnOn(int cor) {
//   /*
//     This method receive a hexadecimal value to set a color in a LED_RGB object
//     obs: The color here is an value 24bits hexadecimal value 0 to 255
//   */
  
//   ledcWrite(PWM_CHANNEL_LED_RED, (cor >> 16) & 0xFF);
//   ledcWrite(PWM_CHANNEL_LED_GREEN, (cor >> 8) & 0xFF);
//   ledcWrite(PWM_CHANNEL_LED_BLUE, cor & 0xFF);
//   /* Esp8266
//   analogWrite(pin_red, (cor >> 16) & 0xFF);
//   analogWrite(pin_green, (cor >> 8) & 0xFF);
//   analogWrite(pin_blue, cor & 0xFF);
//   */
// }

// void LED_RGB::turnOff() {
//   /*
//     This method turns off the LED_RGB object
//   */
//   ledcWrite(PWM_CHANNEL_LED_RED, 0x00);
//   ledcWrite(PWM_CHANNEL_LED_GREEN, 0x00);
//   ledcWrite(PWM_CHANNEL_LED_BLUE, 0x00);
// }

// void LED_RGB::blink(int cor, int tempo_ligado, int tempo_desligado, int num_piscadas) {
//   /*
//     This method takes the led blink with a scpecif color, time and blink numbers
//   */
//   for (int i = 0; i < num_piscadas; i++) {
//     turnOn(cor);
//     delay(tempo_ligado);
//     turnOff();
//     delay(tempo_desligado);
//   }
// }
