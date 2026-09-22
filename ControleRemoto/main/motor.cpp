/**********************************************************************************************/
/*                                                                                            */
/*                                                                                            */
/*        motor.cpp                                    Author  : Luiz Felipe                  */
/*                                                     Email   :                              */
/*                                                     address : DF, BRAZIL                   */
/*        Created: 2023/02/26          by Luiz F.                                             */
/*        Updated: 2023/03/31          by Luiz F.                                             */
/*                                                                       All rights reserved  */
/**********************************************************************************************/

#include "motor.h"
#include <Arduino.h>

Motor::Motor(int pin_R, int pin_L, int channelR, int channelL) {
    this->pin_R = pin_R;
    this->pin_L = pin_L;
    this->channelR = channelR;
    this->channelL = channelL;

    ledcSetup(channelR, PWM_MOTOR_FREQUENCY, PWM_MOTOR_RESOLUTION);
    ledcSetup(channelL, PWM_MOTOR_FREQUENCY, PWM_MOTOR_RESOLUTION);

    ledcAttachPin(pin_R, channelR);
    ledcAttachPin(pin_L, channelL);
}

void Motor::moveForward(int speed, int direction) {
    /*
        This method starts the motor and sets a speed and direction
      - the speed is given in an 8-bit hex value to modify the motor duty cicle
      - the 'forward' direction is given by a number, 1 to R direction and -1 to L direction
    */
    if (direction == 1){
        // Move motor to R direction
        ledcWrite(channelL, LOW);
        ledcWrite(channelR, speed);        
    }
    else if (direction == -1) {
        // Move motor to L direction
        ledcWrite(channelL, speed);
        ledcWrite(channelR, LOW);              
    }
}

void Motor::stop() {
    // This method stop both direction by setting duty cicle equal zero
    ledcWrite(channelR, 0x00);
    ledcWrite(channelL, 0x00);
}


