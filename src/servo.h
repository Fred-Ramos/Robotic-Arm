#ifndef SERVO_H
#define SERVO_H

#include <Arduino.h>
#include <RP2040_PWM.h>

#include <fsm.h>

//define ServoMotor PINS
int pinServo0 = 0;
int pinServo1 = 1;
int pinServo2 = 2;
int pinServo3 = 3;

//creates pwm instance
RP2040_PWM* PWM_Instance0;
RP2040_PWM* PWM_Instance1;
RP2040_PWM* PWM_Instance2;
RP2040_PWM* PWM_Instance3;

float frequency = 50;

typedef struct {
  //creates pwm instance
  RP2040_PWM* PWM_Instance;
  fsm fsm_servo; //each servo motor has its own state machine 
  float duty_max, duty_min, duty_current;
  int angle_max, angle_min, angle_previous, angle_current, angle_target;
  unsigned long angle_period;
} servo;

void set_angle(servo* motor, float angle){ //change angle of certain servo motor
 if( (angle >= motor->angle_min) && (angle <= motor->angle_max)){
  motor->angle_previous = motor->angle_current;
  motor->angle_current = angle;
  motor->duty_current = motor->duty_min + (motor->duty_max - motor->duty_min)/180 * angle; //angle to duty
 }
 else if (angle < motor->angle_min){
  motor->angle_previous = motor->angle_current;
  motor->angle_current = motor->angle_min;
  motor->duty_current = motor->duty_min + (motor->duty_max - motor->duty_min)/180 * motor->angle_min; //angle to duty
 }
 else if (angle > motor->angle_max){
  motor->angle_previous = motor->angle_current;
  motor->angle_current = motor->angle_max;
  motor->duty_current = motor->duty_min + (motor->duty_max - motor->duty_min)/180 * motor->angle_max; //angle to duty
 }
}

//DEFINE STATE MACHINES
servo* motor0;
servo* motor1;
servo* motor2;
servo* motor3;

//SERVO MOTORS SETUP
void setup_servo(){
  //initialize motors
  motor0 = new servo;
  motor1 = new servo;
  motor2 = new servo;
  motor3 = new servo;
  //save min/max duty of the 4 servos to use
  motor0->duty_min = 2.1;
  motor0->duty_max = 11.7;
  motor1->duty_min = 2.5;
  motor1->duty_max = 12.3;
  motor2->duty_min = 0.2;
  motor2->duty_max = 11.2;
  motor3->duty_min = 3.7;
  motor3->duty_max = 14.1;

  //define min/max angle for each motor
  motor0->angle_min = 0;    //arm pointing to the right
  motor0->angle_max = 180;  //arm ponting to the left
  motor1->angle_min = 90;   //arm 1st half is pointing up
  motor1->angle_max = 180;  //arm 1st half is pointing to the front
  motor2->angle_min = 90;   //arm second half is pointing down
  motor2->angle_max = 180;  //arm second helf is pointing to the front
  motor3->angle_min = 0;    //arm claw opened
  motor3->angle_max = 90;   //arm claw closed

  //put motors on minimum angle
  set_angle(motor0, motor0->angle_min); //main rotation
  set_angle(motor1, motor1->angle_min); //1st axis 
  set_angle(motor2, motor2->angle_max); //2nd axis
  set_angle(motor3, motor3->angle_min); //claw

  //set initial states for statemachines
  motor0->fsm_servo.new_state = 0;
  set_state(motor0->fsm_servo, motor0->fsm_servo.new_state);
  motor1->fsm_servo.new_state = 0;
  set_state(motor1->fsm_servo, motor1->fsm_servo.new_state);
  motor2->fsm_servo.new_state = 0;
  set_state(motor2->fsm_servo, motor2->fsm_servo.new_state);
  motor3->fsm_servo.new_state = 0;
  set_state(motor3->fsm_servo, motor3->fsm_servo.new_state);

  //assigns pins, with frequency of 50 Hz and a duty cycle defined by the inital angle
  motor0->PWM_Instance = new RP2040_PWM(pinServo0, frequency, motor0->duty_current);
  motor0->PWM_Instance->setPWM(pinServo0, frequency, motor0->duty_current);
  motor1->PWM_Instance = new RP2040_PWM(pinServo1, frequency, motor1->duty_current);
  motor1->PWM_Instance->setPWM(pinServo1, frequency, motor1->duty_current);
  motor2->PWM_Instance = new RP2040_PWM(pinServo2, frequency, motor2->duty_current);
  motor2->PWM_Instance->setPWM(pinServo2, frequency, motor2->duty_current);
  motor3->PWM_Instance = new RP2040_PWM(pinServo3, frequency, motor3->duty_current);
  motor3->PWM_Instance->setPWM(pinServo3, frequency, motor3->duty_current);
  //inital motor angle targets:
  motor0->angle_target = motor0->angle_current;  //doesnt move for now
  motor1->angle_target = motor1->angle_current; //doenst move
  motor2->angle_target = motor2->angle_current; //doesnt move
  motor3->angle_target = motor3->angle_current; //doesnt move
  //initial motor speeds
  motor0->angle_period = 100;
  motor1->angle_period = 30;
  motor2->angle_period = 30;
  motor3->angle_period = 5;
}

void fsm_servo_function(int servo_number){
  
  servo* motor_this = motor1; // Initialize motor_this to point to motor1
 switch (servo_number){
   case 0:
     motor_this = motor0;
     break;
   case 1:
     motor_this = motor1;
     break;
   case 2:
     motor_this = motor2;
     break;
   case 3:
     motor_this = motor3;
     break;
   default:
     Serial.print("Error motor index");
     break;
 }

  //UPDATE CURRENT TIS
  unsigned long curr_time = millis();
  motor_this->fsm_servo.tis = curr_time - motor_this->fsm_servo.tes;

  //CALCULATE NEW STATES
  if (motor_this->fsm_servo.state == 0 && motor_this->angle_current < motor_this->angle_target && motor_this->fsm_servo.tis > motor_this->angle_period){ //if target angle is higher then current angle, change to "Increase Angle" state
    motor_this->fsm_servo.new_state = 1;
  }
  else if(motor_this->fsm_servo.state == 0 && motor_this->angle_current > motor_this->angle_target && motor_this->fsm_servo.tis > motor_this->angle_period){ //if target angle is smaler then current angle, change to "Decrease Angle" state
    motor_this->fsm_servo.new_state = 2;
  }
  else if (motor_this->fsm_servo.state == 1 || motor_this->fsm_servo.state == 2){ //change back to stationary state
    motor_this->fsm_servo.new_state = 0;
  }

  //UPDATE STATES
  set_state(motor_this->fsm_servo, motor_this->fsm_servo.new_state);

  //CALCULATE OUTPUTS BASED ON NEW CURRENT STATE
  if (motor_this->fsm_servo.state == 1){
    set_angle(motor_this, motor_this->angle_current + 1); //changes the duty acording to the pretended angle
  }
  if (motor_this->fsm_servo.state == 2){   
    set_angle(motor_this, motor_this->angle_current - 1); //changes the duty acording to the pretended angle
  }

  //Serial.println("New angle is: " + String(motor_this->angle_current) + " //");

  //SET OUTPUT
  int pinServo = servo_number;
  if (motor_this->angle_current != motor_this->angle_previous){
    motor_this->PWM_Instance->setPWM(pinServo, frequency, motor_this->duty_current);
  }
}



#endif



