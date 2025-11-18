#include <Arduino.h>

#include <Wire.h>
#include <SPI.h>

#include <math.h>
#include <Keyboard.h>

#include <fsm.h>
#include <tof.h>
#include <tcs.h>
#include <servo.h>
#include <robotarm.h>

//WORKING MODE

extern char robot_mode;

//servo variables
extern float frequency;
extern unsigned long angle_period;

//DISTANCE SENSOR VARIABLES
extern VL53L0X tof;
extern float measurement, distance, prev_distance;

extern unsigned long interval_tof;
extern unsigned long currentMicros_tof, previousMicros_tof;

//COLOR SENSOR VARIABLES
extern double r_perc, g_perc, b_perc;
extern Adafruit_TCS34725 tcs;

//ServoMotor PINS
extern int pinServo0;
extern int pinServo1;
extern int pinServo2;
extern int pinServo3;

//STATE MACHINES
extern servo* motor0;
extern servo* motor1;
extern servo* motor2;
extern servo* motor3;


void setup() 
{
  //SERIAL  Serial.begin(115200);//start serial port
  //DISTANCE SENSOR
  Wire.setSDA(16); //connect VL53L0X SDA to pin gpio 16
  Wire.setSCL(17); //connect VL53L0X SCL to pin gpio 17
  Wire.begin();    //IC2 wire 0
  setup_VL53L0X(); //setup distance sensor
  //COLOR   
  Wire1.setSDA(14);  // Connect TCS34725 SDA to gpio 19
  Wire1.setSCL(15);  // Connect TCS34725 SCL to gpio 20
  Wire1.begin();     //IC2 wire 1
  setup_TCS34725();  //setup color sensor
  //SERVOS
  setup_servo();   //setup servo motors
  
  fsm_robotarm3.new_state = 0;
  set_state(fsm_robotarm3, fsm_robotarm3.new_state);
  fsm_robotarm1.new_state = 0;
  set_state(fsm_robotarm1, fsm_robotarm1.new_state);
  //DELAY BEFORE STARTING
  delay(2500);
} 

void loop() 
{  
  if (robot_mode == '0'){
    if (Serial.available() > 0) {
      // read the incoming byte:
      char received = Serial.read();

      // say what you got:
      switch(received){
        case '1':
        j_piece = -1;
        i_piece = -1;
        fsm_robotarm1.new_state = 1;
        set_state(fsm_robotarm1, fsm_robotarm1.new_state);
        robot_mode = received;
        Serial.print("Robot mode: ");
        Serial.println(robot_mode);
        break;

        case '2':
        j_piece = -1;
        i_piece = -1;
        number_pieces = -1;
        given_pieces = 0;
        sorted_pieces = 0;
        fsm_robotarm2.new_state = 100;
        set_state(fsm_robotarm2, fsm_robotarm2.new_state);
        robot_mode = received;
        Serial.print("Robot mode: ");
        Serial.println(robot_mode);
        break;

        case '3':
        fsm_robotarm3.new_state = 1;
        set_state(fsm_robotarm3, fsm_robotarm3.new_state);
        robot_mode = received;
        Serial.print("Robot mode: ");
        Serial.println(robot_mode);
        break;

        default:

        break;
      }
   }
  }
  else if (robot_mode == '1'){
    //Robot arm state machine
    fsm_robotarm_function1();

    //SERVO MOTORS STATE MACHINES
    fsm_servo_function(0); //state machine function of servo motor 0
    fsm_servo_function(1); //state machine function of servo motor 1
    fsm_servo_function(2); //state machine function of servo motor 2
    fsm_servo_function(3); //state machine function of servo motor 2
    //SEE LATER, SERVO1 AND 2 CANT BE BOTH AT 90 DEGREES (articulacao do 1 fica no caminho do 2, nada de especial)

    //Restart program
    if (Serial.available() > 0) {
      // read the incoming byte:
      char received = Serial.read();

      // say what you got:
      if (received == 'R' || received == 'r'){
        Serial.println("Restarting robot");
        fsm_robotarm1.new_state = 0; //VER ISTOOOOO
        set_state(fsm_robotarm1, fsm_robotarm1.new_state);
      }
   }
  }
  else if (robot_mode == '2'){
    //Robot arm state machine
    fsm_robotarm_function2();

    //SERVO MOTORS STATE MACHINES
    fsm_servo_function(0); //state machine function of servo motor 0
    fsm_servo_function(1); //state machine function of servo motor 1
    fsm_servo_function(2); //state machine function of servo motor 2
    fsm_servo_function(3); //state machine function of servo motor 2
    //SEE LATER, SERVO1 AND 2 CANT BE BOTH AT 90 DEGREES (articulacao do 1 fica no caminho do 2, nada de especial)

    //Restart program
    if (Serial.available() > 0) {
      // read the incoming byte:
      char received = Serial.read();

      // say what you got:
      if (received == 'R' || received == 'r'){
        Serial.println("Restarting robot");
        fsm_robotarm2.new_state = 0; //VER ISTOOOOO
        set_state(fsm_robotarm2, fsm_robotarm2.new_state);
      }
   }
  }
  else if (robot_mode == '3'){
    //Robot arm state machine
    fsm_robotarm_function3();

    //SERVO MOTORS STATE MACHINES
    fsm_servo_function(0); //state machine function of servo motor 0
    fsm_servo_function(1); //state machine function of servo motor 1
    fsm_servo_function(2); //state machine function of servo motor 2
    fsm_servo_function(3); //state machine function of servo motor 2
    //SEE LATER, SERVO1 AND 2 CANT BE BOTH AT 90 DEGREES (articulacao do 1 fica no caminho do 2, nada de especial)

    //Restart program
    if (Serial.available() > 0) {
      // read the incoming byte:
      char received = Serial.read();

      // say what you got:
      if (received == 'R' || received == 'r'){
        Serial.print("Stop robot");
        fsm_robotarm3.new_state = 0; //VER ISTOOOOO
        set_state(fsm_robotarm3, fsm_robotarm3.new_state);
      }
   }
  }
}



    //for (float i = 8; i > 2 ; i-=0.1){
    //  motor2->PWM_Instance->setPWM(pinServo2, frequency, i);
    //  Serial.println(i);
    //  delay(1000);
    //}