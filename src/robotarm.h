#ifndef ROBOTARM_H
#define ROBOTARM_H

#include <Arduino.h>
#include <fsm.h>
#include <tof.h>
#include <tcs.h>
#include <servo.h>

#include <math.h>

//robotarm variables
char robot_mode = '0';

//BOXES LOCATIONS
float mu_blue_box = 1;
float theta_blue_box = 136;
float phi_blue_box = 180;

float mu_yellow_box = 3;
float theta_yellow_box = 90;
float phi_yellow_box = 180;

float mu_red_box = 15;
float theta_red_box = 150;
float phi_red_box = 180;

float mu_green_box = 23;
float theta_green_box = 106;
float phi_green_box = 166;

//MODE 1

int i_piece = -1;
int j_piece = -1;
char true_received = 'N';
float r;
float mu_1;

//MODE 2
int number_pieces = -1;
int given_pieces = 0;
int sorted_pieces = 0;
int piece_positions[9][2];
int x_this;
int y_this;


//MODE 3
float x_detect;
float x_detect_max = 23.8; //15 centimeters from axis
float x_detect_min = 4;
float theta, phi, mu;

float mu_color0 = 173;
float theta_color0 = 103;
float phi_color0 = 122;

float mu_color1 = 174;
float theta_color1 = 131;
float phi_color1 = 91;

String color_piece;


fsm fsm_robotarm1; //given position, 1 piece
fsm fsm_robotarm2; //GRID 
fsm fsm_robotarm3; //searching for piece//getting piece//detecting color//placing piece//

void angles_xy(float x, float y){
    float l = 8;

    float alpha = atan(y/x);
    float beta = acos(sqrt(pow(x, 2) + pow(y, 2))/(2*l));

    if (isinf(beta)){
      theta = 180;  
      phi = 180;
    } 
    else{
      theta = (M_PI - alpha - beta)*(180/(M_PI));
      phi = (M_PI + alpha - beta)*(180/(M_PI));
    }
    
    mu = motor0->angle_current + 2; //add 1 to main axis angle (calibration, the sensor is not perfectly alined)

    Serial.print("X_d = " + String(distance) + " || X = " + String(x) + " || Theta = " + String(theta) + " || Phi = " + String(phi));
}

void grid_polar(int x, int y){
  float x_real = (x-4)*2;
  float y_real = 1 + (y)*2;
  Serial.print("x_real: " + String(x_real));
  Serial.print(" y_real: " + String(y_real));
  r = sqrt(pow(x_real , 2) + pow(y_real, 2)) - 2.7;
  mu_1 = atan(x_real/y_real);
  if (mu_1 > 0){
    mu_1 = 90 + mu_1*180/M_PI - 5;
  }
  else{
    mu_1 = 90 + mu_1*180/M_PI + 5;
  }
  Serial.print(" r: " + String(r) + "Mu: " + String(mu_1) + "||||||||||||||||||");
}

void fsm_robotarm_function3(){

  //UPDATE CURRENT TIS
  unsigned long curr_time = millis();
  fsm_robotarm3.tis = curr_time - fsm_robotarm3.tes;

  // READ INPUTS
  //measure distance 
  if (fsm_robotarm3.state == 1 || fsm_robotarm3.state == 2){
    tof_measure();
  }

  //CALCULATE NEW STATES

  int minangle = 60;
  int maxangle = 130;

  if (fsm_robotarm3.state == 0 && motor0->angle_current == motor0->angle_min && motor1->angle_current == motor1->angle_min && motor2->angle_current == motor2->angle_max && motor3->angle_current == motor3->angle_min){
    Serial.println("Robot restarted");
    robot_mode = '0';
    Serial.print("Robot mode: ");
    Serial.println(robot_mode);
  }
  
  if ((fsm_robotarm3.state == 1 || fsm_robotarm3.state == 2) && motor0->angle_current <= maxangle && motor0->angle_current >= minangle && x_detect_min <= distance && distance <= x_detect_max ){ //if in probing mode and finds object, changes to state 3
    Serial.println("OBJECT FOUND go to top of piece");
    fsm_robotarm3.new_state = 3;
    x_detect = (distance - 5.5)  - 1.3 - 4.3;
    angles_xy(x_detect , 3); //calculate angles we want the motors 1 and 2 to have
  }
  else if (fsm_robotarm3.state == 1 && motor0->angle_current == maxangle){ //invert rotation
    fsm_robotarm3.new_state = 2;
  }
  else if (fsm_robotarm3.state == 2 && motor0->angle_current == minangle){ //invert rotation
    fsm_robotarm3.new_state = 1;
  }
  else if (fsm_robotarm3.state == 3 && motor1->angle_current == motor1->angle_target && motor2->angle_current == motor2->angle_target){
    fsm_robotarm3.new_state = 31;
    angles_xy(x_detect , -2); //calculate angles we want the motors 1 and 2 to have
    Serial.println("Go to piece ");    
  }
  else if (fsm_robotarm3.state == 31 && motor1->angle_current == motor1->angle_target && motor2->angle_current == motor2->angle_target){
    fsm_robotarm3.new_state = 4;
    Serial.println("Let's close claw");    
  }
  else if (fsm_robotarm3.state == 4 && motor3->angle_current == motor3->angle_target){
    fsm_robotarm3.new_state = 5;
    Serial.println("Go to top of color sensor");
  }

  else if (fsm_robotarm3.state == 5 && motor0->angle_current == motor0->angle_target && motor1->angle_current == motor1->angle_target && motor2->angle_current == motor2->angle_target){
    fsm_robotarm3.new_state = 6;
    Serial.println("Go to color sensor");
  }
  else if (fsm_robotarm3.state == 6 && motor0->angle_current == motor0->angle_target && motor1->angle_current == motor1->angle_target && motor2->angle_current == motor2->angle_target){
    fsm_robotarm3.new_state = 7;
    Serial.println("measure color");
    color_piece = "y";
  }
  else if (fsm_robotarm3.state == 7 && fsm_robotarm3.tis > 1000){
    fsm_robotarm3.new_state = 8;
    Serial.println("Go to top of boxes");
  }
  else if (fsm_robotarm3.state == 8 && motor0->angle_current == motor0->angle_target && motor1->angle_current == motor1->angle_target && motor2->angle_current == motor2->angle_target){
    fsm_robotarm3.new_state = 9;
    Serial.println("Go to box");
  }
  else if (fsm_robotarm3.state == 9 && motor0->angle_current == motor0->angle_target &&  motor1->angle_current == motor1->angle_target && motor2->angle_current == motor2->angle_target){
    fsm_robotarm3.new_state = 10;
    Serial.println("drop piece");
  }  
  

  //UPDATE STATES
  set_state(fsm_robotarm3, fsm_robotarm3.new_state);
  //Serial.println("New state is " + String(fsm_robotarm3.state) + " angle is " + String(motor0->angle_current) + "target is " + String(motor0->angle_target));

  //CALCULATE OUTPUTS BASED ON NEW CURRENT STATE
  switch (fsm_robotarm3.state){
  case 0:
    motor0->angle_target = motor0->angle_min;
    motor1->angle_target = motor1->angle_min;
    motor2->angle_target = motor2->angle_max;
    motor3->angle_target = motor3->angle_min;
    break;
  case 1:
  motor0->angle_period = 100;
    if (motor0->angle_target == motor0->angle_current){ //if in increasing probing mode, increase target angle by 1
        motor0->angle_target = motor0->angle_current + 1;
      }
    motor1->angle_target = motor1->angle_min; //probing position
    motor2->angle_target = motor2->angle_max; //probing position
    motor3->angle_target = motor3->angle_min; //probing position
    break;
  
  case 2:
  motor0->angle_period = 100;
    if (motor0->angle_target == motor0->angle_current){ //if in decreasing probing mode, decrease target angle by 1
      motor0->angle_target = motor0->angle_current - 1;
    }
    motor1->angle_target = motor1->angle_min; //motor 1 probing position
    motor2->angle_target = motor2->angle_max; //motor 2 probing position
    motor3->angle_target = motor3->angle_min; //motor 3 probing position
    break;

  case 3:
    motor0->angle_target = round(mu);
    motor1->angle_target = round(theta);
    motor2->angle_target = round(phi);
    motor3->angle_target = motor3->angle_min;
    break;

  case 31:
    motor0->angle_target = round(mu);
    motor1->angle_target = round(theta);
    motor2->angle_target = round(phi);
    motor3->angle_target = motor3->angle_min;
    break;

  case 4:
    motor3->angle_target = motor3->angle_max; //set claw to close
    break;

  case 5:
  motor0->angle_period = 40;
    motor0->angle_target = mu_color0;
    motor1->angle_target = theta_color0;
    motor2->angle_target = phi_color0;
    motor3->angle_target = motor3->angle_max; //keep claw closed when moving
    break;

  case 6:
    motor0->angle_target = mu_color1;
    motor1->angle_target = theta_color1;
    motor2->angle_target = phi_color1;
    motor3->angle_target = motor3->angle_max; //keep claw closed when moving
    break;

  case 7:
    String Col = tcs_getcolor();

    if (Col != "y"){
      color_piece = Col;
    }

    Serial.println("|| Color: " + color_piece);
    break;

  }

  if(fsm_robotarm3.state == 8){
    motor0->angle_target = mu_yellow_box;
    motor1->angle_target = 90;
    motor2->angle_target = 180;
    motor3->angle_target = motor3->angle_max; //keep claw closed when moving
  }
  else if(fsm_robotarm3.state == 9){
    if (color_piece == "b"){
      motor0->angle_target = mu_blue_box;
      motor1->angle_target = theta_blue_box;
      motor2->angle_target = phi_blue_box;
      motor3->angle_target = motor3->angle_max; //keep claw closed when moving
    }
    else if (color_piece == "y"){
      motor0->angle_target = mu_yellow_box;
      motor1->angle_target = theta_yellow_box;
      motor2->angle_target = phi_yellow_box;
      motor3->angle_target = motor3->angle_max; //keep claw closed when moving
    }
    else if (color_piece == "r"){
      motor0->angle_target = mu_red_box;
      motor1->angle_target = theta_red_box;
      motor2->angle_target = phi_red_box;
      motor3->angle_target = motor3->angle_max; //keep claw closed when moving
    }
    else if (color_piece == "g"){
      motor0->angle_target = mu_green_box;
      motor1->angle_target = theta_green_box;
      motor2->angle_target = phi_green_box;
      motor3->angle_target = motor3->angle_max; //keep claw closed when moving
    }
  }
  else if(fsm_robotarm3.state == 10){
    motor3->angle_target = motor3->angle_min; //set claw to open
  }
  

  //SET OUTPUT

}

////////////////////////////////////////////////////////////////////MODE 1///////////////////////////////////////////////////////////////////////////

void fsm_robotarm_function1(){

  //UPDATE CURRENT TIS
  unsigned long curr_time = millis();
  fsm_robotarm1.tis = curr_time - fsm_robotarm1.tes;

  // READ INPUTS

  //CALCULATE NEW STATES

  int minangle = 60;
  int maxangle = 130;

  if (fsm_robotarm1.state == 0 && motor0->angle_current == motor0->angle_target && motor1->angle_current == motor1->angle_target && motor2->angle_current == motor2->angle_target && motor3->angle_current == motor3->angle_target){
    Serial.println("Robot restarted");
    robot_mode = '0';
    Serial.print("Robot mode: ");
    Serial.println(robot_mode);
  }
  else if (fsm_robotarm1.state == 1 && j_piece == -1){
    i_piece = 0; //change to arbitrary position (1,1) just because
    j_piece = 0;
    motor0->angle_period = 30;
    motor1->angle_period = 30;
    motor2->angle_period = 30;
    motor3->angle_period = 5;
    Serial.println("Input y Coordinate:");
  }
  else if (fsm_robotarm1.state == 1 && true_received != 'N'){
    i_piece = (int)true_received - (int)'0'; //ASCII TO INT
    true_received = 'N';
    Serial.println("y = " + String(i_piece));
    fsm_robotarm1.new_state = 11;
  }
  else if(fsm_robotarm1.new_state == 11){
    if (Serial.available() == 0){
      Serial.println("Input x Coordinate:");
      fsm_robotarm1.new_state = 2;
    }
  }
  else if (fsm_robotarm1.state == 2 && true_received != 'N'){
    j_piece = (int)true_received - (int)'0'; //ASCII TO INT
    true_received = 'N';
    Serial.println("x = " + String(j_piece));
    fsm_robotarm1.new_state = 21;
  }
  else if(fsm_robotarm1.new_state == 21){
    if (Serial.available() == 0){
      Serial.println("Going to top of piece");
      grid_polar(j_piece, i_piece);
      angles_xy(r , 4); 
      fsm_robotarm1.new_state = 3;
    }
  }
  else if (fsm_robotarm1.state == 3 && motor0->angle_current == motor0->angle_target && motor1->angle_current == motor1->angle_target && motor2->angle_current == motor2->angle_target && motor3->angle_current == motor3->angle_target){
    Serial.println("Going to piece");
    angles_xy(r , -2.5); 
    fsm_robotarm1.new_state = 31;
    Serial.println("Going to piece");    
  }
  else if (fsm_robotarm1.state == 31 && motor0->angle_current == motor0->angle_target && motor1->angle_current == motor1->angle_target && motor2->angle_current == motor2->angle_target && motor3->angle_current == motor3->angle_target){
    fsm_robotarm1.new_state = 4;
    Serial.println("Let's close claw");     
  }
  else if (fsm_robotarm1.state == 4 && motor3->angle_current == motor3->angle_target){
    fsm_robotarm1.new_state = 5;
    Serial.println("Go to top of color sensor");
  }

  else if (fsm_robotarm1.state == 5 && motor0->angle_current == motor0->angle_target && motor1->angle_current == motor1->angle_target && motor2->angle_current == motor2->angle_target){
    fsm_robotarm1.new_state = 6;
    Serial.println("Go to color sensor");
  }
  else if (fsm_robotarm1.state == 6 && motor0->angle_current == motor0->angle_target && motor1->angle_current == motor1->angle_target && motor2->angle_current == motor2->angle_target){
    fsm_robotarm1.new_state = 7;
    Serial.println("measure color");
    color_piece = "y";
  }
  else if (fsm_robotarm1.state == 7 && fsm_robotarm1.tis > 1000){
    fsm_robotarm1.new_state = 8;
    Serial.println("Go to top of boxes");
  }
  else if (fsm_robotarm1.state == 8 && motor0->angle_current == motor0->angle_target && motor1->angle_current == motor1->angle_target && motor2->angle_current == motor2->angle_target){
    fsm_robotarm1.new_state = 9;
    Serial.println("Go to box");
  }
  else if (fsm_robotarm1.state == 9 && motor0->angle_current == motor0->angle_target &&  motor1->angle_current == motor1->angle_target && motor2->angle_current == motor2->angle_target){
    fsm_robotarm1.new_state = 10;
    Serial.println("drop piece");
  }

  

  //UPDATE STATES
  set_state(fsm_robotarm1, fsm_robotarm1.new_state);
  //Serial.println("New state is " + String(fsm_robotarm1.state) + " angle is " + String(motor0->angle_current) + "target is " + String(motor0->angle_target));

  //CALCULATE OUTPUTS BASED ON NEW CURRENT STATE
  if (fsm_robotarm1.state == 0){
    motor0->angle_target = motor0->angle_min;
    motor1->angle_target = motor1->angle_min;
    motor2->angle_target = motor2->angle_max;
    motor3->angle_target = motor3->angle_min;
  }
  else if(fsm_robotarm1.state == 1){
    //Wait for coordinates
    if (Serial.available() > 0) {
      // read the incoming byte:
      char received = Serial.read();

      if (received == '0' || received == '1' || received == '2' || received == '3' || received == '4' || received == '5' || received == '6' || received == '7' || received == '8' || received == '9'){
        true_received  = received;
      }
   }   
  }
  else if(fsm_robotarm1.state == 2){
    //Wait for coordinates
    if (Serial.available() > 0) {
      // read the incoming byte:
      char received = Serial.read();

      if (received == '0' || received == '1' || received == '2' || received == '3' || received == '4' || received == '5' || received == '6' || received == '7' || received == '8' || received == '9'){
        true_received  = received;
      }
    }   
  }
  else if(fsm_robotarm1.state == 3){
    motor0->angle_target = round(mu_1);
    motor1->angle_target = round(theta);
    motor2->angle_target = round(phi);
    motor3->angle_target = motor3->angle_min;
  }
  else if(fsm_robotarm1.state == 31){
    motor0->angle_target = round(mu_1);
    motor1->angle_target = round(theta);
    motor2->angle_target = round(phi);
    motor3->angle_target = motor3->angle_min;
  }
  else if(fsm_robotarm1.state == 4){
    motor3->angle_target = motor3->angle_max; //set claw to close
  }
  else if(fsm_robotarm1.state == 5){
    motor0->angle_target = mu_color0;
    motor1->angle_target = theta_color0;
    motor2->angle_target = phi_color0;
    motor3->angle_target = motor3->angle_max; //keep claw closed when moving
  }
  else if(fsm_robotarm1.state == 6){
    motor0->angle_target = mu_color1;
    motor1->angle_target = theta_color1;
    motor2->angle_target = phi_color1;
    motor3->angle_target = motor3->angle_max; //keep claw closed when moving
  }
  else if(fsm_robotarm1.state == 7){
   String Col = tcs_getcolor();

    if (Col != "y"){
      color_piece = Col;
    }

    Serial.println("|| Color: " + color_piece); 
  }
  else if(fsm_robotarm1.state == 8){
    motor0->angle_target = mu_yellow_box;
    motor1->angle_target = 90;
    motor2->angle_target = 180;
    motor3->angle_target = motor3->angle_max; //keep claw closed when moving
  }
  else if(fsm_robotarm1.state == 9){
    if (color_piece == "b"){
      motor0->angle_target = mu_blue_box;
      motor1->angle_target = theta_blue_box;
      motor2->angle_target = phi_blue_box;
      motor3->angle_target = motor3->angle_max; //keep claw closed when moving
    }
    else if (color_piece == "y"){
      motor0->angle_target = mu_yellow_box;
      motor1->angle_target = theta_yellow_box;
      motor2->angle_target = phi_yellow_box;
      motor3->angle_target = motor3->angle_max; //keep claw closed when moving
    }
    else if (color_piece == "r"){
      motor0->angle_target = mu_red_box;
      motor1->angle_target = theta_red_box;
      motor2->angle_target = phi_red_box;
      motor3->angle_target = motor3->angle_max; //keep claw closed when moving
    }
    else if (color_piece == "g"){
      motor0->angle_target = mu_green_box;
      motor1->angle_target = theta_green_box;
      motor2->angle_target = phi_green_box;
      motor3->angle_target = motor3->angle_max; //keep claw closed when moving
    }
  }
  else if(fsm_robotarm1.state == 10){
    motor3->angle_target = motor3->angle_min; //set claw to open
  }

  //SET OUTPUT

}











////////////////////////////////////////////////////////////////////////////////////////////////FUNCTION 2///////////////////////////////////

void fsm_robotarm_function2(){

  //UPDATE CURRENT TIS
  unsigned long curr_time = millis();
  fsm_robotarm2.tis = curr_time - fsm_robotarm2.tes;

  // READ INPUTS

  //CALCULATE NEW STATES

  int minangle = 60;
  int maxangle = 130;

  if (fsm_robotarm2.state == 0 && motor0->angle_current == motor0->angle_target && motor1->angle_current == motor1->angle_target && motor2->angle_current == motor2->angle_target && motor3->angle_current == motor3->angle_target){
    Serial.println("Robot restarted");
    robot_mode = '0';
    Serial.print("Robot mode: ");
    Serial.println(robot_mode);
  }
  else if (fsm_robotarm2.state == 100 && number_pieces == -1){
    number_pieces = 0;
    motor0->angle_period = 30;
    motor1->angle_period = 30;
    motor2->angle_period = 30;
    motor3->angle_period = 5;
    Serial.println("Input number of pieces:");
  }
  else if (fsm_robotarm2.state == 100 && true_received != 'N'){
    number_pieces = (int)true_received - (int)'0'; //ASCII TO INT
    true_received = 'N';
    i_piece = 0; //change to arbitrary position (1,1) just because
    j_piece = 0;
    motor0->angle_period = 30;
    motor1->angle_period = 30;
    motor2->angle_period = 30;
    motor3->angle_period = 5;
    Serial.println("Number of pieces = " + String(number_pieces));
    fsm_robotarm2.new_state = 101;
  }
  else if (fsm_robotarm2.state == 101){
    if (Serial.available() == 0){
      Serial.println("Input Piece number" + String(1) +  " y Coordinate:");
      fsm_robotarm2.new_state = 1;
    }
  }
  else if (fsm_robotarm2.state == 1 && true_received != 'N'){
    piece_positions[given_pieces][1] = (int)true_received - (int)'0'; //ASCII TO INT
    true_received = 'N';
    Serial.println("y = " + String(piece_positions[given_pieces][1]));
    fsm_robotarm2.new_state = 11;
  }
  else if(fsm_robotarm2.new_state == 11){
    if (Serial.available() == 0){
      Serial.println("Input Piece number" + String(1) +  " x Coordinate:");
      fsm_robotarm2.new_state = 2;
    }
  }
  else if (fsm_robotarm2.state == 2 && true_received != 'N'){
    piece_positions[given_pieces][0] = (int)true_received - (int)'0'; //ASCII TO INT
    true_received = 'N';
    Serial.println("x = " + String(piece_positions[given_pieces][0]));
    for (int i = 1; i < number_pieces; i++){
      piece_positions[i][0] = piece_positions[0][0] + 3*(i%3);
      piece_positions[i][1] = piece_positions[0][1] + 1*(i/3);
    }
    fsm_robotarm2.new_state = 21;
  }
  else if(fsm_robotarm2.new_state == 21){
    if (Serial.available() == 0){
      //for (int i = 0; i < 9; i++) {
      //  for (int j = 0; j < 2; j++) {
      //    Serial.print(piece_positions[i][j]);
      //    Serial.print("\t");
      //  }
      //  Serial.println();
      //}
      //delay(30000);
      fsm_robotarm2.new_state = 3;
      Serial.println("Going to top of piece");
      x_this = piece_positions[sorted_pieces][0];
      y_this = piece_positions[sorted_pieces][1];
      grid_polar(x_this, y_this);
      angles_xy(r , 3); 
    }
  }
  else if (fsm_robotarm2.state == 3 && motor0->angle_current == motor0->angle_target && motor1->angle_current == motor1->angle_target && motor2->angle_current == motor2->angle_target && motor3->angle_current == motor3->angle_target){
    angles_xy(r , -2.5); 
    fsm_robotarm2.new_state = 31;
    Serial.println("Going to piece");    
  }
  else if (fsm_robotarm2.state == 31 && motor0->angle_current == motor0->angle_target && motor1->angle_current == motor1->angle_target && motor2->angle_current == motor2->angle_target && motor3->angle_current == motor3->angle_target){
    fsm_robotarm2.new_state = 4;
    Serial.println("Let's close claw");     
  }
  else if (fsm_robotarm2.state == 4 && motor0->angle_current == motor0->angle_target && motor1->angle_current == motor1->angle_target && motor2->angle_current == motor2->angle_target && motor3->angle_current == motor3->angle_target){
    fsm_robotarm2.new_state = 41;
    Serial.println("Go back up");  
    angles_xy(r , 4);    
  }
  else if (fsm_robotarm2.state == 41 && motor0->angle_current == motor0->angle_target && motor1->angle_current == motor1->angle_target && motor2->angle_current == motor2->angle_target && motor3->angle_current == motor3->angle_target){
    fsm_robotarm2.new_state = 5;
    Serial.println("Go to top of color sensor");
  }

  else if (fsm_robotarm2.state == 5 && motor0->angle_current == motor0->angle_target && motor1->angle_current == motor1->angle_target && motor2->angle_current == motor2->angle_target){
    fsm_robotarm2.new_state = 6;
    Serial.println("Go to color sensor");
  }
  else if (fsm_robotarm2.state == 6 && motor0->angle_current == motor0->angle_target && motor1->angle_current == motor1->angle_target && motor2->angle_current == motor2->angle_target){
    fsm_robotarm2.new_state = 7;
    Serial.println("measure color");
    color_piece = "y";
  }
  else if (fsm_robotarm2.state == 7 && fsm_robotarm2.tis > 1000){
    fsm_robotarm2.new_state = 8;
    Serial.println("Go to top of boxes");
  }
  else if (fsm_robotarm2.state == 8 && motor0->angle_current == motor0->angle_target && motor1->angle_current == motor1->angle_target && motor2->angle_current == motor2->angle_target){
    fsm_robotarm2.new_state = 9;
    Serial.println("Go to box");
  }
  else if (fsm_robotarm2.state == 9 && motor0->angle_current == motor0->angle_target &&  motor1->angle_current == motor1->angle_target && motor2->angle_current == motor2->angle_target){
    fsm_robotarm2.new_state = 10;
    Serial.println("drop piece");
  }
  else if (fsm_robotarm2.state == 10 && motor3->angle_current == motor3->angle_target && sorted_pieces < (number_pieces - 1)){
    sorted_pieces += 1;
    fsm_robotarm2.new_state = 21;
  }

  

  //UPDATE STATES
  set_state(fsm_robotarm2, fsm_robotarm2.new_state);
  //Serial.println("New state is " + String(fsm_robotarm2.state) + " angle is " + String(motor0->angle_current) + "target is " + String(motor0->angle_target));

  //CALCULATE OUTPUTS BASED ON NEW CURRENT STATE
  if (fsm_robotarm2.state == 0){
    motor0->angle_target = motor0->angle_min;
    motor1->angle_target = motor1->angle_min;
    motor2->angle_target = motor2->angle_max;
    motor3->angle_target = motor3->angle_min;
  }
  else if(fsm_robotarm2.state == 100){
    //Wait for coordinates
    if (Serial.available() > 0) {
      // read the incoming byte:
      char received = Serial.read();

      if (received == '0' || received == '1' || received == '2' || received == '3' || received == '4' || received == '5' || received == '6' || received == '7' || received == '8' || received == '9'){
        true_received  = received;
      }
   }  
  }
  else if(fsm_robotarm2.state == 1){
    //Wait for coordinates
    if (Serial.available() > 0) {
      // read the incoming byte:
      char received = Serial.read();

      if (received == '0' || received == '1' || received == '2' || received == '3' || received == '4' || received == '5' || received == '6' || received == '7' || received == '8' || received == '9'){
        true_received  = received;
      }
   }   
  }
  else if(fsm_robotarm2.state == 2){
    //Wait for coordinates
    if (Serial.available() > 0) {
      // read the incoming byte:
      char received = Serial.read();

      if (received == '0' || received == '1' || received == '2' || received == '3' || received == '4' || received == '5' || received == '6' || received == '7' || received == '8' || received == '9'){
        true_received  = received;
      }
    }   
  }
  else if(fsm_robotarm2.state == 3){
    motor0->angle_target = round(mu_1);
    motor1->angle_target = round(theta);
    motor2->angle_target = round(phi);
    motor3->angle_target = 10;
  }
  else if(fsm_robotarm2.state == 31){
    motor0->angle_target = round(mu_1);
    motor1->angle_target = round(theta);
    motor2->angle_target = round(phi);
    motor3->angle_target = 10;
  }
  else if(fsm_robotarm2.state == 4){
    motor3->angle_target = motor3->angle_max; //set claw to close
  }
  else if(fsm_robotarm2.state == 41){
    motor0->angle_target = round(mu_1);
    motor1->angle_target = round(theta);
    motor2->angle_target = round(phi);
    motor3->angle_target = motor3->angle_max;
  }
  else if(fsm_robotarm2.state == 5){
    motor0->angle_target = mu_color0;
    motor1->angle_target = theta_color0;
    motor2->angle_target = phi_color0;
    motor3->angle_target = motor3->angle_max; //keep claw closed when moving
  }
  else if(fsm_robotarm2.state == 6){
    motor0->angle_target = mu_color1;
    motor1->angle_target = theta_color1;
    motor2->angle_target = phi_color1;
    motor3->angle_target = motor3->angle_max; //keep claw closed when moving
  }
  else if(fsm_robotarm2.state == 7){
   String Col = tcs_getcolor();

    if (Col != "y"){
      color_piece = Col;
    }

    Serial.println("|| Color: " + color_piece); 
  }
  else if(fsm_robotarm2.state == 8){
    motor0->angle_target = mu_yellow_box;
    motor1->angle_target = 90;
    motor2->angle_target = 180;
    motor3->angle_target = motor3->angle_max; //keep claw closed when moving
  }
  else if(fsm_robotarm2.state == 9){
    if (color_piece == "b"){
      motor0->angle_target = mu_blue_box;
      motor1->angle_target = theta_blue_box;
      motor2->angle_target = phi_blue_box;
      motor3->angle_target = motor3->angle_max; //keep claw closed when moving
    }
    else if (color_piece == "y"){
      motor0->angle_target = mu_yellow_box;
      motor1->angle_target = theta_yellow_box;
      motor2->angle_target = phi_yellow_box;
      motor3->angle_target = motor3->angle_max; //keep claw closed when moving
    }
    else if (color_piece == "r"){
      motor0->angle_target = mu_red_box;
      motor1->angle_target = theta_red_box;
      motor2->angle_target = phi_red_box;
      motor3->angle_target = motor3->angle_max; //keep claw closed when moving
    }
    else if (color_piece == "g"){
      motor0->angle_target = mu_green_box;
      motor1->angle_target = theta_green_box;
      motor2->angle_target = phi_green_box;
      motor3->angle_target = motor3->angle_max; //keep claw closed when moving
    }
  }
  else if(fsm_robotarm2.state == 10){
    motor3->angle_target = motor3->angle_min; //set claw to open
  }

  //SET OUTPUT

}


#endif