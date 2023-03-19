// This is the final version of integrated code and is what was used in the final competition
// Performs main navigation logic of following lines and detecting junctions 
// Operate grabber using servo motor to collect and deposit blocks 
// Identify colour of block using colour sensor 
// Optimisation of block collection - if deposit a blue block then go other way around the table 

// MOTOR SHIELD RELATED CODE 
// include header files for motor shield and servo
#include <Adafruit_MotorShield.h>
#include <Servo.h> 
#include <Wire.h> 

// create motor shield class
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

// create motor objects 
Adafruit_DCMotor *RMotor = AFMS.getMotor(1);
Adafruit_DCMotor *LMotor = AFMS.getMotor(2);

// create servo objects
Servo left_grabber; // one servo operate the left grabber and the other is moved using gear mecahnism 


// DEFINE VARIABLES AND CONSTANTS 

// define value of digital pin for servo 
const int servo1Pin = 10; 

// define value of digital pin for amber flashing light 
const int ledPin = 0;

// define values of digital pins for line sensors 
const int lineFollowOL = 1; // outside left line sensor
const int lineFollowOR = 2; // outisde right line sensor
const int lineFollowIL = 3; // inside left line sensor 
const int lineFollowIR = 4; // inside right line sensor

// define value of analog pin for colour sensor 
const int colourPin = 0;

// define value of digital pin for button
const int buttonPin = 8;
// other button related variables 
int buttonState;
bool start_flag=false;

// define digital pins for colour detection leds
const int redledPin = 11;
const int greenledPin = 12;

// stage variable used to indicate at what stage the robot is in code 
// start(0), traversing table(1), block collection after depositing blue (2), return to home after depositing last block (3), end(4) 
int stage=0; 

// max number of table loops robot can complete in 5 mins when it has collected and deposited a block
const float max_loop_count = 5; 
// number of table loops robot will have completed if it decides to do the next loop (the first loop is always counted as 1)
float loop_count = 1; 

// declare line sensor variables to store latest values 
int OL, OR, IL, IR;

// general speed used whenever robot it moving 
const int base_speed = 100; // this is max speed i.e. 100%

// global motor variables - avoid updating motor speed if no change required 
int prev_r_speed=0;
int prev_l_speed=0;

// turn function variables 
enum turn_direction {RIGHT=0, LEFT=1};
const int right_angle_time = 925;  // perfect 90 degrees turn time 
const int larger_spin_time = 1050; // slighlty larger than 90 degrees turn time 

// start sequence variable
int start_junction=0; // this can be 0 or 1 as there are two junctions involved at the start

// junction detection variable - only need to specify between each right junction 
int right_junction = 0; // this will either be value 0,1,2 depending on specific junction
// 2 is the location for blue block, 0 is the start box, 1 is the location for brown block

// grabber operation variables 
const int open_position = 0; // used when not carrying a block 
const int close_position = 82; // used when carrying a block

// block collection variable - related to when grabber is operated 
bool block_carry = false; // used to check if robot is carrying a block or not 

// block colour variables 
enum block_colour {BLUE=0, BROWN=1};
block_colour colour;


// DECLARE FUNCTIONS 

// read value from all line sensors 
void readLineSensors() {
  OL=digitalRead(lineFollowOL);
  OR=digitalRead(lineFollowOR);
  IL=digitalRead(lineFollowIL);
  IR=digitalRead(lineFollowIR);
}

// takes a value from -100 to 100 and sets right motor speed
void rmotor(int speed) {
  if (speed > 100 || speed < -100){
    Serial.println("ERROR: Speed must be between -100 and 100");
    return 0;
  }
  if (speed == prev_r_speed) {
    return 0;
  }
  else {
    prev_r_speed = speed;
    digitalWrite(ledPin, HIGH); // turn on amber flashing light when moving 
    if (speed > 0) {
      RMotor->setSpeed(round(speed*254/100));
      RMotor->run(FORWARD);
    }
    else if (speed < 0) {
      RMotor->setSpeed(round(-1*speed*254/100));
      RMotor->run(BACKWARD);
    }
    else {
      RMotor->run(RELEASE);
    }
  }
}

// takes a value from -100 to 100 and sets left motor speed
void lmotor(int speed) {
  if (speed > 100 || speed < -100){
    Serial.println("ERROR: Speed must be between -100 and 100");
    return 0;
  }
  if (speed == prev_l_speed) {
    return 0;
  }
  else {
    prev_l_speed = speed;
    digitalWrite(ledPin, HIGH); // turn on amber flashing light when moving 
    if (speed > 0) {
      LMotor->setSpeed(round(speed*254/100));
      LMotor->run(FORWARD);
    }
    else if (speed < 0) {
      LMotor->setSpeed(round(-1*speed*254/100));
      LMotor->run(BACKWARD);
    }
    else {
      LMotor->run(RELEASE);
    }
  }
}

// stops the robot and turns off the amber flashing light 
void stop(){
  rmotor(0);
  lmotor(0);
  digitalWrite(ledPin, LOW);
}

// rotates the robot to the right or left for a given amount of time (either 90 degrees or slighly more)
void turn(turn_direction direction, int spin_time){
  // turn right 
  if (direction == RIGHT){
    Serial.println("Right");
    rmotor(-base_speed);
    lmotor(base_speed);    
  }
  // turn left 
  else if (direction == LEFT){
    Serial.println("Left");
    rmotor(base_speed);
    lmotor(-base_speed);
  }
  delay(spin_time);
  stop(); // always need to stop after turn to avoid keep spining 
}

// uses inside line sensor values to follow line
void lineFollow(){
  if (IL==IR) {
    // Go straight 
    rmotor(base_speed); //100
    lmotor(base_speed); //100
  }
  else if (IL==1 && IR==0) {
    // Go right
    rmotor(base_speed); //100
    lmotor(0.1*base_speed); //10 - this is effectively 0
    }
  else if (IL==0 && IR==1){
    // Go left 
    rmotor(0.1*base_speed); //10 - this is effectively 0
    lmotor(base_speed); //100
  }
}

// move robot away from junction by calling lineFollower() for set amount of time (ignores outside sensors)
void getOutJunction(){
  for(int i=0; i<10; i++) {
      readLineSensors();
      lineFollow();
      delay(30);
  }
}

// use colour sensor to determine if block is blue or brown 
void detectColour(){
  // declare local variable
  int colour_value; 
  // read the colourPin 200 times to ensure it captures the colour 
  for (int i=0; i<200; i++){
    colour_value = analogRead(colourPin);
    delay(10);
    if (colour_value <= 570){ // about 2.8 volts
      colour = BLUE;
      return;
    }
  }
  // if not blue then must be brown 
  colour = BROWN;
}

// turn on green led for 5 seconds if blue is detected or red led for 5 seconds if brown is detected 
void colourIndication() {
  if (colour == BLUE){
      digitalWrite(greenledPin, HIGH);
    }
    else {
      digitalWrite(redledPin, HIGH);
    }
    delay(5000);
    digitalWrite(greenledPin, LOW);
    digitalWrite(redledPin, LOW);
}

// park sequence performed at right junction labelled 0 (when the stage is 3)
void park() {
  // turn into junction 
  // handle case when approaching from other direction as last block delivered was brown 
  if (colour==BROWN){
    turn(LEFT, right_angle_time);
  }
  else{
    turn(RIGHT, right_angle_time);
  }
  // go straight untill all of robot is inside line
  rmotor(base_speed);
  lmotor(base_speed);
  delay(1200); // value found through testing   
  // stop robot and update led 
  stop();
  // update stage of operation
  stage = 4;
}

// deposit block sequence performed at right junctions labelled 1 and 2
void depositBlock(){
  // turn right 90 degrees into junction 
  turn(RIGHT, right_angle_time);
  // go straight for small adjustment 
  rmotor(base_speed); 
  lmotor(base_speed);
  delay(300); // found through testing 
  // stop robot and control led 
  stop();
  // small pause either side of grabbing block 
  delay(100);  
  // open grabber to deposit block 
  left_grabber.write(open_position);
  delay(100);
  // go backwards by same small ajustment  
  rmotor(-base_speed);
  lmotor(-base_speed);
  delay(300); 
  // handle case of return to home after last brown block or going to collect another block from blue deposit block 
  if ((stage==3 && colour==BROWN)|| (stage==2)){
    turn(RIGHT, larger_spin_time); 
  }
  else{
    turn(LEFT, larger_spin_time); 
  }    
}

// collect block sequence performed at left junctions (may be seen as a right junction if traversing other way)
void collectBlock(){
  // turn left 90 degrees into junction
  if (stage==2){
    turn(RIGHT, right_angle_time); // handle case robot is going other way around table 
    stage=1;
  }
  else {
    turn(LEFT, right_angle_time);
  }
  // small pause either side of grabbing block - make it easier to see what is going on
  delay(100); 
  // close grabber to collect block 
  left_grabber.write(close_position);
  delay(100); // MAY NEED TO INCREASE THIS PAUSE
  // use colour sensor
  detectColour();  
  // control green and red leds to indicate what colour was detected 
  colourIndication();
  // turn left 90 degrees out of junction 
  turn(RIGHT, larger_spin_time); // right hand turns made greater than 90 to ensure return to line after collecting block
}

// perform start sequence to get of the box and onto main line 
void start() {
  // line of the box is the first junction seen - ignore this 
  if (start_junction==0){
    start_junction = 1;
    getOutJunction();   
  }
  // intersecting lines is the second junction seen - turn here 
  else {
    // turn 90 degrees right 
    turn(RIGHT, right_angle_time); 
    // move away from this junction
    getOutJunction();
    // update stage variable - no longer in start sequence 
    stage = 1;
  }
}

// determine suitable action based on specific right junction, how much time is left, and whether a certain coloured block is being carried 
void rightJunction(int right_junction){
  // handle case when last block deposited was blue so is traversing table the other way (right junction now indicates a block collection place)
  if (stage==2){
    collectBlock();
    block_carry = true;  
  }

  // when at correct junction to deposit a block 
  else if (block_carry == true) {
    if ((colour==BLUE && right_junction==2) || (colour==BROWN && right_junction==1)){ 
      
      if (colour==BLUE){
        loop_count += 0.8; // the next loop will take slightly less time (although have to asusme worse case of collecting brown block)
      }
      else {
        loop_count += 1; // 1 full loop of the table is defined from brown box to brown box 
      }

      if (loop_count > max_loop_count) {
        stage = 3; // indicates that robot needs to be making its way home after depositing this current block  
      }
      else { // indicates robot has time to collect more blocks 
        if (colour==BLUE){
          stage=2; // go other way around table for next block to save time 
        }
      }
      // need to deposit block 
      depositBlock();
      // update block_carry variable
      block_carry = false;      
    }
  }
  // handle the case of parking when last block delivered was blue 
  else if (right_junction==0 && stage==3) {
    park();
  }
}

// determine suitable action at left junction based on if a block has been detected recently and if not carrying a block 
void leftJunction() {
  // handle the case of parking when last block delivered was brown (start box now seen as a left junction as other way around) 
  if (stage==3){
    park();
  }
  else{
    // only try collect a block if not already carrying one 
    if (block_carry==false) {
      collectBlock();   
      // update block_carry variable 
      block_carry = true;   
    }
  }
}

// uses outside line sensors to perform overall navigation logic - calls different functions depending on each value combo
void navigation() {
  // no junction detected - follow line using inside sensors
  if (OL==0 && OR==0){
    lineFollow();
  }
  // any type of junction detected - handle special case of start stage (stage 0)
  else if (stage==0){
    start();
  }
  // handle traversing table stage (stage 1)  
  // left junction detected (box collection)
  else if (OL==1 && OR==0){
    // bring robot to a stop - shows robot has seen junction
    stop();
    // identify action to take at junction i.e. can robot collect a block?
    leftJunction();
    // move away from this junction 
    getOutJunction();
  }
  // right junction detected (box deposit or start junction) - these are labelled 2,0,1 from left to right 
  else if (OL==0 && OR==1){
    // bring robot to a stop - shows robot has seen junction
    stop();
    delay(500); // add a pause at right junction to check robot has seen it and ensure counting remains correct 
    // indentify junction by updating right_junction variable - only do this if traversing the table the correct way 
    if (stage!=2){
      right_junction = (right_junction + 1) % 3;
    }
    else{
      right_junction = right_junction - 1;
    }
    // identify action to take based on specifc junction and colour of block if being carried ie. can a block be deposited or does robot need to park?
    rightJunction(right_junction); 
    // move away from this junction 
    getOutJunction();
  }
  // this case should not occur unless performing a set sequence in which case robot's movement is handled else where 
  else if (OL==1 && OR==1){
    lineFollow();
  }
}


// MAIN PROGRAM

// code that is executed once when program is first run  
void setup() {
  // set serial connection - used for debugging purposes 
  Serial.begin(9600);

  // initial motor shield 
  AFMS.begin();

  // set line sensor pins as inputs 
  pinMode(lineFollowOL, INPUT); 
  pinMode(lineFollowOR, INPUT);
  pinMode(lineFollowIL, INPUT);
  pinMode(lineFollowIR, INPUT);

  // set colour sensor pins as inputs 
  pinMode(colourPin, INPUT);

  // set led pin
  pinMode(ledPin, OUTPUT);

  // set servo motor pins
  left_grabber.attach(servo1Pin); 
  left_grabber.write(open_position); // should not move grabber as start at 0 degrees

  // set button pin
  pinMode(buttonPin, INPUT);

  // set colour detection red and green leds
  pinMode(redledPin, OUTPUT);
  pinMode(greenledPin, OUTPUT);

}

// code that is continually executed 
void loop() {

  // read the state of the pushbutton value:
  buttonState = digitalRead(buttonPin);

  // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
  if (buttonState == HIGH) {
    if (start_flag==false){
      // set start flag to true
      start_flag = true;
    }
    else{
      stop();
      // reset variables so can start runing again knowing how many loops it has already done
      start_flag = false;
      stage = 0;
      //loop_count = 0; Dont reset this as if have to restart want it to know how many loops it can still do in the time 
      prev_r_speed=0;
      prev_l_speed=0;
      start_junction=0;
      right_junction = 0;
      block_carry = false;
      left_grabber.write(open_position);
    }
  }

  // continue to run main program if flag is true and end stage has not been reach 
  if (start_flag==true && stage!=4){
    readLineSensors(); // get the four latest line sensor values 
    navigation(); // coordinate robot's action
  else{
    stop();
  }
}
