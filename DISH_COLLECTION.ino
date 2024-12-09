#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include <addons/TokenHelper.h>
#include <addons/RTDBHelper.h>


// Firebase Credentials
#define API_KEY "AIzaSyAEQVUGAVSp-6OPL59vDmZn9p86AKl3WVg"
#define DATABASE_URL "https://test-63f82-default-rtdb.asia-southeast1.firebasedatabase.app/"
#define USER_EMAIL "eliaskufakunesu07@gmail.com"
#define USER_PASSWORD "dbpassword"

// WiFi Credentials - Can be modified for user input
#define WIFI_SSID "Ok"            // WiFi SSID
#define WIFI_PASSWORD "123123123" // WiFi Password

// Motor Driver Pins
#define ENA 14 // Motor A PWM
#define IN1 13 // Motor A direction control pin 1
#define IN2 12 // Motor A direction control pin 2
#define ENB 25 // Motor B PWM
#define IN3 27 // Motor B direction control pin 1
#define IN4 26 // Motor B direction control pin 2

// Infrared (IR) Sensors
#define RightBackSensor 22 // Right back IR sensor
#define LeftBackSensor 21  // Left back IR sensor

//Make line following module 
#define MakerPostion 33 //Analog Values 

// Ultrasonic Sensor Pins
const int Trig = 17; // Ultrasonic trigger pin
const int Echo = 16; // Ultrasonic echo pin

// Weight Sensors
const int BeamSensorLevel1 = 34; // First beam sensor
const int BeamSensorLevel2 = 35; // Second beam sensor

// Kitchen Dismiss Button
#define KitchenBtn 5 // Button to dismiss in the kitchen

//BUZZER 
#define Buzzer 0

// TFT LCD Display
TFT_eSPI tft = TFT_eSPI();
int screenWidth = 128;  // Screen width in pixels
int screenHeight = 160; // Screen height in pixels

// Variables
int gridCount = 0; // Counter for grid position

// Table structure for database
struct Table {
  const char *name; // Table name
  int GRID;         // Grid position of the table
};

Table *TableBeingServed;
int status = 0;

// Tables configuration for the database
Table tables[] = {
  {"TABLE1", 1}, {"TABLE2", 2}, {"TABLE3", 3},
  {"TABLE4", 4}, {"TABLE5", 5}, {"TABLE6", 6},
  {"TABLE7", 7}, {"TABLE8", 8}, {"TABLE9", 9},
  {"TABLE10", 10}, {"TABLE11", 11}, {"TABLE12", 12},
  {"TABLE13", 13}, {"TABLE14", 14}
};

// Firebase data objects
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

// Timing variables
unsigned long previousMillis = 0; // Stores the last LED update time
const long interval = 200;        // LED update interval in milliseconds

//State Variables 

bool goToTable = false;
bool atTable  = false;
bool goToKitchen = false;
bool atKitchen  = false;
bool isServing = false;
int StatusKitchen = 0;    // Kitchen status flag


//OTHER FLAGS 
bool turnL1 = false;
bool turnL2 = false;
int turn = 0;
bool  makeLeftT = false; 
bool  makeRight =  false; 
bool makeUturn = false;

//PID 
const double KP = 0.105;///0.045;  // 0.0975; // Proportional constant; previously tested values: 0.043, 0.08, 0.04353 (tuned for system response)
const double KD = 2.605;
double lastError = 0;
const int GOAL = 2000;
const unsigned char MAX_SPEED = 180; //opt 165 

//LCD 
int eyeRadius = 20; 
int pupilRadius = 8;
int eyeXOffset = 40; 
int eyeYOffset = 40; 
int eyeSpacing = 16;

void setup() {
  //Serial.begin(115200);
  initSensors();
  MotorDriver();
  drawWelcomePage();
  delay(1000); 
   drawInstructionScreen();
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  //Serial.print("Connecting to Wi-Fi");
    while (WiFi.status() != WL_CONNECTED) {
      //INSTRUCTIONS 
       drawInstructionScreen();
       // Serial.print(".");
        delay(250);
    }
  database_setup();  //DATA  BASE SETUP 
}

//FUNCTION SETUP 
// Firebase database setup
void database_setup() {
  config.api_key = API_KEY;                  // Firebase API Key
  auth.user.email = USER_EMAIL;             // User email for authentication
  auth.user.password = USER_PASSWORD;       // User password for authentication
  config.database_url = DATABASE_URL;       // Firebase Realtime Database URL
  config.token_status_callback = tokenStatusCallback; // Token callback
  Firebase.reconnectNetwork(true);          // Reconnect to Firebase if network drops
  Firebase.begin(&config, &auth);           // Initialize Firebase connection
}
//Motor Driver Setup 
void MotorDriver() {
  // Initialize Left Motor pins
  pinMode(ENA, OUTPUT); // Set PWM pin for left motor as output
  pinMode(IN1, OUTPUT); // Set direction control pin 1 for left motor as output
  pinMode(IN2, OUTPUT); // Set direction control pin 2 for left motor as output
  ledcAttach(ENA, 1000, 8); // Attach PWM signal to left motor with 1kHz frequency and 8-bit resolution

  // Initialize Right Motor pins
  pinMode(ENB, OUTPUT); // Set PWM pin for right motor as output
  ledcAttach(ENB, 1000, 8); // Attach PWM signal to right motor with 1kHz frequency and 8-bit resolution
  pinMode(IN3, OUTPUT); // Set direction control pin 1 for right motor as output
  pinMode(IN4, OUTPUT); // Set direction control pin 2 for right motor as output
}

//Sensor Setup 
void initSensors() {
  // Initialize Ultrasonic Sensor pins
  pinMode(Trig, OUTPUT); // Set ultrasonic trigger pin as output
  pinMode(Echo, INPUT);  // Set ultrasonic echo pin as input

  // Initialize Sensor Maker 
  pinMode(MakerPostion ,INPUT);

  // Initialize Back IR Sensors
  pinMode(RightBackSensor, INPUT); // Set right back IR sensor as input
  pinMode(LeftBackSensor, INPUT);  // Set left back IR sensor as input

  // Initialize Buzzer
  pinMode(Buzzer, OUTPUT); // Set buzzer pin as output

  //initialize Beam breaking sensors 
  pinMode(BeamSensorLevel1, INPUT);
  pinMode(BeamSensorLevel2, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  drawWelcomePage();
  if ( isServing == false){

    if (!Firebase.ready()) {
       //
       // Serial.println("Firebase is not ready yet. Retrying...");
        return; // Exit the loop if Firebase is not ready
    }
    unsigned long currentMillis = millis();
    // Check if the interval has passed
    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis; // Update the last update time
        getTableToServe();
    }

  }
  if ( goToTable == true){
      //Follow PID To table
      goToTableNav();
  }
  if ( atTable == true){
      //Wait for Dismissal 
      targetTableReached();
  }
  if( goToKitchen == true ){
     //PID to kitchen 
     navigateBackToKitchen();
  }
  if ( atKitchen == true ){
    //wait for kitchen dismissal then reset variables and repeat 
    arrivedAtKitchen();
  }

}

void getTableToServe() {
  // Calculate the number of tables in the array
  int tableSize = sizeof(tables) / sizeof(tables[0]);

  // Iterate through all tables to check for a service request
  for (int i = 0; i < tableSize; i++) {
    Table *currentTable = &tables[i]; // Pointer to the current table in the array

    // Retrieve the status of the current table from the Firebase database
    if (Firebase.RTDB.getInt(&fbdo, F(currentTable->name), &status)) {
      // Check if a service request has been made (status == 1)
      if (status == 1) {
        // Assign the table to the serving queue
        TableBeingServed = currentTable;

        // Set the serving flag and navigate to the requested table
        isServing = true;
        navigateToTable();
         // Exit the loop once a table is found
      }
      else{
        // ERROR;
      }
    } 
    // You can handle errors or unexpected behavior here if needed
  }
}

// FUNCTION TO NAVIGATE TO THE TARGET TABLE
void navigateToTable() {
  // Retrieve the target grid of the table being served
  int target = TableBeingServed->GRID;

  //Decide which path to take  
  if ( target  <= 5){
    //FIRST ROW  L 1
    turnL1 = true ;
    
   
  } else if ( target  <= 10 ){
    //SECOND ROW MIDDLE  ROW  L 2 
    gridCount += 5;
    turnL2 = true; 

  }
  else if ( target  >= 10  && target <= 15 ){
    //Third Row  No Turn 
    gridCount += 10;
    turnL1 = false ;
    turnL2 = false ;  
    
  }


  goToTable = true;

}

//PID TO Table
void goToTableNav(){
  int TableNumber = TableBeingServed->GRID;
  unsigned int position = analogRead(MakerPostion);
  int LeftIR = digitalRead(LeftBackSensor);
  int RightIR = digitalRead(RightBackSensor);
  obstacleDetection(); //obstacle checking 
 
    if (LeftIR  == 1 && RightIR == 1){
        MotorL(0);MotorR(0);
        while( LeftIR  == 1 && RightIR == 1){
             LeftIR = digitalRead(LeftBackSensor);
             RightIR = digitalRead(RightBackSensor);
             MotorL(150);MotorR(150);
             delay(100);
        }

        gridCount++;

      }

      if( gridCount == TableNumber){
        atTable = true;
        goToTable = false;
      }

      //WHEN TURN = 1 FISRT ROW TABLES 
      if (LeftIR  == 1 && RightIR == 0  && turnL1 == true ){
       //STEPS TO MAKE 90 DEGREE TURN  Stop on Line , Rotate left until oveshoot , then rotate until on the line 
       //Serial.print("RightIR : ");Serial.println(RightIR);
       MotorL(0);MotorR(0);
        while( position != 0  ){
              // Rotate left 
              position = analogRead(MakerPostion);
              reverseL();
              position = analogRead(MakerPostion);
              
        }
        while ( !(position >= 1800 && position <= 2100  )){
             position = analogRead(MakerPostion);
             reverseL();
             position = analogRead(MakerPostion);
             
        }
        turnL1 = false;
            MotorL(0);
            MotorR(0);
            delay(20);
      }

      //WHEN TURN = 2 SECOND  ROW TABLES 
      if (LeftIR  == 1 && RightIR == 0 && turn  ==  2 && turnL2 == true  ){
       //STEPS TO MAKE 90 DEGREE TURN  Stop on Line , Rotate left until oveshoot , then rotate until on the line 
       MotorL(0);MotorR(0);
        while( position != 0  ){
              // Rotate left 
              position = analogRead(MakerPostion);
              reverseL();
              position = analogRead(MakerPostion);
              
        }
        while ( !(position >= 1800 && position <= 2100  )){
             position = analogRead(MakerPostion);
             reverseL();
             position = analogRead(MakerPostion);
             
        }
        turnL1 = false;
            MotorL(0);
            MotorR(0);
            delay(20);

      }else if ( turnL2 == true &&  LeftIR  == 1 && RightIR == 0  ){
            while( LeftIR  == 1 && RightIR == 0){
             MotorL(0);MotorR(0); //stop 
             LeftIR = digitalRead(LeftBackSensor);
             RightIR = digitalRead(LeftBackSensor);
             MotorL(150);MotorR(150);
             delay(100);
        }

       turn ++;
      }


  position = analogRead(MakerPostion);
  // Calculate the error between the target position (GOAL) and the current position
  int error = GOAL - position;

  // Calculate the PID adjustment
  lastError = error;  // Store the current error for the next calculation
  int adjustment = KP * error + KD * (error - lastError); // PID formula using KP and KD

  // Adjust motor speeds to follow the line based on PID output
  MotorL(MAX_SPEED - adjustment); // Adjust left motor speed
  MotorR(MAX_SPEED + adjustment); // Adjust right motor speed

}
//Target Reached 
void targetTableReached() {
  // Retrieve the current service status of the table from Firebase
  Firebase.RTDB.getInt(&fbdo, F(TableBeingServed->name), &status);

  // While the table still has a service request (status == 1)
  while (status == 1) {
    playArrivedTone(); // Play a sound indicating arrival at the table
    Firebase.RTDB.getInt(&fbdo, F(TableBeingServed->name), &status); 
     tft.fillScreen(TFT_RED);
     delay( 100);
    tft.fillScreen(TFT_GREEN); // Continuously check if the table is still requesting service
     delay(100);
  }

  // Set status to 0 after serving the table
  status = 0;
  getWeight(); //get the weigth Right 
  // Navigate back to the kitchen after serving the table
  //Set Flag to go to kitchen 
  //drawWelcomePage();
   atTable = false;
   goToKitchen = true;
}
//FUNCTION TO NAVIGATE TO THE TARGET TABLE
void navigateBackToKitchen(){
  
  if ( gridCount <= 5){
      makeUturn = true;

   }
   else if (  gridCount  > 5 &&  gridCount <=9  ){
    makeLeftT = true; 
    makeRight = true; 

   }
   else if (  gridCount  >= 10 &&  gridCount  <= 14){
    //
    makeRight = true;
   }

  unsigned int position_p = analogRead(MakerPostion);
  int LeftIR = digitalRead(LeftBackSensor);
  int RightIR = digitalRead(RightBackSensor);

//Make A uturn
  if ( makeUturn == true){
    //Make A Uturn back to kitchen 
        while (position_p != 0 ){
          position_p = analogRead(MakerPostion);
          reverseL();
          position_p = analogRead(MakerPostion);
        }

        while ( !(position_p >= 1800 && position_p <= 2100  )){
            position_p = analogRead(MakerPostion);
            delay(10);
            reverseL();
            position_p = analogRead(MakerPostion);
        }
          MotorL(0);
          MotorR(0);
          delay(200);
          return;
        }
  //AT The T junction 
if ( position_p == 4095 || position_p > 3950){
      MotorL(0);  // Stop left motor
      MotorR(0);  // Stop right motor
      delay(100); // Brief pause to ensure the robot has stopped

      // Move forward slightly to ensure the robot is fully in the junction
      MotorL(170);
      MotorR(170);
      delay(100); // Adjust the delay as needed based on your robot's speed and junction size

      MotorL(0);  // Stop left motor
      MotorR(0);  // Stop right motor
position_p = analogRead(MakerPostion);
if(position_p  == 0   && makeRight == true  && makeLeftT == false){
  //no edge  do a 90 Degree turn right 
  while ( !(position_p >= 1800 && position_p <= 2100  )){
      position_p = analogRead(MakerPostion);
      reverse();
      position_p = analogRead(MakerPostion);
  }
    MotorL(0);
    MotorR(0);
    delay(20);

    makeRight = false;
}
if(position_p  == 0   && makeRight == false  && makeLeftT == true ){

  //no edge  do a 90 Degree turn Left
  while ( !(position_p >= 1800 && position_p<= 2100  )){
      position_p = analogRead(MakerPostion);
      reverse();
      position_p = analogRead(MakerPostion);
  }
    MotorL(0);
    MotorR(0);
    delay(20);

  makeLeftT = false;

}
}
//When it Reaches The Kitchen 
if (position_p >= 3950) {
  MotorL(0);  // Stop left motor
  MotorR(0);  // Stop right motor
  delay(50); // Brief pause to ensure the robot has stopped

  while (!(LeftIR == 1 && RightIR == 1)) {
    LeftIR = digitalRead(LeftBackSensor);
    RightIR = digitalRead(RightBackSensor);
    MotorL(175);
    MotorR(175);
  }
  position_p = analogRead(MakerPostion);

  // **NOTE:** You likely want to use position_p here as well for consistency
  if (LeftIR == 1 && RightIR == 1 && position_p >= 3950) { 
    while (position_p != 0) { 
      position_p = analogRead(MakerPostion);
      reverse();
      position_p = analogRead(MakerPostion);
    }
    while (!(position_p >= 1800 && position_p <= 2100)) { 
      position_p = analogRead(MakerPostion);
      delay(10);
      reverse();
      position_p = analogRead(MakerPostion);
    }
    MotorL(0);
    MotorR(0);
    delay(500);
    return;
    
    // Arrived at kitchen  
    atKitchen = true; 
    goToKitchen = false;

  }
}

 position_p = analogRead(MakerPostion);

  int error_p = GOAL - position_p;
  lastError = error_p;  // Store error for next increment
  int adjustment_p = KP*error_p + KD*(error_p - lastError);

   // Adjust motors To follow the Line 
  MotorL(MAX_SPEED - adjustment_p);
  MotorR(MAX_SPEED + adjustment_p);
    
}
//Function at Kitchen 
void arrivedAtKitchen(){

  StatusKitchen = 1;
  while (StatusKitchen == 1) {
      playArrivedTone(); //BUZZER 
    // wait for kitchen crew
     int staffbtn = digitalRead(KitchenBtn);
     delay(10);
    if( staffbtn == LOW){
        StatusKitchen = 0;
        isServing  = false; // reset is serving to false 
     }

  }
  gridCount = 0;
  turnL1 = false;
  turn = 0;
  turnL2 = false;
  status = 0;
  isServing = false;
  atKitchen =  false; 
  delay(100);

}

//FUNCTIONS FOR MOTOR  
void reverse() {
  //Motor L
    ledcWrite(ENA,  160);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);

  //Motor R
    ledcWrite(ENB, 170);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);

}
void reverseL(){
  //Motor L
    ledcWrite(ENA,  175);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);

  //Motor R
    ledcWrite(ENB, 175);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);

}

// Function to control the left motor
void MotorL(int LeftSpeed) {
    // Ensure the speed is within the allowable range (0 to 230)
    LeftSpeed = constrain(LeftSpeed, 0, 210);

    // Set the PWM signal for the left motor speed
    ledcWrite(ENA, LeftSpeed);

    // Set the direction for the left motor (forward)
    digitalWrite(IN1, HIGH);  // IN1 pin HIGH to move forward
    digitalWrite(IN2, LOW);   // IN2 pin LOW to move forward
}

// Function to control the right motor
void MotorR(int RightSpeed) {
    // Ensure the speed is within the allowable range (0 to 230)
    RightSpeed = constrain(RightSpeed, 0, 210);

    // Set the PWM signal for the right motor speed
    ledcWrite(ENB, RightSpeed);

    // Set the direction for the right motor (forward)
    digitalWrite(IN3, LOW);   // IN3 pin LOW to move forward
    digitalWrite(IN4, HIGH);  // IN4 pin HIGH to move forward
}

// OBSTACLE DETECTION FUNCTION
void obstacleDetection() {
  // Measure the distance to an obstacle
  long distance = getDistance(); // Get the current distance from the ultrasonic sensor
    delay(10); // Short delay to stabilize sensor readings
  // Check if an obstacle is detected within a specific range
  while (distance >= 5 && distance <= 20) { 
    MotorR(0);MotorL(0);      // Stop the motors immediately to avoid collision
    obstacleBeep();        // Activate the buzzer to signal an obstacle
    tft.fillScreen(TFT_RED);   // Display "angry eyes" on the screen (visual alert)
    distance = getDistance(); // Re-check the distance to the obstacle
  }
}

void obstacleBeep() {
  tone(Buzzer, 900, 200); // Generate a tone at 900 Hz for 200 milliseconds using the buzzer
  delay(200);             // Wait for 200 milliseconds before stopping the tone
  noTone(Buzzer);         // Stop the tone on the buzzer
  delay(200);             // Add a short delay before the function ends
}
// FUNCTION TO MEASURE DISTANCE USING ULTRASONIC SENSOR
long getDistance() {
  // Trigger the ultrasonic sensor to send a pulse
  digitalWrite(Trig, LOW);          // Ensure the trigger pin is LOW
  delayMicroseconds(2);             // Wait for 2 microseconds
  digitalWrite(Trig, HIGH);         // Set the trigger pin HIGH to send a pulse
  delayMicroseconds(10);            // Hold the pulse for 10 microseconds
  digitalWrite(Trig, LOW);          // Set the trigger pin LOW again

  // Measure the duration of the echo pulse
  long duration = pulseIn(Echo, HIGH); // Read the time (in microseconds) for the echo to return

  // Calculate the distance in centimeters
  // Sound travels at 343 m/s (or 0.034 cm/µs), and the signal travels to the object and back (hence divided by 2)
  return duration * 0.034 / 2; 
}

//FUNCTION FOR BUZZER 
void playArrivedTone(){
  tone(Buzzer, 1000, 200);
  delay(250);
  tone(Buzzer, 1500, 200);
  delay(250);
  tone(Buzzer, 2000, 200);
  delay(250);
  noTone(Buzzer);
  return;
}

//FUNCTION TO GET THE WEIGHT OF THE ROBOT 

void getWeight(){
  int Level1 = digitalRead(BeamSensorLevel1);
  int Level2 = digitalRead(BeamSensorLevel2);
  
  if ( Level1  == 1 && Level1 == 1 ){
      while( Level1  == 1 && Level1 == 1){
         //overload 
         //LCD tell user to remove some dishes 
         displayWeightInstructions();
      }
  }
  else if ( Level1 == 0 && Level2 == 1){
    //weight unbalanced put dishes below 
     while( Level1 == 0 && Level2 == 1 ){
      // tell user to put some of the dishes below unbalanced load
      displayWeightInstructions();
     }
  }
  else{
    //good payload 
  }

  //displayWeightInstructions();
  delay(250); //enough time for user to finish sorting before  going back 
}
//LCD FUNCTIONS 

//LCD FUNCTIONs 
 void drawInstructionScreen(){

  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(1); // Adjust as needed to fit the screen
  tft.setTextColor(TFT_WHITE, TFT_BLACK); 
  tft.setTextDatum(MC_DATUM);
  
  // Draw instructions
   tft.drawString("Create WIFI - OK", screenWidth / 2, screenHeight / 3);
   tft.drawString("Set Pass - 123123123", screenWidth / 2, screenHeight / 2);
   return;

 }

void drawWelcomePage(){
     tft.fillScreen(TFT_BLACK);
     drawEyes();
     tft.setTextSize(2); // Adjust text size as needed tft.setTextColor(TFT_WHITE, TFT_BLACK); 
     tft.setTextDatum(MC_DATUM); 
     tft.drawString("JaxBOT", screenWidth / 2, screenHeight * 3 / 4);
 }


void drawEyes() {
   // Left Eye
      tft.fillCircle(screenWidth / 2 - eyeXOffset, screenHeight / 2 - eyeYOffset, eyeRadius, TFT_BLUE); 
      tft.fillCircle(screenWidth / 2 - eyeXOffset, screenHeight / 2 - eyeYOffset, pupilRadius, TFT_BLACK);
        // Right Eye 
      tft.fillCircle(screenWidth / 2 + eyeXOffset, screenHeight / 2 - eyeYOffset, eyeRadius, TFT_BLUE);
      tft.fillCircle(screenWidth / 2 + eyeXOffset, screenHeight / 2 - eyeYOffset, pupilRadius, TFT_BLACK);
   }

   
void drawEyesAngry() {
   // Left Eye
      tft.fillCircle(screenWidth / 2 - eyeXOffset, screenHeight / 2 - eyeYOffset, eyeRadius, TFT_RED); 
      tft.fillCircle(screenWidth / 2 - eyeXOffset, screenHeight / 2 - eyeYOffset, pupilRadius, TFT_BLACK);
        // Right Eye 
      tft.fillCircle(screenWidth / 2 + eyeXOffset, screenHeight / 2 - eyeYOffset, eyeRadius, TFT_RED);
      tft.fillCircle(screenWidth / 2 + eyeXOffset, screenHeight / 2 - eyeYOffset, pupilRadius, TFT_BLACK);
   }

void displayWeightInstructions() {
  int Level1 = digitalRead(BeamSensorLevel1);
  int Level2 = digitalRead(BeamSensorLevel2);

  tft.fillScreen(TFT_BLACK); 
  tft.setTextSize(1); 
  tft.setTextColor(TFT_WHITE, TFT_BLACK); 
  tft.setTextDatum(MC_DATUM); 

  if (Level1 == 1 && Level2 == 1) {
    tft.drawString("OVERLOADED!", screenWidth / 2, screenHeight / 3);
    tft.drawString("Remove some dishes.", screenWidth / 2, screenHeight / 2); 
  } else if (Level1 == 0 && Level2 == 1) {
    tft.drawString("UNBALANCED!", screenWidth / 2, screenHeight / 3);
    tft.drawString("Distribute weight evenly.", screenWidth / 2, screenHeight / 2); 
  } else {
    tft.drawString("Good payload!", screenWidth / 2, screenHeight / 3);
    tft.drawString("Ready to go!", screenWidth / 2, screenHeight / 2); 
  }
}

