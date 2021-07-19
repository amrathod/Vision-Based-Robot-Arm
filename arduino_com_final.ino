/*
 * This sketch is used to control the braccio arm with feed from raspberry pi for the task
 * of picking and placing items.
 * The whole pick and place takes place autonomously using Pi Camera.
 * ------------------------------------------
 * Authors: Aniket Somwanshi, Joseph Peltroche, Abhishek Rathod
 * Date: 11th May 2021
 */


#include <Braccio.h> //Including the Braccio Standard Library for Braccio Arm
#include <Servo.h> //Including servo.h for controlling servo motions 

//declaring all servos
Servo base;
Servo shoulder;
Servo elbow;
Servo wrist_ver;
Servo wrist_rot;
Servo gripper;

//Initialize angles to the safety position of Arm
int m1 = 90;
int m2 = 120;
int m3 = 20;
int m4 = 30;
int m5 = 0;
int m6 = 10;

int wor = 2; //Initializing variable to determine the classification of the object in the environment

int flag = 1; //Initializing flag to send to Raspberry Pi after droppin off the object 

//Initialize strings  
String m1_st ="";
String m2_st ="";
String m3_st ="";
String m4_st ="";
String m5_st ="";
String m6_st ="";
String wor_st ="";

// initialize comma position to seperate data from received string
int comma_position;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);      //Begin Serial Communication
  
  Braccio.begin();        //Begin Braccio Arm
  Braccio.ServoMovement(20,         90, 120, 20, 30, 0, 10);    // Set braccio arm to safety position
}

void loop() {
  // put your main code here, to run repeatedly:
  Braccio.ServoMovement(20,         m1, m2, m3, m4, m5, m6);    //move braccio arm to desired position(safety)
  
  if(Serial.available()>0){                                     //checks if Raspberry pi is sending data. Loop will run if data is available.

    String data = Serial.readStringUntil('\n');                 // reads data upto /n and stores in string 'data' 

 
    // Reading value of m1 from the string
    comma_position = data.indexOf(',');   
    m1_st = data.substring(0,comma_position);                   //seperates data of m1 i.e 0 to comma position
    m1 = m1_st.toInt();                                         //converts data to int
    data = data.substring(comma_position+1, data.length());     //stores only rest of data in the variable data i.e. removes the data that was read.

    // Reading value of m2 from the string
    comma_position = data.indexOf(',');   
    m2_st = data.substring(0,comma_position);
    m2 = m2_st.toInt();
    data = data.substring(comma_position+1, data.length());

   //  Reading value of m3 from the string
    comma_position = data.indexOf(',');   
    m3_st = data.substring(0,comma_position);
    m3 = m3_st.toInt();
    data = data.substring(comma_position+1, data.length());

    //  Reading value of m4 from the string
    comma_position = data.indexOf(',');   
    m4_st = data.substring(0,comma_position);
    m4 = m4_st.toInt();
    data = data.substring(comma_position+1, data.length());
 
    
     //  Reading value of waste or recyclable from the string
    comma_position = data.indexOf(',');   
    wor_st = data.substring(0,comma_position);
    wor = wor_st.toInt();
    data = data.substring(comma_position+1, data.length());



    //digitalWrite(data, LOW);.   
    //                    step delay, M1, M2, M3, M4, M5, M6:  
    Braccio.ServoMovement(20,         m1, m2, m3, m4, m5, m6);  // sets braccio arm to angles received from the Raspberry Pi

    if(wor == 0){                                               // checks the classification parameter received   
     Braccio.ServoMovement(20,         m1, m2, m3, m4, m5, 10); // sets braccio arm to angles received from the Raspberry Pi with gripper open
     delay(50);
     Braccio.ServoMovement(20,         m1, m2, m3, m4, m5, 73); // close the gripper
     delay(10);
     Braccio.ServoMovement(20,         5, 60, 60, 90, 0, 73);   // move to the waste bin
     Braccio.ServoMovement(20,         5, 60, 60, 90, 0, 10);   //open gripper to drop the object.
     
     //sets angles for safety position.
     m1 = 90;
     m2 = 120;
     m3 = 20;
     m4 = 30;
     m5 = 0;
     m6 = 10;   

     Serial.print(flag);                                        //send flag back to Raspberry Pi showing that the object is picked and dropped.
    }
    else if(wor == 1){                                            // checks the classification parameter received 
     Braccio.ServoMovement(20,         m1, m2, m3, m4, m5, 10);   // sets braccio arm to angles received from the Raspberry Pi with gripper open
     delay(50);
     Braccio.ServoMovement(20,         m1, m2, m3, m4, m5, 73);   // close the gripper
     delay(10);
     Braccio.ServoMovement(20,         175, 60, 60, 90, 0, 73);   // move to the recycle bin
     Braccio.ServoMovement(20,         175, 60, 60, 90, 0, 10);   //open gripper to drop the object
     
     //sets angles for safety position.
     m1 = 90;
     m2 = 120;
     m3 = 20;
     m4 = 30;
     m5 = 0;
     m6 = 10;

     Serial.print(flag);                                      //send flag back to Raspberry Pi showing that the object is picked and dropped.
       
    }
    
  }

}
