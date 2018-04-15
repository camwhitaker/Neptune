#include <Wire.h>
int SLAVE_ADDRESS = 0x04; //only one address per slave I2C device

//Condition for sending data
int condition;


//Lidar Globals
#include <LIDARLite.h>
unsigned long pulseWidth;
unsigned long newpulseWidth;
int lidar_cm;
long sensor, cm, inches;
int test_var;
byte distance_measures[2];
boolean Switch = false;
int measure;

//Ultrasonic Globals
int anVolt;
int anPin = A0;
long Sonic_sensor, Sonic_cm, Sonic_inches;
unsigned long Sonic_pulseWidth;
int digital = 31;
boolean U_Switch = false;
int U_measure;


//Camera Servo Motor Globals

#include <Servo.h>     //Servo Motor
Servo myservo;  // create servo object to control a servo 
                // twelve servo objects can be created on most boards
int motor_pos = 85;    // variable to store the default servo position
int pos_adj = 10; //variable to store the increment/decrement value


//Internal Temperature Globals

int sensePin = A5;  //This is the Arduino Pin that will read the sensor output
int sensorInput;    //The variable we will use to store the sensor input
double temp;        //The variable we will use to store temperature in celsius degrees. 
int int_temp;

//External Temperature Globals
#include <OneWire.h>
#include <DallasTemperature.h>
// Data wire is plugged into pin 28 on the Arduino
#define ONE_WIRE_BUS 28
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

int inPin=28; // define D28 as input pin connecting to DS18S20 S pin (Temperature Sensor)
OneWire ds(inPin);    //Temperature Sensor
byte i;
byte present = 0;
byte type_s;
byte data[12];
byte addr[8];

//float celsius, fahrenheit;
long celsius, fahrenheit;



//Scaling Lasers Globals

int laser1 = 24;   //laser #1
int laser2 = 25;   //laser #2

//Internal Lights Globals
int Int_lights = 8;     //internal light #1 pin
int int_brightness = 0;    // how bright the LED is
int int_fadeAmount = 51;    // how many points to fade the LED by (six pwm values, 0, 51, 102, 153, 204, 255)


//External Lights Globals
int lightpin1 = 12;  //external light #1 pin
int lightpin2 = 11;  //external light #2 pin
int lightpin3 = 10;  //external light #3 pin
int lightpin4 = 9;  //external light #4 pin
int brightness = 0;    // how bright the LED is
int fadeAmount = 51;    // how many points to fade the LED by (six pwm values, 0, 51, 102, 153, 204, 255)



//Batteries Monitoring Circuit Globals

//Condition for sending data
//int condition;
boolean ledOn = false;


void setup() {
  Serial.begin(9600); //Start the Serial Port at 9600 baud (default)
  Wire.begin(SLAVE_ADDRESS);	//Initialize as I2C slave
  // Register I2C callbacks
  Wire.onReceive(processMessage);
  Wire.onRequest(sendData);          //all data
  
  //Camera Servo Motor
  myservo.attach(13);  // attaches the servo on pin 6 to servo motor
  myservo.write(motor_pos); //motor to default position at start up
    
   //Lidar
    
  pinMode(2, OUTPUT); // Set pin 2 as trigger pin (LIDAR)
  digitalWrite(2, LOW); // Set trigger LOW for continuous read (LIDAR)
  pinMode(5, INPUT); // Set pin 5 as monitor pin (LIDAR)
  
  //Ultrasonic
  pinMode(digital, OUTPUT);
  
  
    //Lasers
    
  pinMode(laser1, OUTPUT);
  pinMode(laser2, OUTPUT);
  
  //External Temperature
  sensors.begin();
}


void loop() {
  Get_ExtTemperature();
  Serial.println("");
  Lidar_getDistance();
  Serial.println("");
  Ultra_getDistance();
  Serial.println("");
  Get_IntTemperature();
  Serial.println("");
  delay(2000); 
}

void processMessage(int bytecount)

{
      char order = Wire.read();
      
      
      //Internal Light Functions
           
      if (order == 'u') {
         
          toggle_All_Int_Lights_Up();
      }
      
      else if (order == 'd') {
      
          toggle_All_Int_Lights_Down();
      
      }
      
      else if (order == 'a') {
      
          Power_All_Int_Lights_On_or_Reset();
      
      }
      
      else if (order == 'b') {
        
          Power_All_Int_Lights_Off();
      
      }
      
      
      //External Light Functions
      
      else if (order == '5') {
        
        toggle_Left_Lights_up();
        
      }
      
      else if (order == '6') {
        
        toggle_Right_Lights_up();
      
      }
      
      else if (order == '7') {
        
        toggle_Left_Lights_down();
        
      }
      
      else if (order == '8') {
        
        toggle_Right_Lights_down();
        
      }
      
      else if (order == 'U') {
        
        toggle_All_Lights_up();
        
      }
      
      else if (order == 'D') {
        
        toggle_All_Lights_down();
        
      }
      
      else if (order == 'A') {
        
        Power_All_Lights_On_or_Reset();
        
      }
      
      else if (order == 'B') {
        
        Power_All_Lights_Off();
        
      }
      
      
      //Laser Functions
       
       else if (order == 'O'){
        Power_Lasers_On();
       }
       
       else if (order == 'P'){
        Power_Lasers_Off();
        
      }
      
      
      //Camera Servo Motor Functions
     
        else if (order == 'm'){
          
          Default_Motor_Pos();    //Default motor position
        
        }
  
        else if (order == 'r'){
          
          Adjust_Motor_Right();  //Adjust motor right   
        }
  
        else if (order == 'l'){
          
          Adjust_Motor_Left();   //Adjust motor left
        }   
      
      
        //Lidar 
        
        else if (order == 'L'){
          
          condition = 1;
        
        }
        
        
        //Ultrasonic
        
        else if (order == 'S'){
          
          condition = 2;
        
        } 
 
        //Internal Temperature Sensor
        
        else if (order == 'I'){
          
          condition = 3;
        
        }        
               
        
        //External Temperature Sensor
        
        else if (order == 'T'){
          
          condition = 4;
        
        }
      
}


void sendData(){
 
   if (condition == 1){
     send_LidarDistance();
   }
 
   else if (condition == 2){
     send_UltraDistance();
   }
 
   else if (condition == 3){
     send_IntTemp();
   }
   
   else if (condition == 4){
     
     send_ExtTemp();
   }
   
   
}        
      
      
      


//Lidar Functions
void Lidar_getDistance(){
  
  pulseWidth = pulseIn(5, HIGH); // Count how long the pulse is high in microseconds

  // If we get a reading that isn't zero, let's print it
  
  if(pulseWidth != 0)
  {
  
    newpulseWidth = pulseWidth / 10; // 10usec = 1 cm of distance
    test_var = 256;  //Eight bits is the limit, so it can support (2^8) or 256 values ranging from 0-255
   
    Serial.print(newpulseWidth); // Print the distance
    Serial.print(" cm (Lidar), ");
  
    inches = newpulseWidth/2.54;
    Serial.print(inches); // Print the distance
    Serial.println(" inches (Lidar)"); 
     
 
  }
  
  lidar_cm = newpulseWidth;

  delay(2000);
}


void send_LidarDistance(){
  
  
  Switch = !Switch;

  if (Switch == true)
  {
    Wire.write(lidar_cm/256);
  } else
    Wire.write(lidar_cm%256);
  }
  

}



//Ultrasonic Functions

void Ultra_getDistance(){
 anVolt = analogRead((anPin));
//cm = anVolt*22; //Takes bit count and converts it to cm

Sonic_cm = (anVolt*6)-20;

Sonic_inches = Sonic_cm/2.54;

//Sonic_pulseWidth = pulseIn(5, HIGH);
//Sonic_cm = Sonic_pulseWidth/18.2; //Takes the pulse width and tells Arduino it is equal to millimeters
//Sonic_inches = Sonic_cm/2.54; //Takes cm and converts it to inches }

Serial.print(Sonic_cm);
Serial.print(" cm (Ultrasonic), ");
Serial.print(Sonic_inches);
Serial.println(" inches (Ultrasonic)");
digitalWrite(digital, HIGH);


}

void send_UltraDistance(){
  Wire.write(Sonic_cm); //bit shift operator, in this case right shift
  
    U_Switch = !U_Switch;


  //Case 1: Divisible by 1, with division result is less than 256
   
  if ((Sonic_cm % 1 == 0) && ((Sonic_cm/1) <= 255) && (U_Switch == true)){ 
    
   U_measure = (Sonic_cm)/1;
   
  Wire.write(1);
  }
 
  //Case 2: Divisible by 2, with division result is less than 256
   
  else if ((Sonic_cm % 2 == 0) && ((Sonic_cm/2) <= 255) && (U_Switch == true)){ 
    
  U_measure = (Sonic_cm)/2;
   
  Wire.write(2);
  }
  

 
  //Case 3: Divisible by 3, with division result is less than 256
   
  else if ((Sonic_cm % 3 == 0) && ((Sonic_cm/3) <= 255) && (U_Switch == true)){ 
    
  U_measure = (Sonic_cm)/3;
   
  Wire.write(3);
  }
  
   
  
  //Case 4: Divisible by 4, with division result is less than 256

 else if ((Sonic_cm % 4 == 0) && ((Sonic_cm/4) <= 255) && (U_Switch == true)){ 
    
  U_measure = (lidar_cm)/3;
   
  Wire.write(4);
  }
  

  //Case 5: Divisible by 5, with division result is less than 256
   
 else if ((Sonic_cm % 5 == 0) && ((Sonic_cm/5) <= 255) && (U_Switch == true)){ 
    
  U_measure = (lidar_cm)/5;
   
  Wire.write(5);
  }
  

  //Case 6: Divisible by 6, with division result is less than 256
 
   
 else if ((Sonic_cm % 6 == 0) && ((Sonic_cm/6) <= 255) && (U_Switch == true)){ 
    
  U_measure = (Sonic_cm)/6;
   
  Wire.write(6);
  }
  
  
  //Case 7: Divisible by 7, with division result is less than 256
   
  else if ((Sonic_cm % 7 == 0) && ((Sonic_cm/7) <= 255) && (U_Switch == true)){ 
    
  U_measure = (Sonic_cm)/7;
   
  Wire.write(7);
  }
  
  
  //Case 8: Divisible by 8, with division result is less than 256

  else if ((Sonic_cm % 8 == 0) && ((Sonic_cm/8) <= 255) && (U_Switch == true)){ 
    
  U_measure = (Sonic_cm)/8;
   
  Wire.write(8);
  }
  


  //Case 9: Divisible by 9, with division result is less than 256
  
 else if ((Sonic_cm % 9 == 0) && ((Sonic_cm/9) <= 255) && (U_Switch == true)){ 
    
  U_measure = (Sonic_cm)/9;
   
  Wire.write(9);
  }
  

 //Case 10: Divisible by 10, with division result is less than 256
  
 else if ((Sonic_cm % 10 == 0) && ((Sonic_cm/10) <= 255) && (U_Switch == true)){ 
    
  U_measure = (Sonic_cm)/10;
   
  Wire.write(10);
  }
  
 //Case 11: Prime Numbers greater than 255 and less than 511
  
 else if ((Sonic_cm % Sonic_cm == 0) && ((Sonic_cm) > 255) && ((lidar_cm) <= 510) && (U_Switch == true)){ 
    
  U_measure = (Sonic_cm)/2;
   
  Wire.write(11);
  
}

 //Case 12: Prime Numbers greater than 510 and less than 2550
  
 else if ((Sonic_cm % Sonic_cm == 0) && ((Sonic_cm) > 510) && ((lidar_cm) <= 2550) && (U_Switch == true)){ 
    
  U_measure = (Sonic_cm)/10;
   
  Wire.write(12);
  
}

  
   else {
  
    Wire.write(U_measure);
  
  }
  
  

}


//Camera Servo Motor Functions

//Function for resetting motor to 90 degrees
void Default_Motor_Pos(){
  motor_pos = 85;
  myservo.write(motor_pos);
}

//Function for motor going from 0 to 180 degrees

void Adjust_Motor_Right(){
  
  motor_pos = motor_pos + pos_adj;
  
  if (motor_pos >= 0 || motor_pos <= 180){
    myservo.write(motor_pos);
  }
  
  else {
    motor_pos = 180;
    myservo.write(motor_pos);
  }
}

//Function for motor going from 180 to 0 degrees

void Adjust_Motor_Left(){
    
  motor_pos = motor_pos - pos_adj;
  
  if (motor_pos >= 0 || motor_pos <= 180){
    myservo.write(motor_pos);
  }
  else {
    motor_pos = 0;
    myservo.write(motor_pos);
  }
}


//Internal Temperature Functions

void Get_IntTemperature(){
  sensorInput = analogRead(sensePin);    //read the analog sensor and store it
  temp = (double)sensorInput / 1024;       //find percentage of input reading
  temp = temp * 5;                 //multiply by 5V to get voltage
  temp = temp - 0.5;               //Subtract the offset 
  temp = temp * 100;               //Convert to degrees 
  int_temp = temp;

  Serial.print("Internal Temperature: ");
  Serial.print(temp);
  Serial.println(" Celsius");
}

void send_IntTemp(){
  Wire.write(int_temp); //bit shift operator, in this case right shift
}


//External Temperature Functions

void Get_ExtTemperature(){

      if ( !ds.search(addr)) {
    //Serial.println("No more addresses.");
    //Serial.println();
    ds.reset_search();
    delay(250);
    return;
  }
  
  //Serial.print("ROM =");
  for( i = 0; i < 8; i++) {
    Serial.write(' ');
    //Serial.print(addr[i], HEX);
  }

  if (OneWire::crc8(addr, 7) != addr[7]) {
      //Serial.println("CRC is not valid!");
      return;
  }
  Serial.println();
 

  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1);        // start conversion, with parasite power on at the end
  
  delay(1000);     // maybe 750ms is enough, maybe not
  // we might do a ds.depower() here, but the reset will take care of it.
  
  present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE);         // Read Scratchpad


  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();

  }

  // Convert the data to actual temperature
  // because the result is a 16 bit signed integer, it should
  // be stored to an "int16_t" type, which is always 16 bits
  // even when compiled on a 32 bit processor.
  int16_t raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
  }
  celsius = (float)raw / 16.0;
  fahrenheit = celsius * 1.8 + 32.0;
  Serial.print("External Temperature: ");
  Serial.print(celsius);
  Serial.print(" Celsius, ");
 Serial.print(fahrenheit);
  Serial.println(" Fahrenheit");
  delay(100);
}

void send_ExtTemp(){
  Wire.write(celsius);    //celsius temperature

}


//Scaling Lasers Functions

void Power_Lasers_On(){
    digitalWrite(laser1, HIGH);
    digitalWrite(laser2, HIGH);
}
void Power_Lasers_Off(){
    digitalWrite(laser1, LOW);
    digitalWrite(laser2, LOW);
}


//Internal Lights Functions

void toggle_All_Int_Lights_Up(){
 
 int_brightness = int_brightness + int_fadeAmount;
 
 if (int_brightness >= 0 || int_brightness <= 255) {
    analogWrite(Int_lights, int_brightness);
     }
 else { 
    int_brightness = 0;
    analogWrite(Int_lights, int_brightness);
 }
 
}

void toggle_All_Int_Lights_Down(){
 
 int_brightness = int_brightness - int_fadeAmount;
 
 if (int_brightness >= 0 || int_brightness <= 255) {
    analogWrite(Int_lights, int_brightness);
     }
 else { 
    int_brightness = 255;
    analogWrite(Int_lights, int_brightness);
 }
 
}

void Power_All_Int_Lights_On_or_Reset(){
    int_brightness = 102;
    analogWrite(Int_lights, int_brightness);
}

void Power_All_Int_Lights_Off(){
    int_brightness = 0;
    analogWrite(Int_lights, int_brightness);
}



//External Lights Functions

void toggle_Left_Lights_up(){                 //Function to toggle left two external lights up

    brightness = brightness + fadeAmount;
 
   if (brightness >= 0 || brightness <= 255) {
    analogWrite(lightpin2, brightness);
    analogWrite(lightpin4, brightness);
  }
  
  else {
    brightness = 0;
    analogWrite(lightpin2, brightness);
    analogWrite(lightpin4, brightness);  
  }
}

void toggle_Right_Lights_up(){              //Function to toggle right two external lights up
    
   brightness = brightness + fadeAmount;
 
   if (brightness >= 0 || brightness <= 255) {
    analogWrite(lightpin1, brightness);
    analogWrite(lightpin3, brightness);
  }
  
  else {
    brightness = 0;
    analogWrite(lightpin1, brightness);
    analogWrite(lightpin3, brightness);  
  }
}

void toggle_Left_Lights_down(){               //Function to toggle left two external lights down
   
   brightness = brightness - fadeAmount;
 
 if (brightness >= 0 || brightness <= 255) {
    analogWrite(lightpin2, brightness);
    analogWrite(lightpin4, brightness);
  }
  
 else { 
    brightness = 255;
   analogWrite(lightpin2, brightness);
    analogWrite(lightpin4, brightness);  
 }
}

void toggle_Right_Lights_down(){              //Function to toggle right two external lights down

   brightness = brightness - fadeAmount;
 
 if (brightness >= 0 || brightness <= 255) {
    analogWrite(lightpin1, brightness);
    analogWrite(lightpin3, brightness);
  }
 else { 
    brightness = 255;
   analogWrite(lightpin1, brightness);
    analogWrite(lightpin3, brightness);  
 }
}


void toggle_All_Lights_up(){               //Function to toggle all external lights up
 brightness = brightness + fadeAmount;
 
 if (brightness >= 0 || brightness <= 255) {
    analogWrite(lightpin1, brightness);
    analogWrite(lightpin2, brightness);
    analogWrite(lightpin3, brightness);
    analogWrite(lightpin4, brightness);
  }
 else { 
    brightness = 0;
    analogWrite(lightpin1, brightness);
    analogWrite(lightpin2, brightness);
    analogWrite(lightpin3, brightness);
    analogWrite(lightpin4, brightness);   
 }

}

void toggle_All_Lights_down(){             //Function to toggle all external lights down
 
 brightness = brightness - fadeAmount;
 
 if (brightness >= 0 || brightness <= 255) {
    analogWrite(lightpin1, brightness);
    analogWrite(lightpin2, brightness);
    analogWrite(lightpin3, brightness);
    analogWrite(lightpin4, brightness);  
  }
 else { 
    brightness = 255;
    analogWrite(lightpin1, brightness);
    analogWrite(lightpin2, brightness);
    analogWrite(lightpin3, brightness);
    analogWrite(lightpin4, brightness);
   
 }
}

void Power_All_Lights_On_or_Reset(){      //Function to power all external lights on
    brightness = 102;
    analogWrite(lightpin1, brightness);
    analogWrite(lightpin2, brightness);
    analogWrite(lightpin3, brightness);
    analogWrite(lightpin4, brightness); 

}

void Power_All_Lights_Off(){              //Function to power all external lights off
    brightness = 0;
    analogWrite(lightpin1, brightness);
    analogWrite(lightpin2, brightness);
    analogWrite(lightpin3, brightness);
    analogWrite(lightpin4, brightness); 
}
