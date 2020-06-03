#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <SoftwareSerial.h>

SoftwareSerial Bluetooth(3, 4); // Arduino(RX, TX) - HC-05 Bluetooth (TX, RX)
int servo1Pos, servo2Pos, servo3Pos, servo4Pos, servo5Pos, servo6Pos; // current position
int servo1PPos, servo2PPos, servo3PPos, servo4PPos, servo5PPos, servo6PPos; // previous position
int servo01SP[50], servo02SP[50], servo03SP[50], servo04SP[50], servo05SP[50], servo06SP[50]; // for storing positions/steps
int speedDelay = 40;
int index=0;
int pos0=172,pos180=565;
int n1,n2,n3,n4,n5,n6;
char dataIn;
Adafruit_PWMServoDriver servos = Adafruit_PWMServoDriver(0x40);

void setServo(uint8_t n_servo, int angulo) {
  int duty;
  duty=map(angulo,0,180,pos0, pos180);
  servos.setPWM(n_servo, 0, duty);  
  
}

void setup() {

  Bluetooth.begin(9600); // Default baud rate of the Bluetooth module
  Bluetooth.setTimeout(1);
  Serial.begin(9600);
  servos.begin();  
  servos.setPWMFreq(60); //Frecuecia PWM de 60Hz o T=16,66ms
  delay(20);
  /////////////////////////////////////////////////////////CAMBIO DE LAS POSICIONES INICIALES
  n1 = 58;                  //SERVO DE HASTA ABAJO
  setServo(0,n1);
  n2 = 100;
  setServo(1,n2);
  n3 = 57;
  setServo(3,n3);
  n4 = 90;
  setServo(4,n4);
  n5 = 90;
  setServo(5,n5);
  n6 = 90;
  setServo(6,n6);
}
void loop() {
  // Check for incoming data
  if (Bluetooth.available() > 0) {
    dataIn = Bluetooth.read();  // Read the data as string
    Serial.println(dataIn);
  }
    ////Servo 1
    
      if (dataIn=='a') {
          n6=180;
          setServo(6,n6);
          delay(40);    
        
      }
      if (dataIn=='b') {
          n6=0;
          setServo(6,n6);
          delay(40);   
        }
     ////Servo 2
    
      if (dataIn=='c') {
          n5=180;
          setServo(5,n5);
          delay(50);    
        
      }
      if (dataIn=='d') {
          n5=0;
          setServo(5,n5);
          delay(50);   
        }
        ////Servo 3
    
      if (dataIn=='e') {
          n4=180;
          setServo(4,n4);
          delay(30);    
        
      }
      if (dataIn=='f') {
          n4=0;
          setServo(4,n4);
          delay(30);   
        }
        ////Servo 4
    
      if (dataIn=='g') {
          n3++;
           if(n3>148){
            n3=148;
          }
          setServo(3,n3);
          delay(70);    
        
      }
      if (dataIn=='h') {
          n3--;
           if(n3<40){
            n3=40;
          }
          setServo(3,n3);
          delay(70);   
        }
    ////Servo 5
    
      if (dataIn=='i') {
          n2++;
        
          setServo(1,n2);
          delay(30);    
        
      }
      if (dataIn=='j') {
          n2--;
          setServo(1,n2);
          delay(30);   
        }
        ////Servo 6
    
      if (dataIn=='k') {
          n1++;
          //CAMBIO DE LIMITES
           if(n1>140){
            n1=140;
          }
          setServo(0,n1);
          delay(50);    
        
      }
      if (dataIn=='l') {
          n1--;
          ///CAMBIO DE LIMITES
           if(n1<-20){
            n6=-20;
          }
         
          setServo(0,n1);
          delay(50);   
        }

          ///FRENAR SERVOS 360
        if(dataIn=='z'){
          setServo(90,n6);
          setServo(90,n5);
          setServo(90,n4);
          
          
        }
        
        ///////////////////////////////////////////IMPRESION DE ANGULO DE SERVOS
        //N1 SERVO DE MAS ABAJO
        Serial.println(n1);
        
    if(dataIn=='s' || dataIn=='t' || dataIn=='r'){
    
    // If button "SAVE" is pressed
    if (dataIn=='s') {
      servo01SP[index] = n1;  
      servo02SP[index] = n2;
      servo03SP[index] = n3;
      servo04SP[index] = n4;
      servo05SP[index] = n5;
      servo06SP[index] = n6;
      index++;                        
    }
    // If button "RUN" is pressed
    if (dataIn=='r') {
      runservo();  // Automatic mode - run the saved steps 
    }
    // If button "RESET" is pressed
    if ( dataIn == 't') {
      memset(servo01SP, 0, sizeof(servo01SP)); // Clear the array data to 0
      memset(servo02SP, 0, sizeof(servo02SP));
      memset(servo03SP, 0, sizeof(servo03SP));
      memset(servo04SP, 0, sizeof(servo04SP));
      memset(servo05SP, 0, sizeof(servo05SP));
      memset(servo06SP, 0, sizeof(servo06SP));
      index = 0;  // Index to 0
    }
    dataIn='z';
  }}

// Automatic mode custom function - run the saved steps
void runservo() {
  while (dataIn != 't') {   // Run the steps over and over again until "RESET" button is pressed
    for (int i = 0; i <= index - 2; i++) {  // Run through all steps(index)
      if (Bluetooth.available() > 0) {      // Check for incomding data
        dataIn = Bluetooth.read();
        if ( dataIn == 'p') {           // If button "PAUSE" is pressed
          while (dataIn != 'r') {         // Wait until "RUN" is pressed again
            if (Bluetooth.available() > 0) {
              dataIn = Bluetooth.read();
              if ( dataIn == 't') {     
                break;
              }
            }
          }
        
        
      }
      }
      // Servo 1
      if (servo01SP[i] == servo01SP[i + 1]) {
      }
      if (servo01SP[i] > servo01SP[i + 1]) {
        for ( int j = servo01SP[i]; j >= servo01SP[i + 1]; j--) {
          setServo(0,j);
          delay(speedDelay);
        }
      }
      if (servo01SP[i] < servo01SP[i + 1]) {
        for ( int j = servo01SP[i]; j <= servo01SP[i + 1]; j++) {
          setServo(0,j);
          delay(speedDelay);
        }
      }
      // Servo 2
      if (servo02SP[i] == servo02SP[i + 1]) {
      }
      if (servo02SP[i] > servo02SP[i + 1]) {
        for ( int j = servo02SP[i]; j >= servo02SP[i + 1]; j--) {
          setServo(1,j);
          delay(speedDelay);
        }
      }
      if (servo02SP[i] < servo02SP[i + 1]) {
        for ( int j = servo02SP[i]; j <= servo02SP[i + 1]; j++) {
          setServo(1,j);;
          delay(speedDelay);
        }
      }
      // Servo 3
      if (servo03SP[i] == servo03SP[i + 1]) {
      }
      if (servo03SP[i] > servo03SP[i + 1]) {
        for ( int j = servo03SP[i]; j >= servo03SP[i + 1]; j--) {
          setServo(3,j);
          delay(speedDelay);
        }
      }
      if (servo03SP[i] < servo03SP[i + 1]) {
        for ( int j = servo03SP[i]; j <= servo03SP[i + 1]; j++) {
          setServo(3,j);
          delay(speedDelay);
        }
      }
      // Servo 4
      if (servo04SP[i] == servo04SP[i + 1]) {
      }
      if (servo04SP[i] > servo04SP[i + 1]) {
        for ( int j = servo04SP[i]; j >= servo04SP[i + 1]; j--) {
          setServo(4,j);
          delay(speedDelay);
        }
      }
      if (servo04SP[i] < servo04SP[i + 1]) {
        for ( int j = servo04SP[i]; j <= servo04SP[i + 1]; j++) {
          setServo(4,j);
          delay(speedDelay);
        }
      }
      // Servo 5
      if (servo05SP[i] == servo05SP[i + 1]) {
      }
      if (servo05SP[i] > servo05SP[i + 1]) {
        for ( int j = servo05SP[i]; j >= servo05SP[i + 1]; j--) {
          setServo(5,j);
          delay(speedDelay);
        }
      }
      if (servo05SP[i] < servo05SP[i + 1]) {
        for ( int j = servo05SP[i]; j <= servo05SP[i + 1]; j++) {
          setServo(5,j);
          delay(speedDelay);
        }
      }
      // Servo 6
      if (servo06SP[i] == servo06SP[i + 1]) {
      }
      if (servo06SP[i] > servo06SP[i + 1]) {
        for ( int j = servo06SP[i]; j >= servo06SP[i + 1]; j--) {
          setServo(6,j);
          delay(speedDelay);
        }
      }
      if (servo06SP[i] < servo06SP[i + 1]) {
        for ( int j = servo06SP[i]; j <= servo06SP[i + 1]; j++) {
          setServo(6,j);
          delay(speedDelay);
        }
      }
    }
  }
}
