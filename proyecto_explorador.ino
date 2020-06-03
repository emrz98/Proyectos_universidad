#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>
Adafruit_PWMServoDriver servo = Adafruit_PWMServoDriver(0x40);
unsigned int pos0=172; // ancho de pulso en cuentas para pocicion 0°
unsigned int pos180=565; // ancho de pulso en cuentas para la pocicion 180°

int M11=3,M12=4, M21=5, M22=6; //Motores traslado

//Servomotores
int Motor;
int gradosMotor;
int gradosMotor0;
int gradosMotor1;
int gradosMotor2;
int gradosMotor3;
int gradosMotor4;

unsigned long tiempoAnterior=0;
unsigned long Periodo=100;

void setup() {
  Serial.begin(9600);
  servo.begin();
  servo.setPWMFreq(60);
  delay(10);   
  pinMode(13,OUTPUT);
  digitalWrite(13,LOW);
}


void loop() {
  char mensaje;
      if(Serial.available()>0){
        mensaje=Serial.read();
      }
      controlMotores(mensaje);
      if(millis()-tiempoAnterior>Periodo)
      {
        controlServos(mensaje);  
        
        setServo(Motor,gradosMotor);
        tiempoAnterior=millis();
      }
     
}

 
 void controlServos(char mensaje){
  switch(mensaje){
    case 'a':
    gradosMotor0++;
    break;
    
    case 'b':
    gradosMotor0--;
    break;
    
    case 'c':
    gradosMotor1++;
    break;
    
    case 'd':
    gradosMotor1--;
    break;
    
    case 'e':
    gradosMotor2++;
    break;
    
    case 'f':
    gradosMotor2--;
    break;
  }
if(mensaje=='a' || mensaje=='b'){
  Motor=0;
  gradosMotor=gradosMotor0;
}
if(mensaje=='c' || mensaje=='d' ){
  Motor=1;
  gradosMotor=gradosMotor1;
}
if(mensaje=='e' || mensaje=='f' ){
  Motor=2;
  gradosMotor=gradosMotor2;
}
  
 }
void controlMotores(char mov){
  if(mov=='v'){   //Adelante
    digitalWrite(M11,LOW);
    digitalWrite(M12,LOW);
    digitalWrite(M21,LOW);
    digitalWrite(M22,LOW);
   }
   else if(mov=='w'){   //Adelante
    digitalWrite(M11,HIGH);
    digitalWrite(M12,LOW);
    digitalWrite(M21,HIGH);
    digitalWrite(M22,LOW);
   }
  else if(mov=='x'){   //Atras
     digitalWrite(M11,LOW);
    digitalWrite(M12,HIGH);
    digitalWrite(M21,LOW);
    digitalWrite(M22,HIGH);
   }
   else if(mov=='y'){   //izquierda
     digitalWrite(M11,LOW);
    digitalWrite(M12,LOW);
    digitalWrite(M21,HIGH);
    digitalWrite(M22,LOW);
   }
   else if(mov=='z'){   //derecha
     digitalWrite(M11,HIGH);
    digitalWrite(M12,LOW);
    digitalWrite(M21,LOW);
    digitalWrite(M22,LOW);
   }

   
  }

void setServo(uint8_t n_servo, int angulo) {
  int duty;
  duty=map(angulo,0,180,pos0, pos180);
  servo.setPWM(n_servo, 0, duty);  
}

  void servoSuaveAbrir(int s1,int s2){
  for(int i=0;i<100;i++){
    s1--;
    s2--;
    servo1.write(s1);
    servo2.write(s2);
    delay(50);
  }
  abierto=true;
  cerrado=false;

 Serial.println(s1);
  Serial.println(s2);
}
