la mama de roman
#include <CO2Sensor.h>
CO2Sensor co2Sensor(A0, 0.99, 100);

// Sensor de ph
const int analogInPin = A1; 
int sensorValue = 0; 
unsigned long int avgValue; 
float b;
int buf[10],temp;

void setup() {
  Serial.begin(9600);
  co2Sensor.calibrate();

}

void loop() { 

  for(int i=0;i<10;i++) 
  { 
    buf[i]=analogRead(analogInPin);
    delay(10);
  }
  for(int i=0;i<9;i++)
  {
    for(int j=i+1;j<10;j++)
    {
     if(buf[i]>buf[j])
      {
        temp=buf[i];
        buf[i]=buf[j];
        buf[j]=temp;
      }                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   
    }
  }
 avgValue=0;
 for(int i=2;i<8;i++)
 avgValue+=buf[i];
 float pHVol=(float)avgValue*5.0/1024/6;
 float phValue = -6.33 * pHVol + 23.07;
 int medida = co2Sensor.read();
 //Serial.print(pHVol);
 //Serial.print("sensor = ");
 Serial.print(phValue);
 Serial.print(",");
// Serial.print("Concentracion CO2: ");
 Serial.println(medida);


  delay(500);
 
}
