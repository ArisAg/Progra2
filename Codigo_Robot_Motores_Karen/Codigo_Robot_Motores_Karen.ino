#include <SoftwareSerial.h> 
#include <Servo.h> 

SoftwareSerial BT(8,9); 

int motorderecho1 = 4;
int motorderecho2 = 5;
int motorizquierdo1 = 6;
int motorizquierdo2 = 7;

int Mover_Adelante();
int Mover_Retroceso();
int Mover_Stop();
int Mover_Derecha();
int Mover_Stop();
int Mover_Izquierda();
int Mover_Stop();
int Subir_hombro();
int Bajar_hombro();

int grado = 90;
int grado1 = 90;

Servo servo;
Servo servo1;

void setup() {
  
 BT.begin(9600);

 pinMode(motorderecho1, OUTPUT);
 pinMode(motorderecho2, OUTPUT);
 pinMode(motorizquierdo1, OUTPUT);
 pinMode(motorizquierdo2, OUTPUT);
 servo.attach(10,750,1800);
 servo1.attach(11,750,1800);
 servo.write(grado);
 servo1.write(grado1);


}

void loop() {

if (Serial.available()) {
     char dato= Serial.read();
     if(dato=='a')
     {
        Mover_Adelante();
      
     }
     else if(dato=='r')
     { 
        Mover_Retroceso();
       
     }
     else if(dato=='d')
     { 
        Mover_Derecha();
       
     }
     else if(dato=='i')
     { 
        Mover_Izquierda();
       
     }   
     else if(dato=='s')
     { 
        Subir_hombro();
        
     }   
      else if(dato=='b')
     { 
        Bajar_hombro();
        
     }   

  else   
  {
    Mover_Stop();
  }
  
  
  delay(1); 
  
}


void Mover_Adelante()

{
  digitalWrite(motorderecho1, HIGH);
  digitalWrite(motorderecho2, LOW);
  digitalWrite(motorizquierdo1, LOW);
  digitalWrite(motorizquierdo1, HIGH);   
 
}

void Mover_Retroceso()

{
  digitalWrite(motorderecho1, LOW);
  digitalWrite(motorderecho2, HIGH);
  digitalWrite(motorizquierdo1, LOW);
  digitalWrite(motorizquierdo1, HIGH); 

}
void Mover_Derecha()

{   
  digitalWrite(motorderecho1, LOW);
  digitalWrite(motorderecho2, HIGH);
  digitalWrite(motorizquierdo1, HIGH);
  digitalWrite(motorizquierdo1, LOW);
}
void Mover_Izquierda()

{
  digitalWrite(motorderecho1, HIGH);
  digitalWrite(motorderecho2, LOW);
  digitalWrite(motorizquierdo1, LOW);
  digitalWrite(motorizquierdo1, HIGH);
}

void Mover_Stop() 
 {
  digitalWrite(motorderecho1, LOW);
  digitalWrite(motorderecho2, LOW);
  digitalWrite(motorizquierdo1, LOW);
  digitalWrite(motorizquierdo1, LOW);  
 }
void Subir_hombro()

 {
    grados++;
    if (grados >= 180)
    {
      grados = 180;
    }
    mi_servo.write(grados);
    delay(10);
}    
  void Bajar_hombro()
{
    grados--;
    if (grados <= 0)
    {
      grados = 0;
    }
    mi_servo.write(grados);
    delay(10);  
}
 
