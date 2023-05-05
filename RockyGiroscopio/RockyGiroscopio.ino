#include <I2Cdev.h>
#include <Wire.h>
#include "MPU6050.h"

#define pin_pwmA 10  //5
#define pin_pwmB 6  //6
#define out0 11  //8
#define out1 9  //9
#define out2 8  //10
#define out3 7  //11
#define echo 4  //13
#define trig 5  //12
#define echoD 3  //13
#define trigD 2  //12


//valores para este laberinto
#define Ancho 32
#define distanciaFrenado 12
#define distanciaPared 7
#define multiplicadorTranqui 10
#define multiplicadorPeligro 24
#define tolerancia 1
 
MPU6050 sensor;

long duration;
double distanciaAdelante;
double distanciaDer;

long tiempo_previo, tiempo;

//FUNCIONES DE MOVIMIENTO
//Hacia adelante
void avanceAdelante(int pwm){
  Serial.println("Estoy avanzando sin guiado");
  Serial.println(pwm);
  analogWrite(pin_pwmA,pwm);
  analogWrite(pin_pwmB,pwm);
  digitalWrite(out0,LOW);
  digitalWrite(out1,HIGH);
  digitalWrite(out2,HIGH);
  digitalWrite(out3,LOW);
}

void avanceAdelanteGuiado(){
  Serial.println("Estoy avanzando");
  autoGuiado();
  digitalWrite(out0,LOW);
  digitalWrite(out1,HIGH);
  digitalWrite(out2,HIGH);
  digitalWrite(out3,LOW);
}

//Hacia la Derecha
void giroDerecha(int pwm){
   analogWrite(pin_pwmA,pwm);
   analogWrite(pin_pwmB,pwm);
   digitalWrite(out0,LOW);
   digitalWrite(out1,HIGH);
   digitalWrite(out2,LOW);
   digitalWrite(out3,HIGH);
}
void anguloDerecha(){
  int gx,gy,gz;
  float anguloZ=0, anguloZ_previo=0;
  Serial.println("estoy derecha");
  while(abs(anguloZ)<88){
    //Serial.println(abs(anguloZ));
    giroDerecha(calcularPwmGiro(abs(anguloZ)));
    sensor.getRotation(&gx,&gy,&gz);
    tiempo=millis()-tiempo_previo;
    tiempo_previo=millis();
    anguloZ = (gz/131)*tiempo/1000.0 + anguloZ_previo;
    anguloZ_previo=anguloZ;
  }
  if (anguloZ>92){
    correccionDer(anguloZ);
  }
}

void correcionDer(int anguloDesface){
  int gx,gy,gz;
  float anguloZ=0, anguloZ_previo=0;
  Serial.println("estoy derecha");
  while(abs(anguloZ)<anguloDesface-90){
    //Serial.println(abs(anguloZ));
    giroIzquierda(128);
    sensor.getRotation(&gx,&gy,&gz);
    tiempo=millis()-tiempo_previo;
    tiempo_previo=millis();
    anguloZ = (gz/131)*tiempo/1000.0 + anguloZ_previo;
    anguloZ_previo=anguloZ;
  }
}

//Hacia la Izquierda
void giroIzquierda(int pwm){
   analogWrite(pin_pwmA,pwm);
   analogWrite(pin_pwmB,pwm);
   digitalWrite(out0,HIGH);
   digitalWrite(out1,LOW);
   digitalWrite(out2,HIGH);
   digitalWrite(out3,LOW);
}
void anguloIzquierda(){
  int gx,gy,gz;
  float anguloZ=0, anguloZ_previo=0;
  Serial.println("estoy izuierda");
  while(abs(anguloZ)<88){
    //Serial.println(abs(anguloZ));
    giroIzquierda(calcularPwmGiro(abs(anguloZ)));
    sensor.getRotation(&gx,&gy,&gz);
    tiempo=millis()-tiempo_previo;
    tiempo_previo=millis();
    anguloZ = (gz/131)*tiempo/1000.0 + anguloZ_previo;
    anguloZ_previo=anguloZ;
  }
}
//Frenado
void detener(){
  Serial.println("Frene");
  digitalWrite(out0,LOW);
  digitalWrite(out1,LOW);
  digitalWrite(out2,LOW);
  digitalWrite(out3,LOW);
  delay(3000);
}

//FUNCIONES DE CALCULOS

//pwm en giro

int calcularPwmGiro(float angulo){
  if(angulo<50){
    return 255;  
  }
  else if(angulo>=50 && angulo<60){
    return 138
    ;
  }
  else{
    return 130;
  }
}

//Distancia Adelante
double CalcularDistancia(){
  digitalWrite(trig,LOW);
  delayMicroseconds(2);
  digitalWrite(trig,HIGH);
  delayMicroseconds(10);
  digitalWrite(trig,LOW);
  duration = pulseIn(echo,HIGH);
  return duration*0.034/2;
}
//Distancia Derecha
double CalcularDistanciaDer(){
  digitalWrite(trigD,LOW);
  delayMicroseconds(2);
  digitalWrite(trigD,HIGH);
  delayMicroseconds(10);
  digitalWrite(trigD,LOW);
  duration = pulseIn(echoD,HIGH);  //Tiempo en microsegundos
  return duration*0.034/2; //
}

//FUNCIONES ANTI-CHOQUE y CENTRADORAS

boolean encontrarCentro(int distanciaIni, int pwm){
  distanciaAdelante = CalcularDistancia();
  while((distanciaIni-distanciaAdelante)<(Ancho/5) ){
    avanceAdelante(pwm);
    distanciaAdelante = CalcularDistancia();
  }
  distanciaDer = CalcularDistanciaDer();
  if(distanciaDer>15){
    return true;
  }
  else{
    return false;
  }
}

int FrenadoAutomatico(int distanciaAdelante){
  if(distanciaAdelante>=29){
    return 255; //100%
  }else if(distanciaAdelante<29 && distanciaAdelante>=12){
    return 130; //
  }else if(distanciaAdelante<12 && distanciaAdelante>=distanciaFrenado){
    return 120; //
  }else{
    return 0;
  }
}
//Choques laterales
void autoGuiado(){
  distanciaAdelante = CalcularDistancia();
  int pwm = FrenadoAutomatico(distanciaAdelante);
  int pwmder=pwm;
  int pwmizq=pwm;
  
  distanciaDer = CalcularDistanciaDer();
  if (distanciaDer<=(Ancho/2)){
    if (distanciaDer<=distanciaPared){
      if (distanciaDer<=distanciaPared-tolerancia){
        pwmizq=pwmizq-(distanciaPared-distanciaDer)*multiplicadorPeligro;
      }
      else{
        pwmizq=pwmizq-(distanciaPared-distanciaDer)*multiplicadorTranqui;
      }
    }
    else if (distanciaDer>distanciaPared){
      if (distanciaDer>distanciaPared+tolerancia){
        pwmder=pwmder-(distanciaDer-distanciaPared)*multiplicadorPeligro;
      }
      else{
        pwmder=pwmder-(distanciaDer-distanciaPared)*multiplicadorTranqui;
      }
    }
  }
  analogWrite(pin_pwmA,pwmizq);
  analogWrite(pin_pwmB,pwmder);
}

//FUNCIONAMIENTO GENERAL
void setup() {
  Serial.begin(9600);
  Serial.println("hola");

  Wire.begin();
  sensor.initialize();

  if(sensor.testConnection()){
    Serial.println("Sensor iniciado");
  }else{
    Serial.println("Error al iniciar el sensor");
  }

  tiempo_previo=millis();

  pinMode(trig,OUTPUT);
  pinMode(echo,INPUT);
  pinMode(trigD,OUTPUT);
  pinMode(echoD,INPUT);
  
  pinMode(pin_pwmA,OUTPUT);
  analogWrite(pin_pwmA,0);
  pinMode(pin_pwmB,OUTPUT);
  analogWrite(pin_pwmB,0);
  
  pinMode(out0,OUTPUT);
  pinMode(out1,OUTPUT);
  pinMode(out2,OUTPUT);
  pinMode(out3,OUTPUT);
  digitalWrite(out0,LOW);//MOTOR A
  digitalWrite(out1,LOW);
  digitalWrite(out2,LOW);//MOTOR B
  digitalWrite(out3,LOW);

  delay(4000);
}

void loop() {
  //temporizadores
  unsigned long tiempo = 0;
  unsigned long tiempoActual = millis();

  distanciaDer = CalcularDistanciaDer();
  distanciaAdelante = CalcularDistancia();
  int pwm = FrenadoAutomatico(distanciaAdelante);
  //Serial.print("distancia: ");
  //Serial.println(distanciaAdelante);
  //float angulo = 0; 
   if(pwm!=0 && distanciaDer<=(Ancho/2)){ //Se mueve para adelante y no tiene lugar a la derecha 
      //Avance adelante con correciones de guiado
      avanceAdelanteGuiado();
   }else if(distanciaDer>(Ancho/2)){  //Se mueve adelante y tiene lugar a la derecha
      //Frene, gire derecha, avance
      if(encontrarCentro(distanciaAdelante,pwm)){//nos centra en el medio del pasillo para doblar
        detener();
        anguloDerecha();
        detener();
        
        distanciaDer = CalcularDistanciaDer();
        while(distanciaDer>(Ancho/2)){
          avanceAdelante(135);
          distanciaDer = CalcularDistanciaDer();
        }
      }
   }else if(pwm==0 && distanciaDer<=(Ancho/2)){ //No se mueve adelante y no tiene lugar a la derecha
      //Gire izquierda
      detener();
      anguloIzquierda();
      detener();
   }
}
