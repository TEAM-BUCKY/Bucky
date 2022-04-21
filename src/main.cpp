// Slave

#include <Arduino.h>
#include <Core.h>
#include <Wire.h>
#include <Serial_C.h>
#include <LightRing.h>

int S[5] = {33, 34, 35, 36, 29};  //A0, A1, A2, A3, A4 Respectively on Arduino

int CS = 30; 
int WR = 31;
int EN = 32;

int INPIN = 27; //IO PIN ON MEGAMUX 

void pinSelect(int pinnum){
  digitalWrite(WR, LOW);
    for (int x = 0; x<5; x++){
      byte state = bitRead(pinnum, x);
      digitalWrite(S[x], state);

    }
   
  digitalWrite(WR, HIGH);

}
void setup() {
  // put your setup code here, to run once:
  for (int x = 0; x < 5; x++) {
    pinMode(S[x], OUTPUT);
    digitalWrite(S[x], LOW);
  }
  pinMode(CS, OUTPUT);
  digitalWrite(CS, LOW);
  pinMode(WR, OUTPUT);
  digitalWrite(WR, LOW);
  pinMode(EN, OUTPUT);
  digitalWrite(EN, LOW);
  pinMode(INPIN, INPUT);
 
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  int x;
  for(int i = 0; i<24;i++){
    x=i;
    if (i> 12) {
      x = i + 4;
    }
  pinSelect(x);
  int val = analogRead(INPIN);
  if (val > 550) {
    Serial.println("out of bounds");
    continue;
    
  }
  else {
    Serial.println("In bounds");
  }
  }
  delay(100);
}

