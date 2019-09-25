#include <SoftwareSerial.h>
int  a;
String pos[10];
int POS[10];
SoftwareSerial BT(10,11);
void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
BT.begin(9600);
pinMode(4,OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  BT.println("T");
  if(BT.available()){
    //Serial.println("RECIBE");
  pos[0]=BT.readStringUntil("\n");  //X
  //Serial.println("RECIBIDO X");
  /*while(!BT.available()){
    delay(1);
    Serial.println("Pillado en X");
  }*/
  pos[1]=BT.readStringUntil("\n");  //Y
  //Serial.println("RECIBIDO Y");
  /*while(!BT.available()){
    delay(1);
    Serial.println("Pillado en Y");
  }*/
    
  pos[2]=BT.readStringUntil("\n");  //Z
  //Serial.println("RECIBIDO Z");
  for(a=0;a<3;a++){ //Pasamos el string a int
    POS[a]=pos[a].toInt();
  Serial.print("String[");
  
    Serial.print(a);
    Serial.print("]  = ");
    Serial.println(pos[a]);
    Serial.print("Int[");
    Serial.print(a);
    Serial.print("]  = ");
    Serial.println(POS[a]);
  }
  }


}

