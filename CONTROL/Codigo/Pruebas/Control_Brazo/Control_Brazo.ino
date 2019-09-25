#include <Servo.h> //Librería para controlar los servos
#include <SoftwareSerial.h> //Hay que añadirla a la carpeta correspondiente

//Defines para cambiar los pines de los servos facilmente
#define SERVO1 5
#define SERVO2 6
#define SERVO3 9
#define SERVO4 10
#define SERVO5 11 
#define SERVO_GARRA 3

//Defines de los puertos del puerto serie
//Hay que comprobar si estan al reves
#define tx 0
#define rx 1

// Definición de las barras de nuestro manipulador
#define L0 0.058
#define L1 0.015
#define L2 0.105
#define L3 0.075
#define L4 0.06
#define L5 0.06

#define OFFSET1 85
#define OFFSET2 107
#define OFFSET3 106
#define OFFSET4 1
#define OFFSET5 1

#define pi PI
#define GARRA 0.075
#define t_grado 10  //Tiempo que tarda un servo en moverse un grado (ms)

//Declaramos los servos
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;
Servo servo_garra;

//Vectores de posiciones
String pos[10];
float POS[10];
float q[10];

//Bandera para comunicacion con BT
bool flag=true;

//Estado de la FSM del main
int estado=1;

//Para calcular el tiempo de movimiento
float q_ant[10];


void mueve_robot(float q[])
{
  int i,j,l;  //Contador para el bucle
  float q_max=0.;  //Maximo angulo girado para llegar al punto
  unsigned int tiempo;  //Tiempo estimado en el que se alcanza el punto
  //Restringimos y mapeamos
  for(i=0;i<4;i++)
  {
    q[i]=constrain(q[i],0,180);
    q_max=max(abs(q[i]-q_ant[i]),q_max); //Se coge el angulo maximo que se va a girar para ese punto
    
  }
  for(j=0;j<4;j++)
  {
    q_ant[j]=q[j];  //Guardamos el valor anterior para calcular el angulo recorrido
    q[i]=map(q[i],0,180,0,255);
  }
  tiempo=q_max*t_grado*1.1;

  //Movemos los servos
  servo1.write(q[0]);
  servo2.write(q[1]);
  servo3.write(q[2]);
  //servo4.write(q[3]);
  //servo5.write(q[4]);
  
  delay(tiempo);
}

void convierte_grados(float q[])  //Funcion que transforma las variables articulares de radianes a grados, y ademas añade el offset
{
  int i;
  int offset[] = {OFFSET1,OFFSET2,OFFSET3,OFFSET4};
  for(i=0;i<3;i++)
  {
    q[i]=(q[i]*180.0)/pi;
    q[i]+=offset[i];
  }
}

void MCI ( float POS[])  // Función que emula el modelo cinemático inverso del manipulador
{
  float x, y, z, d, r, alpha, beta, gamma, q1, q2, q3, q4, q5; // Definición de las variables necesarias
  x = POS[0]; y = POS[1]; z = POS[2]-L0;            // Archivo el vector de inicio en las variables correspondientes
  r = sqrt(pow(x,2)+pow(y,2));                         // Cálculo  de la distancia reflejada en el plano XY 
  d = sqrt(pow(r,2)+pow(z,2));                // Cálculo la distancia en línea recta desde la base hasta el extremo del posicionador 

  alpha = atan(z/fabs(r));       // Ángulo auxiliar alpha
  beta = atan(fabs((sqrt(fabs(1-pow(pow(d,2)+pow(L2,2)-pow(L3,2)/(2*d*L2),2)))/((pow(d,2)+pow(L2,2)-pow(L3,2))/(2*d*L2)))));
  gamma = atan(fabs((sqrt(fabs(1-pow(pow(L3,2)+pow(L2,2)-pow(d,2)/(2*L2*L3),2)))/((pow(L3,2)+pow(L2,2)-pow(d,2))/(2*L2*L3)))));
  q2 = alpha + beta;                  // Articulación 2
  q3 = gamma;                         // Articulación 3
  q1 = atan(fabs(y/x)); // Articulación 1, que depende de las anteriores
  q4 = -q2-q3; // Articulción 4 que controla la rotación respecto al eje perpendicular a la garra en la sujeción (mantenemos en horizontal)
  q5 = 0; //Articulación 5 (Mantanemos sin inclinar respecto al eje longitudinal de la garra)
  q[0] = q1; q[1] = q2; q[2] = q3; q[3] = q4; q[4] = q5;    // Exportamos las articulaciones en un vector q[], declarado globalmente
  convierte_grados(q);
}

void recibe_BT()  //Hay que convertir de string a int
{
  int a,aux;
  pos[0]=Serial.readStringUntil("\n");  //X
  while(!Serial.available())
    delay(1);
  pos[1]=Serial.readStringUntil("\n");  //Y
  while(!Serial.available())
    delay(1);
  pos[2]=Serial.readStringUntil("\n");  //Z
  for(a=0;a<3;a++)  //Pasamos el string a int
    POS[a]=pos[a].toFloat();
}

void abre_garra()
{
  int valor=20;
  valor=map(valor,0,180,0,255);
  servo_garra.write(valor);
}

void cierra_garra()
{
  int valor=160;
  valor=map(valor,0,180,0,255);
  servo_garra.write(valor);
}

SoftwareSerial BT(rx,tx);
void envia_BT(bool flag){
  if(flag)
    BT.println("T");
  else
    BT.println("F");
  delay(10);  //Hay que comprobar si es suficiente
}


void setup() {
  // put your setup code here, to run once:
  servo1.attach(SERVO1);
  servo2.attach(SERVO2);
  servo3.attach(SERVO3);
  servo4.attach(SERVO4);
  servo5.attach(SERVO5);
  servo_garra.attach(SERVO_GARRA);
  Serial.begin(9600);
  BT.begin(9600);
}

void loop() {
  switch(estado)
{
  /*case 0: //Primera ejecucion
    envia_BT(flag);
    if(Serial.available())
      estado=2;
    break;*/  
  case 1: //Reposo
    if(Serial.available())
      estado++;
    break;
  case 2: //Recibimos los datos desde la app
    recibe_BT();
    flag=false;
    estado++;
    break;
  case 3: //Envio de estado hacia la app
    envia_BT(flag);
    if(flag==false)
      estado++;
    else
      estado=1;
    break;
  case 4: //Cinematica inversa del brazo
    MCI(POS);
    estado++;
    break;
  case 5: //Transformamos las coordenadas cartesianas en movimiento para las articulaciones
    mueve_robot(q);
    flag=true;
    estado=3;
    break;
    }
    Serial.println(estado);
}
