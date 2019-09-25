
#include <Servo.h> //Librería para controlar los servos

//Defines para cambiar los pines de los servos facilmente
#define SERVO1 5
#define SERVO2 6
#define SERVO3 9
#define SERVO4 10
#define SERVO5 11 //¿QUÉ PUERTO ES?
#define SERVO_GARRA 3

// Definición de las barras de nuestro manipulador
#define L0 0.058
#define L1 0.015
#define L2 0.105
#define L3 0.075
#define L4 0.06
#define L5 0.06
#define GARRA 0.075

#define t_grado 10  //Tiempo que tarda un servo en moverse un grado (ms)

#define pi 3.141592653589793

//Declaramos los servos
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;
Servo servo_garra;

//Declarar las dos posiciones de prueba
 float POS1[] = {0.0325,-0.0562,0.2155};
 float POS2[] = {0,-0.0125,0.2014};

//Vectores de posiciones
float pos[10];
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
    //q[i]=map(q[i],0,180,0,255);
  }
  tiempo=(q_max*t_grado)*1.1;

  //Movemos los servos
  servo1.write(q[0]);
  servo2.write(q[1]);
  servo3.write(q[2]);
  servo4.write(q[3]);
  servo5.write(q[4]);
  
  //delay(tiempo);  //Hay que comprobar si la funcion write espera a que llegue a la posicion o no. IMPORTANTE
}

void MCI ( float POS[])  // Función que emula el modelo cinemático inverso del manipulador
{
  float x, y, z, d, r, alpha, beta, gamma, q1, q2, q3, q4, q5; // Definición de las variables necesarias
  x = POS[0]; y = POS[1]; z = POS[2]-L0;            // Archivo el vector de inicio en las variables correspondientes
  r = sqrt(pow(x,2)+pow(y,2))-L1;                         // Cálculo  de la distancia reflejada en el plano XY 
  d = sqrt(pow(r,2)+pow(z,2));                // Cálculo la distancia en línea recta desde la base hasta el extremo del posicionador 

  alpha = atan(z/fabs(r));       // Ángulo auxiliar alpha
  beta = atan(fabs(sqrt(fabs(4*pow(L2,2)*pow(d,2)-pow(pow(L2,2)+pow(d,2)-pow(L3,2),2)))/(pow(L2,2)+pow(d,2)-pow(L3,2)))); // Ángulo auxiliar beta
  gamma = atan(fabs(sqrt(fabs(4*pow(L2,2)*pow(L3,2)-pow(pow(L2,2)+pow(L3,2)-pow(d,2),2)))/(pow(L2,2)+pow(L3,2)-pow(d,2)))); // Ángulo auxiliar gamma
  q2 = alpha + beta;                  // Articulación 2
  q3 = gamma;                         // Articulación 3
  q1 = atan((y/(L3*cos(q2 + q3) + L2*cos(q2)))/(x/(L3*cos(q2 + q3) + L2*cos(q2)))); // Articulación 1, que depende de las anteriores
  q4 = PI/2 - q2 - q3; // Articulción 4 que controla la rotación respecto al eje perpendicular a la garra en la sujeción (mantenemos en horizontal)
  q5 = 0; //Articulación 5 (Mantenemos sin inclinar respecto al eje longitudinal de la garra)
  q[0] = (q1*180/pi)+135; q[1] = (q2*180/pi)+20; q[2] = (q3*180/pi)+70; q[3] = (q4*180/pi)+120; q[4] = q5*180/pi;    // Exportamos las articulaciones en un vector q[], declarado globalmente
}

void recibe_BT()
{
  int aux;
  pos[0]=Serial.read();  //X
  while(!Serial.available())
    delay(1);
  pos[1]=Serial.read();  //Y
  while(!Serial.available())
    delay(1);
  aux=Serial.read();  //Z
  if(aux==1. || aux==0.)  //En realidad z es booleano, esta linea es un filtro de ruido
    pos[2]=aux;
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


void setup() {
  // put your setup code here, to run once:
  servo1.attach(SERVO1);
  servo2.attach(SERVO2);
  servo3.attach(SERVO3);
  servo4.attach(SERVO4);
  servo5.attach(SERVO5);
  servo_garra.attach(SERVO_GARRA);
  Serial.begin(9600);
}

void loop() {
  int a;
  float cero[]={0.0128,0.1155,0.4044};
  float uno[]={0.0521,-0.1039,0.4044};
  MCI(cero);
  /*q[0]=135;
  q[1]=110;
  q[2]=70;
  q[3]=120;*/
   for(a=0;a<4;a++)
  {
    Serial.print("q[");
    Serial.print(a);
    Serial.print("]=");
    Serial.println(q[a]);
  }
  mueve_robot(q);
  delay(5000);
  MCI(uno);
     for(a=0;a<4;a++)
  {
    Serial.print("q[");
    Serial.print(a);
    Serial.print("]=");
    Serial.println(q[a]);
  }
  mueve_robot(q);
  delay(5000);
  /*delay(3000);
  //MCI(POS2);
  for(a=0;a<4;a++)
  {
    Serial.print("q[");
    Serial.print(a);
    Serial.print("]=");
    Serial.println(q[a]);
  }
  mueve_robot(q);
  delay(3000);*/
}
