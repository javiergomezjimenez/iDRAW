#include <Servo.h> //Librería para controlar los servos

//Defines para cambiar los pines de los servos facilmente
#define SERVO1 5
#define SERVO2 6
#define SERVO3 9
#define SERVO4 10
#define SERVO5 11 //¿QUÉ PUERTO ES?
#define SERVO_GARRA 3

// Definición de las barras de nuestro manipulador
#define L0 0.11
#define L1 0
#define L2 0.105
#define L3 0.133
#define L4 0.05
#define GARRA 0.127

//Offsets del brazo
#define OFFSET1 
#define OFFSET2 
#define OFFSET3 
#define OFFSET4 
#define OFFSET5

#define t_grado 7.1428  //Tiempo que tarda un servo en moverse un grado (ms)

//Declaramos los servos
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;
Servo servo_garra;

//Declarar las dos posiciones de prueba
 float POS1[] = {0.2,0.2,0.2};
 float POS2[] = {4,5,7};

//Vectores de posiciones
float pos[10];
float q[10];

//Bandera para comunicacion con BT
bool flag=true;

//Estado de la FSM del main
int estado=1;

//Para calcular el tiempo de movimiento
float q_ant[10];

char letra; //Para la prueba de la garra


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
  q3 = gamma + PI;                         // Articulación 3
  q1 = atan((y/(L3*cos(q2 + q3) + L2*cos(q2)))/(x/(L3*cos(q2 + q3) + L2*cos(q2)))); // Articulación 1, que depende de las anteriores
  q4 = PI/2 - q2 - q3; // Articulción 4 que controla la rotación respecto al eje perpendicular a la garra en la sujeción (mantenemos en horizontal)
  q5 = 0; //Articulación 5 (Mantanemos sin inclinar respecto al eje longitudinal de la garra)
  q[0] = q1; q[1] = q2; q[2] = q3; q[3] = q4; q[4] = q5;    // Exportamos las articulaciones en un vector q[], declarado globalmente
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
  int valor=45;
  valor=map(valor,0,180,0,255);
  servo_garra.write(valor);
}

void cierra_garra()
{
  int valor=60;
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
//
void loop() {
 /*q[0]=0;
 mueve_robot(q);*/

 if(Serial.available())
 {
  letra=Serial.read();
  Serial.print("Letra= ");
  Serial.println(letra);
  if(letra=='r'){
    q[0]--;
    mueve_robot(q);
  }
  if(letra=='s'){
    q[0]++;
    mueve_robot(q);
  }
  if(letra=='i')
  {
    q[0]=90;
    mueve_robot(q);
  }
  if(letra=='m'){
    Serial.print("Angulo: ");
    Serial.println(q[0]);
 }
 }
 /*abre_garra();
 delay(3000);
 cierra_garra();
 delay(3000);*/
}
