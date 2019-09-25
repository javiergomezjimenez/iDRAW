/*///////////////////////////////////////////////////////////////////////////////////
                                   INCLUDES
///////////////////////////////////////////////////////////////////////////////////*/

#include <Servo.h> //Librería para controlar los servos
#include <SoftwareSerial.h> //Hay que añadirla a la carpeta correspondiente
//SoftwareSerial BT(10,11);

/*///////////////////////////////////////////////////////////////////////////////////
                                   DEFINES
///////////////////////////////////////////////////////////////////////////////////*/

//Pines de cada servo
#define SERVO1 5
#define SERVO2 6
#define SERVO3 3
#define SERVO4 10
#define SERVO_GARRA 9

//Longitudes para el modelo del robot
#define L0 0.11
#define L1 0
#define L2 0.105
#define L3 0.133
#define L4 0.043

//Offset con respecto a los ejes dextrogiros para el modelo del robot
#define OFFSET1 85
#define OFFSET2 165
#define OFFSET3 110
#define OFFSET4 120
#define GARRA 0.1

//Para comodidad de programación
#define pi PI

//Para optimizacion del movimiento del robot 
#define t_grado 10  //Tiempo que tarda un servo en moverse un grado (ms)


/*///////////////////////////////////////////////////////////////////////////////////
                            DECLARACION DE OBJETOS
///////////////////////////////////////////////////////////////////////////////////*/
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo_garra;


/*///////////////////////////////////////////////////////////////////////////////////
                            DECLARACION DE VARIABLES GLOBALES
///////////////////////////////////////////////////////////////////////////////////*/

//Vectores de posiciones
String pos[10]={"1040.0","0","0.11"}; //Posicion en string, recien recibida de BT, sin conversiones. Inicializada para evitar que se empiece fuera de rango
float POS_ant[10];  //Variable para calcular la articulacion que tardara mas en llegar a su referencia y optimizar el tiempo real
float POS[10]={0.34,0,0.11};  //Posicion recibida ya convertida, inicializada para evitar que se empiece fuera de rango
//float POS[10] = {0.38099999, 0, 0.11}; //POSICIÓN ROBOT RECTO, HOME
float q[10];  //Vector de posiciones articulares para cada servomotor

//Bandera para comunicacion con BT
bool flag = true; //Define cuando el robot esta listo para recibir una nueva posicion de la app

//Estado de la FSM del main
int estado = 1;

//Para calcular el tiempo de movimiento
float q_ant[10];  //Angulos anteriores de cada servo

bool inicio=true; //Para secuencia de apertura y agarre del rotulador



/*///////////////////////////////////////////////////////////////////////////////////
                            FUNCIONES 
///////////////////////////////////////////////////////////////////////////////////*/

//mueve_robot-->Una vez calculado el angulo que debe girar cada servo, actuamos sobre los motores
void mueve_robot(float q[])
{
  int i, j, l; //Contador para el bucle
  float q_max = 0.; //Maximo angulo girado para llegar al punto
  unsigned int tiempo;  //Tiempo estimado en el que se alcanza el punto
  //Restringimos y mapeamos
  for (i = 0; i < 4; i++)
  {
    q[i] = constrain(q[i], 0, 180);
    q_max = max(abs(q[i] - q_ant[i]), q_max); //Se coge el angulo maximo que se va a girar para dibujar ese punto
  }
  for (j = 0; j < 4; j++)
  {
    q_ant[j] = q[j]; //Guardamos el valor anterior para calcular el angulo recorrido en el siguiente punto
  }
  tiempo = q_max * t_grado * 1.1; //El tiempo que se tardara en dibujar a un punto, ie, en llegar a una posicion será el tiempo mas grande de cada articulacion, con un margen de seguridad
  
  //Actuamos sobre los servomotores
  servo1.write(q[0]);
  servo2.write(q[1]); 
  servo3.write(q[2]);
  servo4.write(q[3]);

  delay(tiempo);  //Damos tiempo para que se ejecute el movimiento con el delay que se ha calculado previamente. Esta optimizacion logra que el funcionamiento del robot sea en tiempo real.
}

//convierte grados-->Convierte cada angulo de giro de servo de radianes (Cinematica inversa) a grados (funcion de giro de servos), ademas de añadirle el offset de cada eje
void convierte_grados(float q[])
{
  int i;
  int offset[] = {OFFSET1, OFFSET2, OFFSET3, OFFSET4};  //Para optimizacion de codigo metemos los offsets en un vector
  for (i = 0; i < 4; i++)
  {
    q[i] = (q[i] * 180.0) / pi; //Conversion de radianes a grados
    //Añadimos el offset
    if(i==1 || i==3)            //Los angulos q2 y q4, debido al planteamiento del modelo del robot, aumentan en el otro sentido
      q[i] = -q[i]+offset[i];
    else
      q[i] += offset[i];
  }
}

//MCI-->Modelo cinematico inverso del robot, dada una posicion a la que se desea llevar el efector del robot, calcula el angulo que debe girar cada servomotor
void MCI (float POS[])
{
  float modu, u[2], auxx, auxy, x, x1, y, y1, z, d, r, alpha, beta, gamma, q1, q2, q3, q4, q5; // Definición de las variables necesarias
  x1 = POS[0]; y1 = POS[1]; z = POS[2] - L0;    // Archivo el vector de inicio en las variables correspondientes

  modu = fabs(sqrt(pow(x1, 2) + pow(y1, 2)));
  u[0] = x1 / modu; u[1] = y1 / modu;
  auxx = (modu - GARRA - L4) * u[0];
  auxy = (modu - GARRA - L4) * u[1];
  u[0] = auxx; u[1] = auxy;
  x = u[0]; y = u[1];                          // Nos situamos en la conexión de la garra con el brazo

  r = sqrt(pow(x, 2) + pow(y, 2));            // Cálculo  de la distancia reflejada en el plano XY
  d = sqrt(pow(r, 2) + pow(z, 2));            // Cálculo la distancia en línea recta desde la base hasta el extremo del posicionador

  alpha = atan(z / fabs(r));     // Ángulo auxiliar alpha
  beta =  atan(sqrt(1 - pow((pow(d, 2) + pow(L2, 2) - pow(L3, 2)) / (2 * d * L2) , 2)) / ((pow(d, 2) + pow(L2, 2) - pow(L3, 2)) / (2 * d * L2 )));
  gamma = acos((pow(L3,2)+pow(L2,2)-pow(d,2))/(2*L2*L3));
  q2 = alpha + beta;                  // Articulación 2
  q3 = gamma-pi;                         // Articulación 3
  q1 = atan(y / x); // Articulación 1, que depende de las anteriores
  q4 = -q2 - q3; // Articulación 4 que controla la rotación respecto al eje perpendicular a la garra en la sujeción (mantenemos en horizontal)
  q[0] = q1; q[1] = q2; q[2] = q3; q[3] = q4;    // Exportamos las articulaciones en un vector q[], declarado globalmente
  //Convertimos al formato que entienden los servos, offsets incluidos
  convierte_grados(q);

}

//recibe_BT-->Funcion que recibe las coordenadas a pintar desde la app de Android
void recibe_BT()
{
  int a;  //Variable para bucles
  //NOTA: Arduino maneja internamente la creación de PWMs a través del puerto serie, por ello es necesario cerrar la comunicacion BT con la app siempre que no sea necesaria.
  Serial.begin(9600); //Abrimos la comunicacion serie por BT
  Serial.println("T");  //Enviamos un enable para indicar que el robot esta listo para recibir
  delay(50);  //Tiempo de apertura de comunicacion y envio del enable, en ms.
  if(Serial.available()){ //Al recibir un dato
    for(a=0;a<2;a++){ //Se recibe cada vez un punto completo, con sus coordenadas x e y.
      pos[a]=Serial.readStringUntil('\n');  //Por protocolo interno de comunicacion, \n define el fin de una coordenada y el inicio de la siguiente
     estado=2;  //Para la FSM
      }
    Serial.end(); //Cerramos la comunicacion serie para no interferir
  }
  for (a = 0; a < 2; a++){ //Pasamos el string a float    
        POS[a] = pos[a].toFloat();
   }
   //Convertimos las posiciones referidas al plano de dibujo de la app a las coordenadas fisicas del folio
   POS[0]=map(POS[0],30,1050,2240,3400)/10000.0;  
   POS[1]=map(POS[1],-710,710,-1485,1485)/10000.0;
   POS[2]=0.11; //Eje z, en realidad es booleano, ie, o levantado o apoyado.
}

//abre_garra-->Comando que abre automaticamente la garra del robot
void abre_garra()
{
  int valor = 60; //Posicion de garra abierta, sin chocar con nada
  servo_garra.write(valor);
}

//cierra_garra-->Comando que cierra automaticamente la garra del robot
void cierra_garra()
{
  int valor = 120;  //Posicion de garra cerrada, sin ejercer demasiada fuerza sobre el soporte del rotulador
  servo_garra.write(valor);
}

/*///////////////////////////////////////////////////////////////////////////////////
                                SETUP
///////////////////////////////////////////////////////////////////////////////////*/

void setup() {
  // put your setup code here, to run once:
  servo1.attach(SERVO1);
  servo2.attach(SERVO2);
  servo3.attach(SERVO3);
  servo4.attach(SERVO4);
  servo_garra.attach(SERVO_GARRA);
  Serial.begin(9600);

/*///////////////////////////////////////////////////////////////////////////////////
                                MAIN
///////////////////////////////////////////////////////////////////////////////////*/
}
void loop() {
  //Secuencia inicial de apertura y agarre de rotulador
  if(inicio)
    {
      abre_garra();
      delay(5000);
      cierra_garra();
      delay(5000);
      inicio=false;
    }
    //Tras la ejecucion inicial de la rutina anterior, la garra permanecera cerrada constantemente
  cierra_garra();
  //FSM
  switch(estado)
{
  case 1: //Recibimos los datos desde la app
    recibe_BT();
    flag=false;
    break;

  case 2: //Cinematica inversa del brazo   
    MCI(POS);
    estado++;
    break;
  case 3: //Transformamos las coordenadas cartesianas en movimiento para las articulaciones
    mueve_robot(q);
    flag=true;
    estado=1;
    break;
    }
}
