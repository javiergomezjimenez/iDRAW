#include <Servo.h> //Librería para controlar los servos
#include <SoftwareSerial.h> //Hay que añadirla a la carpeta correspondiente
SoftwareSerial BT(10,11);

//Defines para cambiar los pines de los servos facilmente
#define SERVO1 5
#define SERVO2 6
#define SERVO3 3
#define SERVO4 9
#define SERVO_GARRA 2

//Defines de los puertos del puerto serie
//Hay que comprobar si estan al reves

#define FILAS 10

// Definición de las barras de nuestro manipulador
#define L0 0.11
#define L1 0
#define L2 0.105
#define L3 0.133
#define L4 0.043


#define OFFSET1 85
#define OFFSET2 165
#define OFFSET3 110
#define OFFSET4 120

#define pi PI
#define GARRA 0.1
#define t_grado 10  //Tiempo que tarda un servo en moverse un grado (ms)

//Declaramos los servos
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo_garra;

//Vectores de posiciones
String pos[FILAS][3]={"1040.0","0","0.11"};
float POS[FILAS][3];
//float POS[10] = {0.38099999, 0, 0.11}; //POSICIÓN ROBOT RECTO
float q[FILAS][3];

//Bandera para comunicacion con BT
bool flag = true;

//Estado de la FSM del main
int estado = 2;
  //int w=0, j=0;

//Para calcular el tiempo de movimiento
float q_ant[FILAS][3];


void mueve_robot(float q[][3])
{
  int a;
  int i, j, l; //Contador para el bucle
  float q_max = 0.; //Maximo angulo girado para llegar al punto
  unsigned int tiempo;  //Tiempo estimado en el que se alcanza el punto
  //Restringimos y mapeamos
   //Serial.println("GRADOS A CADA SERVO DESPUÉS DE RESTRINGIR");
  for(a=0;a<FILAS;a++)
  {
  for (i = 0; i < 4; i++)
  {
    q[a][i] = constrain(q[a][i], 0, 180);
    q_max = max(abs(q[a][i] - q_ant[a][i]), q_max); //Se coge el angulo maximo que se va a girar para ese punto

  }
  //Serial.println("-------------------------------------------------------");

  for (j = 0; j < 4; j++)
  {
    q_ant[a][j] = q[a][j]; //Guardamos el valor anterior para calcular el angulo recorrido
    //q[i] = map(q[i], 0, 180, 0, 255);
    //Serial.println(q[j]);
  }
  tiempo = q_max * t_grado * 1.1;
  Serial.println(" ");
  Serial.println(" ");
  Serial.println(" ");
  Serial.println(" ");
  Serial.println(" ");
  Serial.println(" ");
  Serial.println(" ");
  Serial.println(" ");
  
  //Movemos los servo
  servo1.write(q[a][0]);
  servo2.write(q[a][1]); 
  servo3.write(q[a][2]);
  servo4.write(q[a][3]);

  delay(tiempo);
  }
}

void convierte_grados(float q[][3])  //Funcion que transforma las variables articulares de radianes a grados, y ademas añade el offset
{
  int i,a,b;
  int offset[] = {OFFSET1, OFFSET2, OFFSET3, OFFSET4};
  for(a=0;a<FILAS;a++)
  {
  for (i = 0; i < 4; i++)
  {
    q[a][i] = (q[a][i] * 180.0) / pi;
    if(i==1 || i==3)                      //El servo aumenta en la otra dirección
      q[a][i] = -q[a][i]+offset[i];
    else
      q[a][i] += offset[i];
  }
  for(b=0;b<4;b++)
    {
      Serial.print("q[");
      Serial.print(b);
      Serial.print("]=");
      Serial.println(q[a][b]);
    }
  /*Serial.println("GRADOS ARTICULACIONES");
  for (int i = 0; i < 4; i++)
      {
        Serial.println(q[i]);
      }*/
  }
}

void MCI (float POS[][3])  // Función que emula el modelo cinemático inverso del manipulador
{
  int a;
  for(a=0;a<FILAS;a++)
  {
    
    float modu, u[2], auxx, auxy, x, x1, y, y1, z, d, r, alpha, beta, gamma, q1, q2, q3, q4, q5; // Definición de las variables necesarias
    x1 = POS[a][0]; y1 = POS[a][1]; z = POS[a][2] - L0;          // Archivo el vector de inicio en las variables correspondientes

    modu = fabs(sqrt(pow(x1, 2) + pow(y1, 2)));
    u[0] = x1 / modu; u[1] = y1 / modu;
    auxx = (modu - GARRA - L4) * u[0];
    auxy = (modu - GARRA - L4) * u[1];
    u[0] = auxx; u[1] = auxy;
    x = u[0]; y = u[1]; // Nos situamos en la conexión de la garra con el brazo

    r = sqrt(pow(x, 2) + pow(y, 2));                     // Cálculo  de la distancia reflejada en el plano XY
    d = sqrt(pow(r, 2) + pow(z, 2));            // Cálculo la distancia en línea recta desde la base hasta el extremo del posicionador

    alpha = atan(z / fabs(r));     // Ángulo auxiliar alpha
    beta =  atan(sqrt(1 - pow((pow(d, 2) + pow(L2, 2) - pow(L3, 2)) / (2 * d * L2) , 2)) / ((pow(d, 2) + pow(L2, 2) - pow(L3, 2)) / (2 * d * L2 )));
    //gamma = atan(sqrt(1 - pow((pow(L3, 2) + pow(L2, 2) - pow(d, 2 )) / (2 * L2 * L3), 2)) / ((pow(L3, 2) + pow(L2, 2) - pow(d, 2 )) / (2 * L2 * L3)));
    gamma = acos((pow(L3,2)+pow(L2,2)-pow(d,2))/(2*L2*L3));
    q2 = alpha + beta;                  // Articulación 2
    q3 = gamma-pi;                         // Articulación 3
    q1 = atan(y / x); // Articulación 1, que depende de las anteriores
    q4 = -q2 - q3; // Articulación 4 que controla la rotación respecto al eje perpendicular a la garra en la sujeción (mantenemos en horizontal)
    q[a][0] = q1; q[a][1] = q2; q[a][2] = q3; q[a][3] = q4;    // Exportamos las articulaciones en un vector q[], declarado globalmente

   }
   convierte_grados(q);
 }

void recibe_BT()  //Hay que convertir de string a int
{
  int a;
  int b=0;
  BT.begin(9600);
  while(b<FILAS)
  {
  BT.write("T");
  delay(100);
  if(BT.available()){
    for(a=0;a<3;a++){
      pos[b][a]=BT.readStringUntil('\n');
      Serial.println(pos[b][a]);
    }
    Serial.println("----------------------------------");
    b++;
     //estado=4;
    
  }
  }
  estado=4;
  BT.end();
  Serial.println("LO QUE ME LLEGA DESPUÉS DE MAPEAR");
  for(b=0;b<FILAS;b++)
  {
  for (a = 0; a < 3; a++){ //Pasamos el string a int       
        POS[b][a] = pos[b][a].toFloat();
        POS[b][0]=map(POS[b][0],30,1050,2240,3400)/10000.0;
        Serial.println(POS[b][0]);
        POS[b][1]=map(POS[b][1],-710,710,-1485,1485)/10000.0;
        Serial.println(POS[b][1]);
        POS[b][2]=0.1;
        Serial.println(POS[b][2]);
   }

 // Serial.println(pos[2]);
}
}

void abre_garra()
{
  int valor = 60;
  //valor = map(valor, 0, 180, 0, 255);
  servo_garra.write(valor);
}

void cierra_garra()
{
  int valor = 120;
  //valor = map(valor, 0, 180, 0, 255);
  servo_garra.write(valor);
}

/*SoftwareSerial BT(rx, tx);
void envia_BT(bool flag) {
  if (flag)
    BT.println("T");
  else
    BT.println("F");
  delay(10);  //Hay que comprobar si es suficiente
}*/


void setup() {
  // put your setup code here, to run once:
  servo1.attach(SERVO1);
  servo2.attach(SERVO2);
  servo3.attach(SERVO3);
  servo4.attach(SERVO4);
  servo_garra.attach(SERVO_GARRA);
  Serial.begin(9600);

  //BT.begin(9600);
}
void loop() {
  
  switch(estado)
{
  /*case 0: //Primera ejecucion
    envia_BT(flag);
    if(Serial.available())
      estado=2;
    break;
  case 1: //Reposo
    if(Serial.available())
      estado++;
    break;*/
  case 2: //Recibimos los datos desde la app
    recibe_BT();
    flag=false;
    break;
  /*case 3: //Envio de estado hacia la app
    envia_BT(flag);
    if(flag==false)
      estado++;
    else
      estado=1;
    break;*/
  case 4: //Cinematica inversa del brazo
  Serial.println("ESTADO 4");
    /*switch(w)
    {
      case 0:
        for(j=0; j<3 ;j++)
        {
          float pw[] = {0.224, -0.1485, 0.11};
          POS[j]=pw[j]; 
        }
        w++;
        break;
        case 1: 
        for(j=0; j<3 ;j++)
        {
          float pw[] = {0.224, 0.1485, 0.11};
          POS[j]=pw[j]; 
        }
        w++;
        break;
        case 2 :
        for(j=0; j<3 ;j++)
        {
          float pw[] = {0.34, 0.1485, 0.11};
          POS[j]=pw[j]; 
        }
        w++;
        break;
        case 3: 
        for(j=0; j<3 ;j++)
        {
          float pw[] = {0.34, -0.1485, 0.11};
          POS[j]=pw[j]; 
        }
        w++;
        break;
    }*/        
    MCI(POS);
    estado++;
    break;
  case 5: //Transformamos las coordenadas cartesianas en movimiento para las articulaciones
    cierra_garra();
    mueve_robot(q);
    flag=true;
    estado=2;
    break;
    }
    
//Serial.println(estado);
//    servo1.write(OFFSET1);
//    servo2.write(OFFSET2);
//    servo3.write(OFFSET3);
//    servo4.write(OFFSET4);
}
 /*void loop() {
 abre_garra();
 switch (estado)
  {
    case 2:
      MCI(POS);
      estado++;
      break;
    case 3: //Transformamos las coordenadas cartesianas en movimiento para las articulaciones
      mueve_robot(q);
      for (int i = 0; i < 4; i++)
      {
        Serial.println(q[i]);
      }
      estado++;
      break;
    case 4:
      break;
  }
}*/
