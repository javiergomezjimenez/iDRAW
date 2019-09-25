package com.example.edrawcompany.edraw;
import android.bluetooth.BluetoothAdapter;
import android.content.Context;
import android.content.pm.ActivityInfo;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Path;
import android.os.Bundle;
import android.support.v7.app.AppCompatActivity;


import android.util.Log;
import android.view.MotionEvent;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;
import android.os.Handler;
import java.io.IOException;
import java.util.UUID;
import java.io.InputStream;
import java.io.OutputStream;
import android.content.Intent;
import android.widget.Toast;

public class main_Activity extends AppCompatActivity {
    private Handler myHandler;
    private String v[][]=new String[2000][3];
    private int k0=0;
    private int j=0;
    private Hilo_Conexion myHiloConexion;
    private Hilo_cliente myHiloCliente;
    private BluetoothAdapter myBluetooth=BluetoothAdapter.getDefaultAdapter();


    @Override
    protected void onCreate(Bundle savedInstanceState) {        //Clase donde se ejecuta la vista generada por la clase Canvas y donde se inicializa la actividad.
        super.onCreate(savedInstanceState);
        myHiloCliente=new Hilo_cliente(myBluetooth.getRemoteDevice(getIntent().getExtras().getString("device_address_found")));  //Se inicializa el objeto myHiloCliente del tipo Hilo_Cliente con
        myHiloCliente.start();   //Se ejecuta el hilo secundario correspondiente a myHiloCliente
        Vista vista=new Vista(this);
        setContentView(vista);          //Seteo de la vista generada por la clase Canvas
        setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_PORTRAIT);      //Bloqueo del giro de pantalla.
    }
    class Vista extends View {
        float x_nueva=50;           //Variables de almacenamiento de coordenadas. Se almacena la coordenada anterior para comparar.
        float x_ant=50;
        float y_nueva=50;
        float y_ant=50;
        float z_nueva,z_ant;
        int e = 2;                  //Valor de precisión asignado
        String accion="accion";
        Path path=new Path();

        public Vista(Context context){

            super(context);
        }
        public void onDraw(Canvas canvas){ //Inicialización de las herramientas de dibujo
            Paint paint = new Paint();
            paint.setStyle(Paint.Style.STROKE);
            paint.setStrokeWidth(5);
            paint.setColor(Color.BLUE);
            Paint pincel = new Paint();
            pincel.setStyle(Paint.Style.STROKE);
            pincel.setTextSize(80);
            pincel.setStrokeWidth(5);
            pincel.setColor(Color.RED);
            String coordenadax = Float.toString(x_nueva);
            String coordenaday = Float.toString(740-y_nueva);     //Reasignación del valor de la coordenada y para adaptación a los ejes del robot
            String coordenadaz = Float.toString(z_nueva);

            paint.setColor(Color.BLACK);
            canvas.drawRect(30,30, 1050,1450,paint);            // Espacio dibujable
            canvas.drawRect(100,1500, 300,1700,paint);          // Borrar
            if (x_nueva > 100 && x_nueva < 300){                                        // Borrar lo pintado en pantalla
                if(y_nueva> 1500 && y_nueva< 1700){
                    path.reset();
                }
            }

            paint.setColor(Color.BLUE);

            //Tras comprobar que ha variado alguna de las tres coordenadas, se almacenan estas en la matriz de coordenadas.
           if ((!(x_ant-e<=x_nueva && x_nueva<=x_ant+e)  || !(y_ant-e<=y_nueva && y_nueva<=y_ant+e) || z_nueva!=z_ant) && j<v.length ){
                x_ant=x_nueva;
                y_ant=y_nueva;
                z_ant = z_nueva;
                v[j][0]=coordenadax+"\r\n";
                v[j][1]=coordenaday+"\r\n";
                v[j][2]=coordenadaz+"\r\n";
                Log.d("Mytaag",Float.toString(j)+"  "+ v[j][0]+"  " + v[j][1]+"  "+v[j][2]);
                j++;
                };

           canvas.drawPath(path,paint);

        }

        //Al detectar un evento de pantalla
        public boolean onTouchEvent(MotionEvent e) {
            //Si las coordenadas captadas se encuentran en el espacio de dibujo definido, se almacenan
            if ((30 <= e.getX() && e.getX() <= 1050 && 30 <= e.getY() && e.getY() <= 1450) || (100 <= e.getX() && e.getX() <= 300 && 1500 <= e.getY() && e.getY() <= 1700)) {
                x_nueva = e.getX();
                y_nueva = e.getY();
            }

            //En función del tipo de acción captada, se construye y dibuja la trayectoria deseada
            if (e.getAction() == MotionEvent.ACTION_DOWN)
                if ((30 <= e.getX() && e.getX() <= 1050 && 30 <= e.getY() && e.getY() <= 1450) || (100 <= e.getX() && e.getX() <= 300 && 1500 <= e.getY() && e.getY() <= 1700)){
                    accion = "down";
                    path.moveTo(x_nueva, y_nueva);
                }

            if(e.getAction()==MotionEvent.ACTION_MOVE ) {
                accion = "move";
                path.lineTo(x_nueva, y_nueva);
                z_nueva = (float) 1.0;
            }

            if(e.getAction()==MotionEvent.ACTION_UP) {
                accion = "up";
                z_nueva = (float) 0.0;
            }
            invalidate();
            return true;

        }
    }
    public class Hilo_cliente extends Thread {              //Hereda de Thread debido a que se desea ejecutar como un hilo secundario
        private final BluetoothSocket myClientSocket;       //para no colapsar el hilo principal
        private final UUID MY_UUID = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB");       //Identificador de conexion UUID
        public Hilo_cliente(BluetoothDevice device) {       //Constructor al que se le pasa como parámetro el objeto device correspondiente al dispositivo
            BluetoothSocket tmp = null;                     //con el que se pretende establecer la conexión
            try {
                tmp = device.createRfcommSocketToServiceRecord(MY_UUID);    //Creación del socket de conexión
            } catch (IOException e) { //En caso de excepción se publica el error.
                runOnUiThread(new Runnable() {      //este codigo se va a ejecutar en el hilo principal con este metodo (runOnUiThread)
                                  @Override
                                  public void run() {
                                      Toast.makeText(getBaseContext(), "Error en la creacion del socket de cliente", Toast.LENGTH_LONG).show();
                                  }
                              });
                finish();
            }
            myClientSocket = tmp;
        }

        public void run() {                             //Código que se va a ejecutar en el hilo secundario
            try {
                myClientSocket.connect();           //Conexión con el dispositivo
            } catch (IOException connectException) {
                try {
                    runOnUiThread(new Runnable() {      //este codigo se va a ejecutar en el hilo principal con este metodo
                        @Override
                        public void run() {
                            Toast.makeText(getBaseContext(),"Error en la creacion del socket de conexion",Toast.LENGTH_LONG).show();
                        }
                    });
                    myClientSocket.close();                 //En caso de que no sea posible la conexión se cierra el socket, se mata la actividad y se publica el error
                    Log.d("Hilo_cliente","error no conectado con servidor");

                } catch (IOException closeException) {      //En caso de que se produzca la excepción se mata la actividad y se publica el error
                    runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            Toast.makeText(getBaseContext(),"Error cerrando el socket",Toast.LENGTH_LONG).show();
                        }
                    });
                    finish();
                }

            }
            if(myClientSocket!=null) {
                myHiloConexion = new Hilo_Conexion(myClientSocket);   //Una vez se consiga establecer la conexión, se inicia el hilo secundario de conexión
                myHiloConexion.start();
            }
        }
        public void cancel() {                  //Método para cancelar la conexión cerrando el socket
            try {
                myClientSocket.close();
            } catch (IOException e) {
                runOnUiThread(new Runnable() { //En caso de excepción se publica el error.
                    @Override
                    public void run() {
                        Toast.makeText(getBaseContext(), "Error cerrando el socket", Toast.LENGTH_LONG).show();
                    }
                });
            }
        }
    }

    public class Hilo_Conexion extends Thread {
        private final BluetoothSocket mySocket;
        private final InputStream Data_input;
        private final OutputStream Data_output;

        public Hilo_Conexion(BluetoothSocket socket) {              //Hilo secundario cuya función es la de establecer los
            Log.d("Hilo_conexion","Lanzando");
            mySocket = socket;                                      // flujos de datos de entrada y salida.
            InputStream tmpIn = null;
            OutputStream tmpOut = null;
            try {
                tmpIn = mySocket.getInputStream();                  //Obtención de los flujos de salida y de entrada de datos
                tmpOut = mySocket.getOutputStream();
            } catch (IOException e) {
                runOnUiThread(new Runnable() {  //En caso de excepción se publica el fallo.
                    @Override
                    public void run() {
                        Toast.makeText(getBaseContext(), "Error obteniendo los flujos de datos", Toast.LENGTH_LONG).show();
                    }
                });
            }
            Data_input = tmpIn;
            Data_output = tmpOut;
        }

        public void run() {
            byte[] buffer = new byte[1024];
            int bytes;
            String readmessager=new String();
            while(true) {
                try {
                    bytes = Data_input.read(buffer);                        //Lectura de datos de entrada
                    readmessager = new String(buffer, 0, bytes);    //Se convierte a String los datos recibidos
                    myHandler.obtainMessage(1, bytes, -1, readmessager).sendToTarget();   //Se envían al hilo principal mediante un Handle

                } catch (IOException e) { }
                readmessager=readmessager.substring(0,1); //Se selecciona el primer caracter de la cadena recibida
                if ((j-k0)>=0&&readmessager.equals("T")) { //Condición de que el índice de envío no adelante al índice de guardado.
                    if (v[k0][0] != null & v[k0][1] != null & v[k0][2] != null) {   //Condición de que las componentes del vector estén inicializadas.
                        try {
                            Data_output.write(v[k0][0].getBytes());     //Envío de coordenadas en formato Bytes
                            Data_output.write(v[k0][1].getBytes());
                            Data_output.write(v[k0][2].getBytes());
                        } catch (IOException e) {
                            runOnUiThread(new Runnable() {      //En caso de que se produzca una excepción publica el error y mata la actividad.
                                              @Override
                                              public void run() {
                                                  Toast.makeText(getBaseContext(), "Error envio de datos", Toast.LENGTH_LONG).show();
                                              }
                                          });
                            finish();
                        }
                    }
                    readmessager="F"; //Se pone a false para que no vuelva a enviar hasta que sea autorizado por el microcontrolador.
                    k0++;     //Actualizacion del índice de envío
                }
            }


        }
        public void cancel() {      //Método encargado de cerrar el socket
            try {
                mySocket.close();
            } catch (IOException e) {
                runOnUiThread(new Runnable() {
                    @Override
                    public void run() {     //En caso de excepción se publica el error.
                        Toast.makeText(getBaseContext(), "Error cerrando el socket", Toast.LENGTH_LONG).show();
                    }
                });
            }
        }

    }
}
