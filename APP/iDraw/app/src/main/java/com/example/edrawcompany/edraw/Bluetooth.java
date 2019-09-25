package com.example.edrawcompany.edraw;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.content.pm.ActivityInfo;
import android.os.Bundle;
import android.support.v7.app.AppCompatActivity;
import android.util.Log;
import android.view.View;
import android.widget.ArrayAdapter;
import android.widget.ListView;
import android.widget.TextView;
import android.widget.Toast;
import android.widget.AdapterView;
import android.content.Intent;
import java.util.Set;
import java.lang.String;


public class Bluetooth extends AppCompatActivity {
    private int REQUEST_ENABLE_BT = 1;
    private String Found_device_address;
    ListView List_disp_b;
    TextView disp_b;
    final BluetoothAdapter MyDispBluetooth = BluetoothAdapter.getDefaultAdapter();              //obtener el adaptador local de bluetooth

    @Override
    protected void onCreate(Bundle savedInstanceState) {//Creacion de la actividad bluetooth

        super.onCreate(savedInstanceState);
        setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_PORTRAIT);
        setContentView(R.layout.dispositivos_bluetooth); //set del layout dispositivos bluetooth
        if (MyDispBluetooth == null) {                  //si el dispositivo no soporta bluetooth
            Toast toast1 = Toast.makeText(getApplicationContext(), "El dispositivo no soporta Bluetooth", Toast.LENGTH_LONG);
            toast1.show();
            System.exit(0);
        }
        if (!MyDispBluetooth.isEnabled()) {                         //si el bluetooth no esta activado
            Intent enable_b = new Intent(MyDispBluetooth.ACTION_REQUEST_ENABLE);            //se crea un intent y se inicia la actividad de peticion de activacion
            startActivityForResult(enable_b, REQUEST_ENABLE_BT);                            //de bluetooth al usuario
        }
    }
    @Override
    protected void onActivityResult(int requestCode,int resultCode,Intent data){
        if(requestCode==REQUEST_ENABLE_BT){
            if(resultCode==RESULT_CANCELED){
                finish();
            }
        }
    }

    @Override
    protected void onResume() {                 //codigo que se va a ejecutar cada vez que se vuelva a la aplicacion
        super.onResume();
        disp_b = (TextView) findViewById(R.id.bluetooth_Title);     //asignar al objeto textview el id bluetoot_title
        final ArrayAdapter<String> array_pdevices = new ArrayAdapter<String>(this, android.R.layout.simple_list_item_1);    //creacion de un array adapter
        final Set<BluetoothDevice> disp_sinc = MyDispBluetooth.getBondedDevices();          //se obtienen los dispositivos emparejados
        if (disp_sinc.size() > 0) {
            Log.d("onResume","rellenando array");
            for (BluetoothDevice device : disp_sinc) {
                array_pdevices.add(device.getName() + "\n" + device.getAddress());          //se rellena el array adapter con los dispositivos
            }                                                                               //emparejados encontrados
            List_disp_b = (ListView) findViewById(R.id.List_disp_b);
            List_disp_b.setAdapter(array_pdevices);                                         //se muestran los dispositivos emparejados al usuario
            List_disp_b.setOnItemClickListener(new AdapterView.OnItemClickListener() {
                @Override
                public void onItemClick(AdapterView<?> parent, View view, int position, long id) {            //accion que se va a llevar a cabo al seleccionar un item
                    Found_device_address = array_pdevices.getItem(position).substring(array_pdevices.getItem(position).length() - 17);  //direccion MAC del dispositivo con el que se va a conectar
                    Intent launch_main = new Intent(getApplicationContext(), main_Activity.class);  //se inicia la actividad main_Activity
                    launch_main.putExtra("device_address_found", Found_device_address);      //se pasa como parametro a la siguiente actividad la direccion anterior
                    startActivity(launch_main);
                }

            });
        }
    }
}
