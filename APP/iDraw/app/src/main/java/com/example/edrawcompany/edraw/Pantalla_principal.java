package com.example.edrawcompany.edraw;

import android.animation.Animator;
import android.animation.AnimatorListenerAdapter;
import android.animation.AnimatorSet;
import android.animation.ObjectAnimator;
import android.content.Intent;
import android.os.Bundle;
import android.support.v7.app.AppCompatActivity;
import android.view.View;
import android.widget.Button;
import android.widget.ImageView;

public class Pantalla_principal  extends AppCompatActivity { //Clase principal.
    private ImageView robot;            //Creacion del objeto correspondiente a la imagen que se prentende mostrar.
    private Button Siguiente;
    private ObjectAnimator rotationrobot;           //Creacion del objeto correspondiente a la animación.

    private long animationduration = 2000;          //Duracion de la animación.
    @Override
    protected void onCreate(Bundle savedInstanceState) { //Clase donde se va a inicializar la actividad y donde se va a setear la animacion.


        super.onCreate(savedInstanceState);
            setContentView(R.layout.pantalla_principal);        //Setea el layout.
            robot = findViewById(R.id.id_Robot);            //Se enlazan las vistas.
            Siguiente = findViewById(R.id.Id_Siguiente);
            rotationrobot = ObjectAnimator.ofFloat(robot, "rotation", 0f, 360f);        //Se introducen los parametros de la animacion requerida mediante método ofFloat
            rotationrobot.setDuration(animationduration);       //Se setea la duración de la animacion.
            AnimatorSet animatorsetrotation = new AnimatorSet();        //Se crea el objeto correspondiente al seteo de la animación.
            animatorsetrotation.playTogether(rotationrobot);        //Se lanza la animación.
            animatorsetrotation.addListener(new AnimatorListenerAdapter() {         //Se ejecuta la animación de rotación creada en bucle infinito.
                                                private boolean cancel=true;
                                                @Override
                                                public void onAnimationStart(Animator animation) {
                                                    cancel=true;
                                                }

                                                @Override
                                                public void onAnimationEnd(Animator animation) {
                                                    animation.start();

                                                }

                                                @Override
                                                public void onAnimationCancel(Animator animation) {
                                                    cancel=false;
                                                }
                                            });
            animatorsetrotation.start();




            Siguiente.setOnClickListener(new View.OnClickListener() {
                public void onClick(View v) {       //Se lanza la siguiente actividad al pulsar el botón START.
                    Intent intent = new Intent(getApplicationContext(), Bluetooth.class);
                    startActivity(intent);
                }
            });



    }
}