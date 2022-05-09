//LA LLAMA
#include <Servo.h>

#define QTR_MAX_SENSORS    16                              //numero maximo de sensores que se pueden usar
#define numSensores        7                               // number of sensors used
#define muestrasPorSensor  8                               //cantidad de muestras que toma el adc para cada sensor de linea
unsigned int valores_sensor[numSensores];                  //arreglo donde se van a guardar los valores de las lecturas del sensor de linea
unsigned int pines[numSensores]={0,2,3,4,5,6,7};           //pines con adc del teensy conectados a los sensores de linea
unsigned int valmax[numSensores];                          //arreglo donde se guardan los maximos producto del ajuste inicial
unsigned int valmin[numSensores];                          //arreglo donde se guardan los minimos producto del ajuste inicial
int linea, linea_ant=0;                                                 //variable donde se guarda la posicion del robot respecto a la linea
/////
#define moti                    6                       //Pin del motor izquierdo.
#define motd                    9                       // Pin del motor derecho.
#define moti_pwm                5//es el 7              //PWM moto izquierdo.
#define motd_pwm                10                      // PWM moto derecho.
#define boton_1                 22                      // pin del boton.
#define led                     13
/////////
//variables para el pid
int  derivativo=0, proporcional=0, integral=0; //parametros
int  proporcional_pasado=0;
float salida_pwm=0;
///////////////AQUI CAMBIEREMOS LOS PARAMETROS DE NUESTRO ROBOT!!!!!!!!!!!!!!!
int velocidad=255;//max 170  
int Vmax=450,Vmin=200;//750-500
float Kp=0.05, Kd=50; // Mejores valores Kp=0.06, Kd=50
//Turbina:
Servo turbina;  // create servo object to control a servo
int pos = 0; //Sets position variable

void setup() {
  analogWriteFrequency(motd_pwm, 1000);
  analogWriteFrequency(moti_pwm, 1000);
  Serial1.begin(38400);//38400                             //velocidad de transmision usada esta se configuro con el hc-05 Como nota, tratar de aumentar esta velocidad
  analogReadResolution(13);                                //13 bits de resolucion del adc para las lecturas de los sensores de linea entregan valores entre 0 y 8191
  analogWriteResolution(10);                              //13 bits resolucion timer 
  pinMode(moti,OUTPUT);
  pinMode(motd,OUTPUT);
  pinMode(boton_1, INPUT); //boton 1 como pull up
  //pinMode(moti_pwm,OUTPUT);
  //pinMode(motd_pwm,OUTPUT);
  
  //delay(4000);                                             //espera 3 segundos para que cuando se conecte el blueth.. al pc alcance a enviar mensaje de encendido
  Serial1.println("ENCENDIDO");                            //mensaje de confirmacion configuracion serial y blueth.. exitosa
  pinMode(led,OUTPUT);                                      //led del teensy
  calibrar(valmin,valmax);                                 //obtenemos el max y min de lecturas para ajustar los valores de los sensores a las condiciones de luz
  turbina.attach(4,1000,2000);
  turbina.write(0);
   
  int var1 = digitalRead(boton_1);
  while(var1==LOW){
    var1 = digitalRead(boton_1);}
  // Turbina
  digitalWrite(led, HIGH);
  turbina.write(60);
  delay(2000);
  digitalWrite(led, LOW);
}

void loop() {
 linea=leer_posicion(valmin,valmax);                      //llama la funcion leer posicion que entrega la posicion del robot respecto a la linea
 Serial1.println(linea);
  

 proporcional = (linea) - 3500; // 2500 es la referencia referencia
  //integral=integral + proporcional_pasado; //obteniendo integral
  //derivativo = (proporcional - proporcional_pasado); //obteniedo el derivativo
  salida_pwm =(  ( proporcional * Kp ) + ( (proporcional - proporcional_pasado) * Kd ) +salida_pwm);
  velocidad = (float)(abs(proporcional))*(Vmax-Vmin)/(-3500.0)+Vmax;

  if (  salida_pwm > velocidad )  salida_pwm = velocidad; //limitamos la salida de pwm
  if ( salida_pwm < -velocidad )  salida_pwm = -velocidad;
  proporcional_pasado = proporcional;  

/*
 if(salida_pwm < 0){
 analogWrite(moti_pwm,abs((velocidad-abs(salida_pwm))));
 analogWrite(motd_pwm,(velocidad));
 digitalWrite(moti,LOW);
 digitalWrite(motd,LOW);
 }
 else
 {
 analogWrite(moti_pwm,velocidad);
 analogWrite(motd_pwm,abs((velocidad-abs(salida_pwm))));
 digitalWrite(moti,LOW);
 digitalWrite(motd,LOW);
 }

  if(linea>6500){ //si se salio por la parte derecha de la linea
   digitalWrite(motd,HIGH);
    analogWrite(motd_pwm,1023-30);
    analogWrite(moti_pwm,(velocidad));
    digitalWrite(moti,LOW);
    
                    }                  
  if(linea<1500){ //si se salio por la parte izquierda de la linea
   digitalWrite(moti,HIGH);
   analogWrite(moti_pwm,1023-30);//ultimo valor bueno 80*238 90*23
   digitalWrite(motd,LOW);
   analogWrite(motd_pwm,velocidad);
                }*/
 }

int leer_posicion(unsigned int *mincalibracion, unsigned int *maxcalibracion)
{   int valor=0,media=0,suma=0,valorAjustado[numSensores],posicion_linea=0;
     leer(valores_sensor);                              //lee los sensores en bruto
     for(int i=0;i<numSensores;i++){
          //valorAjustado establece una ecuacion que ajusta los valores de los sensores segun las condiciones de luz(min y max sensados)
          valorAjustado[i]= abs(valores_sensor[i]*(8000.0/(float)(maxcalibracion[i]-mincalibracion[i])) -mincalibracion[i]*(8000.0/(float)(maxcalibracion[i]-mincalibracion[i])));
        if(valorAjustado[i]<200){valorAjustado[i]=0;} //establezco un humbral para eliminar                          
                                    }
    for (int i = 0; i < numSensores; i++)
    {
        Serial1.print(valorAjustado[i]);Serial1.print(' '); //imprime los valores ajustados que nos serviran para calcular el promedio ponderado
    }

    for(int i=0;i<numSensores;i++){                   //realiza un promedio ponderado que me entrega posicion entre 0 y 8000
          media +=valorAjustado[i]*((i+1)*1000);          
          suma += valorAjustado[i];
    }
    //Serial1.print(media);Serial1.print(' ');
    //Serial1.print(suma);Serial1.print(' ');
    //Serial1.print(linea_ant);Serial1.print(' ');
   if(suma==0 || media==0){                           //cuando se sale de los extremos marque 0 o 8000 (NO FUNCIONA SE DEBE SOLUCIONAR)
    
    if(linea_ant>3500){
      
      posicion_linea=numSensores*1000;
      return posicion_linea;}
    
    else{return 0;}
     }
    else{posicion_linea=media/suma;}
    //Serial1.println(posicion_linea);
     linea_ant=posicion_linea;
     return posicion_linea;
}

void calibrar(unsigned int *mincalibracion, unsigned int *maxcalibracion) //funcion para ajustar los valores de los sensores a las condiciones de luz
{ 
      digitalWrite(13,HIGH);                                        //prende led para indicar inicio de ajuste
      int i=0,j=0;
      int valmin[numSensores];                                      //arreglo para guardar lecturas minimas de cada sensor
      int valmax[numSensores];                                      //arreglo para guardar lecturas maximas de cada sensor    
      
      for(j=0;j<5500;j++){                                          //encuentra el maximo y minimo de datos sensados
          leer(valores_sensor);
          for(i=0;i<numSensores;i++){
            if(j==0 || valmax[i] < valores_sensor[i])
               valmax[i]=valores_sensor[i];
            if(j==0 || valmin[i] > valores_sensor[i])
               valmin[i]=valores_sensor[i];
                            }
                        }
       for(i=0;i<numSensores;i++){
        mincalibracion[i]=valmin[i];  
        maxcalibracion[i]=valmax[i];
                                  }
         Serial1.println("minimos");                                  //imprime valores para verificar correcto ajuste
    for (int i = 0; i < numSensores; i++)
    {
        Serial1.print(valmin[i]);Serial1.print(' ');
    }
    Serial1.println();
    Serial1.println("maximos");
      for (int i = 0; i < numSensores; i++)
      {
        Serial1.print(valmax[i]);
        Serial1.print(' ');
      } Serial1.println();digitalWrite(13,LOW);                   //apaga led para confirmar que termino la calibracion
}

void leer(unsigned int *valores_sensores)                     //funcion que lee los valores en bruto de cada sensor de linea y los guarda en un arreglo
{
      unsigned char i, j;
      for(i = 0; i < numSensores; i++){valores_sensor[i] = 0;}//resetea valores dejandolos en cero
      for(j = 0; j < muestrasPorSensor; j++){                 //cantidad de veces que repite el proces, numero de muestras que toma por cada sensor
         for (i = 0; i < numSensores; i++) {
             valores_sensor[i] += analogRead(pines[i]);   // sumo el resultado del adc
                                           }
                                            }
      for (i = 0; i < numSensores; i++){
             valores_sensor[i] = (valores_sensor[i])/ muestrasPorSensor; //saco el promedio de las muestras para dejar un solo valor
                                  }
}
