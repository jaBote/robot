//////////////////////////
// Library Includes
/////////////////////////
#include <Servo.h>
#include <SoftwareSerial.h>

/////////////////////////////////////////////////////////////////////////////////////////
// Constants - Define pins
/////////////////////////////////////////////////////////////////////////////////////////
#define IN1 4    // Input3 conectada al pin 4
#define IN2 12    // Input4 conectada al pin 12
#define ENA  3    // ENA conectada al pin 3 de Arduino

#define IN3 7    // Input3 conectada al pin 7
#define IN4 8    // Input4 conectada al pin 8 
#define ENB 6    // ENB conectada al pin 6 de Arduino

#define TRIG_DEL 11   // Trigger a wave     - Output Pin
#define ECHO_DEL 9    // Receive a echo     - Input Pin  
	
#define TRIG_TRAS 10   // Trigger a wave    - Output Pin
#define ECHO_TRAS 2    // Receive a echo     - Input Pin 

#define FOTO_IZQ 13  // Fotorreceptores
#define FOTO_DER 0

#define DIR_DEL 0
#define DIR_TRAS 1

#define DIR_UP 0
#define DIR_DOWN 1

const int dir_cajas;
long distandel;          // ultrasonic sensor variable
long tiempo1;             // ultrasonic sensor variable
long distantra;          // ultrasonic sensor variable
long tiempo2;             // ultrasonic sensor variable
int izquierda;
int derecha;
long obstaculo=20; //pendiente de ajuste (cm)
int variacion=10; //pendiente de ajustar (grados)
int grados=90;       // Se inicializa en el programa [jaBote]
int ya=1;
 
/////////////////////////////////////////////////////////////////////////////////////////
// Global Variables and Objects
/////////////////////////////////////////////////////////////////////////////////////////
Servo servo;  // create servo object to control a servo

/////////////////////////////////////////////////////////////////////////////////////////
// Functions Headers
////////////////////////////////////////////////////////////////////////////////////////
void traccionTrasera(int velocidad, int sentido, int tiempo);
void elevarCarretilla(int velocidad, int sentido, int tiempo, int nivel);
void direccionDelantera(int posicion, int tiempo);
void leerDistanciadelantera();
void leerDistanciatrasera();
void redirecciona ();

////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
void setup()
{
 // inicia el protocolo de comunicación a PC para debuggear
 Serial.begin(9600);
 pinMode (ENB, OUTPUT); 
 pinMode (IN3, OUTPUT);
 pinMode (IN4, OUTPUT);
 
 pinMode (ENA, OUTPUT); 
 pinMode (IN1, OUTPUT);
 pinMode (IN2, OUTPUT);
 
 servo.attach(5); //Pin 5 para el PWM del servomotor
 
 pinMode(TRIG_DEL, OUTPUT); /*activación del trigDistancia como salida: para el pulso ultrasónico*/
 pinMode(ECHO_DEL, INPUT); /*activación del ecoDistancia como entrada: tiempo del rebote del ultrasonido*/
 pinMode(TRIG_TRAS, OUTPUT); /*activación del trigDistancia como salida: para el pulso ultrasónico*/
 pinMode(ECHO_TRAS, INPUT); /*activación del ecoDistancia como entrada: tiempo del rebote del ultrasonido*/

 pinMode(FOTO_IZQ,INPUT);
 pinMode (FOTO_DER,INPUT);
}


void loop(){
// CICLO 1



//----------------------------------------------------Pasos a seguir:
// 1.Estamos en el punto 1
delay(2000); //Esperamos 2 segundos antes de arrancar el código

// 2.Motor a 90º
direccionDelantera(90,0);   //ponemos las ruedas rectas (90º)

// 3.Seguidor de linea 1
leerDistanciadelantera();   //lee distancia

if(distandel>obstaculo) //mientras que la distancia sea menor que obstaculo(15) 
		{
			traccionTrasera(255,DIR_DEL, 0);
                        delay(2000);
		}
		
	leerDistanciadelantera();   //lee distancia
	
                                   //Seguidor
while(distandel>obstaculo) //mientras que la distancia sea menor que obstaculo(15) 
		{
			traccionTrasera(210,DIR_DEL, 0); //avanza 0.5 segundos hacia delante
			if (digitalRead(FOTO_IZQ) == 0)  // 1 es negro 0 es blanco
				{
				grados=(grados-variacion);
				if(grados<55)
					{
					grados=55;
					}	
				servo.write(grados);
				delay(1500);
				}
			if (digitalRead(FOTO_DER) == 0)
				{
				grados=(grados+variacion);
				if(grados>135)
					{
					grados=135;
					}
					servo.write(grados);
					delay(1500);
				}
			leerDistanciadelantera(); //lee distancia
  //delay(100);
		}
traccionTrasera(0,DIR_DEL,0);


//Detectamos cajas (Punto A)  //hasta aquí esta hecho
// 4.Pala en posición A1 (verde)
while(ya==1)
		{
			int sensorValue = analogRead(A1);
			if (sensorValue<1000)
				{          //Estamos en el punto A?
					elevarCarretilla(255, DIR_UP, 0); //No, sígue subiendo. velocidad sentido(0sube) tiempo
                }
			else 
				{
					elevarCarretilla(0, 0, 0); //Sí, la para. Pasa
					ya=2;              //Cambiamos el valor de "ya" para que salga del bucle
					delay (2000);
				}
         }
// 5.Avanzamos lo suficiente para poder subir la caja, distancia x
direccionDelantera(90,0);   //ponemos las ruedas rectas (90º)
leerDistanciadelantera();   //lee distancia
while(distandel>dir_cajas) //mientras que la distancia sea menor que distancia cajas
		{
			traccionTrasera(255,DIR_DEL,0);
			leerDistanciadelantera();
		}
//6. Subimos la caja hasta la posición A0 (lila)
while(ya==2)
		{
			int sensorValue = analogRead(A0);
			if (sensorValue<1000)
				{          //Estamos en el punto A0?
					elevarCarretilla(255, DIR_UP, 0); //No, sígue subiendo. velocidad sentido(0sube) tiempo
                }
			else 
				{
					elevarCarretilla(0, 0, 0); //Sí, la para. Pasa
					ya=3;              //Cambiamos el valor de "ya" para que salga del bucle
					delay (2000);
				}
        }
//7.Sensor de presión detecta que hay caja, continuamos.

//8.Retrocedemos hasta que el ultrasonido trasero detecta obstáculo

direccionDelantera(90,0);   //ponemos las ruedas rectas (90º)
leerDistanciatrasera();   //lee distancia
while(distantra>obstaculo) //mientras que la distancia sea menor que distancia cajas
		{
			traccionTrasera(255,DIR_TRAS,0);
			leerDistanciatrasera();
		}
//9.=3.Seguimos linea 2 hasta detectar cajas con ultrasonido delantero(Punto B)
//---REPETIMOS CÓDIGO SEGUIDOR LINEA 1 ¿CREAR FUNCIÓN?

//10.=5.Avanzamos hasta distancia x
//Bajamos hasta posición A3 (amarillo)
while(ya==3)
		{
			int sensorValue = analogRead(A0);
			if (sensorValue<1000)
				{          //Estamos en el punto A0?
					elevarCarretilla(255, DIR_DOWN, 0); //No, sígue baja. velocidad sentido(0sube) tiempo
                }
			else 
				{
					elevarCarretilla(0, 0, 0); //Sí, la para. Pasa
					ya=4;              //Cambiamos el valor de "ya" para que salga del bucle
					delay (2000);
				}
        }

//11.=8.Marcha atrás hasta obstáculo
//SE QUEDA EN LA POSICIÓN INICIAL
// // // CICLO 2

// // // Pasos a seguir:
// // // Estamos en el punto 1
// // // Motor a 90º
// // // Seguidor de linea 1
// // // Detectamos cajas (Punto A)
// // // Pala en posición A2 (azul)

// // // Avanzamos lo suficiente para poder subir la caja, distancia x
// // // Subimos la caja hasta la posición A0 (lila)
// // // Sensor de presión detecta que hay caja, continuamos.
// // // Retrocedemos hasta que el ultrasonido trasero detecta obstáculo
// // // Seguimos linea 2 hasta detectar cajas con ultrasonido delantero(Punto B)
// // // Avanzamos hasta distancia x
// // // Bajamos hasta posición A2 (azul)
// // // Marcha atrás hasta obstáculo

// // // CICLO 3

// // // Pasos a seguir:
// // // Estamos en el punto 1

// // // Motor a 90º
// // // Seguidor de linea 1
// // // Detectamos cajas (Punto A)
// // // Pala en posición A3 (amarillo)

// // // Avanzamos lo suficiente para poder subir la caja, distancia x
// // // Subimos la caja hasta la posición A0 (lila)
// // // Sensor de presión detecta que hay caja, continuamos.
// // // Retrocedemos hasta que el ultrasonido trasero detecta obstáculo
// // // Seguimos linea 2 hasta detectar cajas con ultrasonido delantero(Punto B)
// // // Avanzamos hasta distancia x
// // // Bajamos hasta posición A1 (verde)
// // // Marcha atrás hasta obstáculo

  
 
 
     
}
/////////////////////////
// RESTO DE FUNCIONES  //
/////////////////////////
void traccionTrasera(int velocidad, int sentido, int tiempo){

 if(sentido==1){  
   //Preparamos la salida para que el motor gire en un sentido
   digitalWrite (IN1, HIGH);
   digitalWrite (IN2, LOW);
  }else{
   //Preparamos la salida para que el motor gire en el otro sentido
   digitalWrite (IN1, LOW);
   digitalWrite (IN2, HIGH);    
  }

  // Aplicamos PWM al pin ENB
  analogWrite(ENA,velocidad);
  
  //Comprobar si hemos dado un valor de tiempo
  if(tiempo>0){
     delay(tiempo);
     analogWrite(ENA,0);
  }

}


/*void elevarCarretilla(int velocidad, int sentido, int tiempo){
  
  if(sentido==DIR_DOWN){  
   //Preparamos la salida para que el motor gire en un sentido
   digitalWrite (IN3, HIGH);
   digitalWrite (IN4, LOW);
  }
  else{
   //Preparamos la salida para que el motor gire en el otro sentido
   digitalWrite (IN3, LOW);
   digitalWrite (IN4, HIGH);    
  }

  // Aplicamos PWM al pin ENB, haciendo girar el motor
  analogWrite(ENB,velocidad);
  
  //Comprobar si hemos dado un valor de tiempo
  if(tiempo>0){
     delay(tiempo);
     analogWrite(ENB,0);
  }
  
}
*/
void direccionDelantera(int posicion, int tiempo){
   // Comprobar si el valor de la posicion dada está entre 0 y 180 grados
   if ((posicion>=0) && (posicion<=180)) {
    servo.write(posicion);
   }
   
   //Comprobar si hemos dado un valor de tiempo
   if(tiempo>0){
     delay(tiempo);
   }
}


void leerDistanciadelantera(){
  
  digitalWrite(TRIG_DEL,LOW); /* Por cuestión de estabilización del sensor*/
  delayMicroseconds(5);
  digitalWrite(TRIG_DEL, HIGH); /* envío del pulso ultrasónico*/
  delayMicroseconds(10);
  tiempo1=pulseIn(ECHO_DEL, HIGH); /* Función para medir la longitud del pulso entrante. 
                                         Mide el tiempo que transcurrido entre el envío
                                         del pulso ultrasónico y cuando el sensor recibe el rebote*/
  distandel= int(0.017*tiempo1);       /*fórmula para calcular la distancia obteniendo un valor entero*/ 
}
void leerDistanciatrasera(){ 
  digitalWrite(TRIG_TRAS,LOW); /* Por cuestión de estabilización del sensor*/
  delayMicroseconds(5);
  digitalWrite(TRIG_TRAS, HIGH); /* envío del pulso ultrasónico*/
  delayMicroseconds(10);
  tiempo2=pulseIn(ECHO_TRAS, HIGH); /* Función para medir la longitud del pulso entrante. 
                                         Mide el tiempo que transcurrido entre el envío
                                         del pulso ultrasónico y cuando el sensor recibe el rebote*/
  distantra= int(0.017*tiempo2);       /*fórmula para calcular la distancia obteniendo un valor entero*/ 

}

void redirecciona ()
{
  while (digitalRead(FOTO_IZQ) == 0) { //1 es negro 0 es blanco
    grados=(grados-variacion); //mirar signo
    servo.write(grados);
    delay(1500); //tiempo para que vuelva al carril

  }
  
  while (digitalRead(FOTO_DER) == 0) {
    grados=(grados+variacion);
    servo.write(grados);
    delay(1500); 
  }
}


