# proyect_crontol

//funciona de manera correcta con esos valores de k
//double Kp=70, Ki=20, Kd=50; 
//
//Programa para presentar el PID
//
//
#include <TimerOne.h>
#include <Wire.h>
#include <PID_v1.h>
#include <Time.h>
#include <TimeAlarms.h>
#include <DallasTemperature.h>
#include <LiquidCrystal_I2C.h>

#define AC_PIN 11
#define B_UP 9
#define B_NEXT 8
#define B_BACK 7
#define B_DOWN 6
#define ONE_WIRE_BUS 4
#define SENSOR_RESOLUTION  10

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress sensorDeviceAddress;

LiquidCrystal_I2C lcd(0x27,20,4);                      // set the LCD address to 0x3F for a 16 chars and 2 line display

//Definicion de Variables:
//double Setpoint, Input, Output, error;
//double Kp=70, Ki=20, Kd=50;                        //Ganancias Ki y Kd sin el periodo.
//PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
//int control;
//float temp;
int day_count=1;
int h,m,s,D=1,M=1,A=20;
int ok=0;
unsigned long MillisAnterior = 0;
unsigned long previousMillis = 0;
const int tiempoAntirrebote = 50;
//////////////////////////////////////
//////////////////////////////////////
// Parametros para analisis a lazo cerrado
float Setpoint = 0;       // Celsius
// Constantes de PID
float Kc = 8; float Tao_I = 80;
int Tiempo0 = 0;     // Retardo (en milisegundos) para ejecutar cambio escalon cuando se encuentra 
                         //a lazo abierto o cambio en el septpoint a lazo cerrado 
//int A = 0;           // Pin A0 de entrada analogica para sensor LM35 (Variable de salida)
float Potencia = 50;  // Potencia inicial enviada al dimmer en rango de 0 a 100 (Variable de entrada)
//  Declaracion de variables
unsigned long Tiempo_previo = 0; 
unsigned long Tiempo_actual = 0;
double Read_Delay = 1000;     // Periodo de muestreo en milisegundos
int Temperatura = 0;       // Celsius
double sp = 0;   
// Variables para PID
double error = 0;
double PID_error = 0;
double previous_error = 0;
double PID_value = 0;
double Error_INT = 0;
double Error_D = 0;
float contador=0;
int tiempo=0;
float op=0;
float potencia = 0;  // Potencia inicial enviada al dimmer en rango de 0 a 100 (Variable de entrada)
//Definicion de Variables:
int control;
float temp;
int modo;
float Kp=17.79;  //nuevos datos un polo rapido y robusto  
float Ki=0.12562;
float Kd=32.3349;
//////////////////////////////////////
//////////////////////////////////////
//////////////////////////////////////
//////////////////////////////////////
//Interrupcion para deteccion de cruce por cero
void zero_cross_detect() {  
  digitalWrite(AC_PIN, LOW);                      // Apaga el TRIAC
  Timer1.initialize(control);                // Inicializa Timer1 cargando la variable de control
  Timer1.attachInterrupt(disparo);      
} 
//Interrupcion Timer1     
void disparo() {                                        
      digitalWrite(AC_PIN, HIGH);                 //Cuando desborda el Timer1, produce el disparo
      Timer1.stop();                           
}

void setup() {
  Serial.begin(9600);
  pinMode(AC_PIN, OUTPUT);                                                      // Pin 9 como salida para TRIAC
  pinMode(B_UP,INPUT_PULLUP);
  pinMode(B_NEXT,INPUT_PULLUP);
  pinMode(B_BACK,INPUT_PULLUP);
  pinMode(B_DOWN,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), zero_cross_detect, RISING);    // Pin 2 como interrupcion externa por flanco de subida para detecion de cruce por cero. 
  lcd.init();                      // initialize the lcd 
  lcd.backlight();
  lcd.clear();
  sensors.getAddress(sensorDeviceAddress, 0);
  sensors.setResolution(sensorDeviceAddress, SENSOR_RESOLUTION); 
  sensors.begin();
  Serial.println("CLEARDATA"); //limpia los datos previos 
    
while(ok<2){
   while(ok==0){
      lcd.setCursor(0,0);
      lcd.print("Ingrese setpoint:");
      lcd.print("                 ");
   if(!antirrebote(B_UP)){
        Setpoint =45; 
        delay(1000);
        ok++;}
   if(!antirrebote(B_NEXT)){
        Setpoint =35; 
        delay(1000);
        ok++;}
   if(!antirrebote(B_DOWN)){
        Setpoint =30; 
        delay(1000);
        ok++;}
   if(!antirrebote(B_BACK)){
        Setpoint =40; 
        delay(1000);
        ok++;}  
}
while(ok==1){
      lcd.setCursor(0,0);
      lcd.print("Ingrese tipo:     ");
     if(!antirrebote(B_UP)){
        modo=3; 
        delay(1000);
        ok++;}
     if(!antirrebote(B_NEXT)){
         modo=3; 
        delay(1000);
        ok++;}
     if(!antirrebote(B_DOWN)){
         modo=1; 
        delay(1000);
        ok++;}
     if(!antirrebote(B_BACK)){
        modo=2; 
        delay(1000);
        ok++;}  
    }
  }

   if(modo==1){
   Kp=17.79;  //modo P  
   Ki=0;
   Kd=0;
  lcd.setCursor(0,0);
  lcd.print("                 ");
  lcd.setCursor(12,0);
  lcd.print("P.");
   }
   if(modo==2){
   Kp=14.924;   //modo PI  
   Ki=0.44237;
   Kd=0;
  lcd.setCursor(0,0);
  lcd.print("                 ");
  lcd.setCursor(12,0);
  lcd.print("PI.");
    }
   if(modo==3){
   Kp=17.79;    //modo PID  
   Ki=0.12562;
   Kd=32.3349;
  lcd.setCursor(0,0);
  lcd.print("                 ");
  lcd.setCursor(12,0);
  lcd.print("PID");
    }
//////////////////////////////////////         
}
                  
void loop() {
   // Mostrar el reloj en el monitor serial y lcd
   unsigned long MillisActual = millis();
    lcd_time();
    serial_monitor_time();
    MillisAnterior = MillisActual;
  
  sensors.requestTemperatures();
  temp=sensors.getTempCByIndex(0);  //Lectura de temperatura

Tiempo_actual = millis(); // Tiempo Actual  
      
control = map(Potencia,0,100,9500,1000);//valor = map(Potencia,0,100,7600,10);    //9500,1000
 
if(Tiempo_actual - Tiempo_previo >= Read_Delay){
       Tiempo_previo += Read_Delay;                
 

if (Tiempo_actual >= Tiempo0){
      PID_error = Setpoint - temp;                   //Calculo del error    
      Error_INT = Error_INT + PID_error*1;      //Calculo de la integral del error
      Error_D = ((PID_error - previous_error)/1) ;
      PID_value = Kp * PID_error +  Ki * Error_INT + Kp * Error_D ;       //PID_value = Kc*(PID_error + (1/Tao_I)*Error_INT);     //Calculo de la salida del controlador PI
      sp = Setpoint;  }
    // Limite de salida del controlador
    if(PID_value < 0)
    {      PID_value = 0;       }
    if(PID_value > 100)
    {      PID_value = 100;    }
    Potencia = PID_value;   //Asignacion a la entrada de la planta.
    previous_error=PID_error;
    
    }
//////////////////////////////////////////////////////
//////////////////////////////////////////////////////
        Serial.print("Potencia:");
        Serial.print(Potencia);
        Serial.print(" ");
        Serial.print("Temperatura:");
        Serial.print(temp);
        Serial.print(" ");
        Serial.print("Setpoint:");
        Serial.print(sp);
        Serial.print(" ");
        Serial.print("PID_value:");
        Serial.print(PID_value);
        Serial.print(" ");
        Serial.print("PID_error:");
        Serial.print(PID_error);
        Serial.print(" ");
        Serial.print("Error_D:");
        Serial.print(Error_D);
        Serial.print(" ");
        Serial.print("Error_INT:");
        Serial.print(Error_INT);    
        Serial.print("Temp.: ");
        Serial.print(temp);
        Serial.print("  Control: ");
        Serial.println(control);
  //lcd.setCursor(0,0);
  //lcd.setCursor(0,0);
  //lcd.print("           ");
  //lcd.setCursor(13,0);
  //lcd.print(modo); 
  lcd.setCursor(3,0);
  lcd.print("Control");
  lcd.setCursor(1,2);
  lcd.print("Temp.:");
  lcd.print(temp); 
  lcd.setCursor(1,3);
  lcd.print("SetPoint:");
  lcd.print(sp);
  
  //if(modo==1){
  //lcd.setCursor(12,0);
  //lcd.print("P.");
  //} 
  //if(modo==2){
  //lcd.setCursor(12,0);
  //lcd.print("PI.");
  //} 
  //if(modo==3){
  //lcd.setCursor(12,0);
  //lcd.print("PID");
  //} 
//////////////////////////////////////      
}//loop

 
 
void serial_monitor_time() {
  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
  Serial.println();
}

void printDigits(int digits) {
  Serial.print(":");
  if (digits < 10)
    Serial.print('0');
  Serial.print(digits);
}

void lcd_time(){
  lcd.setCursor(1, 1);
  lcd.print("    ");
  if(hour() < 10) lcd.print('0');
  lcd.print(hour(), DEC);
  lcd.print(':');
  if(minute() < 10) lcd.print('0');
  lcd.print(minute(), DEC);
  lcd.print(':');
  if(second() < 10) lcd.print('0');
  lcd.print(second(), DEC); 
}

bool antirrebote(int pin){
  int tiempo = 0;
  bool estado;
  bool estadoAnterior;
  do{
    estado = digitalRead(pin);
    if(estado != estadoAnterior){
      tiempo = 0;
      estadoAnterior = estado;
    }
    else tiempo++;
    delay(1);
  }while(tiempo < tiempoAntirrebote);
  return estado;
} 
