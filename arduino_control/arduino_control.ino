/*
 * BIBLIOTECAS
 */
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <math.h>
#include <BasicLinearAlgebra.h>
#define CARACTERES_LCD 16
#define LINEAS_LCD 2
LiquidCrystal_I2C lcd(0x27,CARACTERES_LCD,LINEAS_LCD);  //

using namespace BLA;

/*
 * DEFINICIONES
 */
#define Res1 9500000
#define Res2 1000000
#define DivRes 0.095238


//Screen


//Constantes para la corriente
#define CURRENT A2
#define MIDDLE 511
#define FACTOR 0.066

//Constante para los promedios ponderados exponenciales
#define beta 0.9

/*
  MODELO
*/
//These constants are imported from the dataset
#define mean_volts 12.16983305
#define mean_ampers 9.41150109
#define std_volts 0.68739603
#define std_ampers 13.52794009

//We declare the Weights and biases of each layer
const BLA::Matrix<2, 16> W1 = { 0.62430006, 0.12717175, 0.42998037, 0.6904147 , 0.27597207,
        1.0165218 , 0.29761577, 3.8431664 , 2.0664756 , 0.5920844 ,
        0.16073455, 2.5485642 , 3.5546489 , 2.8624206 , 0.09568144,
        0.19124523, -5.852534 , -5.2285337, -4.991827 , -4.0638022, -5.450951 ,
        -5.272052 , -4.2926   , -3.3046987, -3.1754308, -3.9155848,
        -5.2291093, -1.8140088, -1.3702942, -3.0370028, -5.2218285,
        -5.5125036};
const BLA::Matrix<1, 16> b1 = {-4.2906795, -3.0728273, -3.089645 , -1.0150888, -2.8823996,
       -2.9094179, -2.1194592, -2.3835533,  4.9944296, -1.41283  ,
       -3.2740686, -3.4730322, -2.2808964, -3.8682146, -3.1476908,
       -3.2696233};
const BLA::Matrix<16, 8> W2 = {1.9056520e+01, -4.0600860e-01, -8.4331281e-02,  2.0547993e+01,
        -1.7060755e+01,  1.8920433e+01, -3.3121445e+01,  1.8997896e+01,6.3290458e+00, -1.0944843e-02, -1.4435944e-01,  6.5717301e+00,
        -5.4967289e+00,  6.2665339e+00, -4.4756060e+00,  6.3529229e+00,3.5432146e+00, -1.5367353e-01,  2.9786885e-01, -2.5529273e+00,
        -5.4762948e-01,  3.9637945e+00, -3.4993117e+00,  4.0638728e+00, 2.5422781e+00, -1.0442791e-01, -3.8940310e-02,  3.0018542e+00,
         8.7008137e-01,  3.2081735e+00, -1.1571219e-02,  2.7536001e+00, 4.9532437e+00, -2.9851997e-01,  3.1556365e-01,  3.8818517e+00,
        -4.2159672e+00,  4.4112372e+00, -2.9777384e+00,  4.8367090e+00, 3.7412651e+00,  5.6867361e-02, -4.2545620e-01, -3.1925776e+00,
        -4.0156034e-01,  3.8859844e+00, -3.7608521e+00,  4.0484076e+00, 4.3736038e+00, -2.6900160e-01, -1.7267609e-01, -8.2685101e-01,
        -1.3757815e+00,  3.7716544e+00, -1.9255459e+00,  4.1389446e+00, 2.6982722e+00,  3.9217472e-03,  1.3633363e-01, -4.1282310e+00,
         6.0316032e-01,  2.8294444e+00, -2.4408548e+00,  3.0130897e+00, 1.6090434e+00, -3.2295048e-01, -4.7151685e-02,  4.8779259e+00,
         1.0316520e+00,  1.0394417e+00,  3.5717075e+00,  8.7556660e-01, 2.7181690e+00, -3.2683933e-01, -3.9528954e-01,  1.6030754e+00,
         8.5126954e-01,  3.2815657e+00, -2.2393899e-01,  3.0852129e+00,  8.1514978e+00,  2.4305023e-03, -3.5992730e-01,  1.0679231e+01,
        -7.6983705e+00,  8.5514174e+00, -7.4820070e+00,  8.1274166e+00, -1.0316849e+00, -4.4435728e-01,  2.1185040e-02, -7.8610244e+00,
         4.5825634e+00, -7.4228895e-01, -3.2232563e+00, -1.6598532e+00, 2.1145368e+00, -2.9230177e-01, -2.0245384e-01, -5.1782885e+00,
         1.2199990e+00,  2.0889897e+00, -1.2204541e+00,  2.0457001e+00, 1.1209675e+00, -3.7509167e-01, -3.2037336e-01, -5.1461830e+00,
         1.6114892e+00,  2.1290319e+00, -2.4710581e+00,  1.5678043e+00, 5.9830375e+00,  2.2334063e-01, -3.4174281e-01,  6.0409555e+00,
        -5.6584706e+00,  6.5662041e+00, -4.7080445e+00,  6.4823537e+00, 6.1092224e+00,  2.7196336e-01,  1.9788307e-01,  4.7674236e+00,
        -5.1890726e+00,  6.0662389e+00, -5.5937157e+00,  5.5036969e+00};
const BLA::Matrix<1, 8> b2 = {-0.9135568 , -0.02576767, -0.03037526,  7.282452  ,  1.0195041 ,
       -1.7326539 ,  4.9917283 , -1.6015177};
const BLA::Matrix<8, 1> W3 = {4.5413661e+00,5.7184881e-01,-8.6309388e-03,
  1.2551964e+01,-8.8284073e+00, 4.2664485e+00,-1.5375823e+01, 4.5541182e+00};
const BLA::Matrix<1, 1> b3 = {0.27948856};


/*
 * PROTOTIPOS FUNCIONES
 */
 
unsigned long int LEDS;
float MedicionLeds(void);
void limpiar_pantalla (void);
void actualizar_pantalla_C (float, float, int, int);
float tiempo(float, float);
void limpiar (void);
void pantalla (float, float, int, int);

void setup() {
  Serial.begin(9600);
  lcd.backlight();
  lcd.init();
  limpiar();
}

//Variables para el promedio ponderado
float Av_ampers = 0.0;
float Av_volts = 0.0;

void loop() {
  //Obtener corriente
  int measure = analogRead(CURRENT);
  int Measure_center = measure-MIDDLE;
  float volt_measure = float(5*Measure_center)/1024;
  float Ampers = volt_measure/FACTOR;
  Av_ampers = beta*Av_ampers+(1-beta)*Ampers;
  

  //Obtener tensión
  float Tension = MedicionLeds();
  float Volts = Tension/4;
  Av_volts = beta*Av_volts+(1-beta)*Volts;
  float Av_tension = Av_volts*4;
  
  float tiempo_res = tiempo(Av_volts, Av_ampers);

  //Print the results
  Serial << "Volts: " << Volts << '\n';
  Serial << "Ampers: " << Ampers << '\n';
  Serial << "Time left: " << tiempo_res << '\n';

  /*
   * Pantalla Variables
   */
  int minutosTotales = tiempo_res; // Cambia este valor por la cantidad de minutos que quieras convertir
  int minutosRestantes = minutosTotales % 60;
  int horas = (minutosTotales - minutosRestantes)/60;
  Serial.print("Horas: ");
  Serial.print(horas);
  Serial.print(" Minutos restantes: ");
  Serial.println(minutosRestantes);
  pantalla (Av_tension, Av_ampers, horas, minutosRestantes);
  delay(1000);
}

float tiempo(float tension, float corriente){
  //The input must be normalized
  BLA::Matrix<1, 2> X = {(tension-mean_volts)/std_volts, (corriente-mean_ampers)/std_ampers};
 
  //First layer feed-forward
  BLA::Matrix<1, W1.Cols> Z1 = (X * W1)+b1;
  BLA::Matrix<1, W1.Cols> a1 = Z1;
  for (char i = 0; i <= W1.Cols; i++) {
    if(Z1(0,i) < 0){
      a1(0,i) = 0;
    }
  }

  //Second layer feed-forward
  BLA::Matrix<1, W2.Cols> Z2 = (a1 * W2)+b2;
  BLA::Matrix<1, W2.Cols> a2 = Z2;
  for (char i = 0; i <= W2.Cols; i++) {
    if(Z2(0,i) < 0){
      a2(0,i) = 0;
    }
  }

  //Output layer prediction
  BLA::Matrix<1, W3.Cols> Z3 = (a2 * W3)+b3;
  BLA::Matrix<1, W3.Cols> y_pred = Z3;
  float Time = y_pred(0,0);
  return Time;

}


void pantalla (float volt, float amper, int hora, int minu)
{
  limpiar();
  lcd.setCursor(0, 0);
  lcd.print(volt);
  lcd.setCursor(6, 0);
  lcd.print("V");
  lcd.setCursor(9, 0);
  lcd.print(amper);
  lcd.setCursor(15, 0);
  lcd.print("A");
  lcd.setCursor(0, 0);
  if (hora < 10)
  {
    lcd.setCursor(5,1);
    lcd.print("0");
    lcd.print(hora);
    lcd.setCursor(8, 1);
    lcd.print(minu);
    lcd.setCursor(15, 1);
  }else
  {
    lcd.setCursor(5,1);
    lcd.print(hora);
    lcd.setCursor(7,1);
  }
  lcd.setCursor(7,1);
  lcd.print(":");
  if (minu < 10)
  {
    lcd.setCursor(8, 1);
    lcd.print("0");
    lcd.print(minu);
    lcd.setCursor(15, 1);
  }else
  {
    lcd.setCursor(8, 1);
    lcd.print(minu);
    lcd.setCursor(15, 1);
  }

}

void limpiar(void)
{
  lcd.clear();
}


float MedicionLeds(void){
  int medicion = analogRead(A0);
  float calculo = (medicion*5);  //Vemos el valor de tension a la salida del divisor de tension.
  float voltaje = calculo/1023;
  float resultado = voltaje/DivRes;
  if ((resultado > 49.16) && (resultado <= 52.8)) LEDS = 225; //10 LEDS
  if ((resultado > 48.04) && (resultado <= 49.16)) LEDS = 160; //9 LEDS
  if ((resultado > 47.48) && (resultado <= 48.04)) LEDS = 140; //8 LEDS
  if ((resultado > 46.36) && (resultado <= 47.48)) LEDS = 130; //7 LEDS
  if ((resultado > 45.24) && (resultado <= 46.36)) LEDS = 110; //6 LEDS
  if ((resultado > 44.68) && (resultado <= 45.24)) LEDS = 90; //5 LEDS
  if ((resultado > 43.56) && (resultado <= 44.68)) LEDS = 80; //4 LEDS
  if ((resultado > 42.44) && (resultado <= 43.56)) LEDS = 50; //3 LEDS
  if ((resultado > 41.88) && (resultado <= 42.44)) LEDS = 40; //2 LEDS
  if ((resultado > 40.2) && (resultado <= 41.88)) LEDS = 20; //1 LED
  if ((resultado > 0) && (resultado<= 40.2)) LEDS = 0; //0 LED
  analogWrite(3, LEDS);
  Serial.println(LEDS);
  Serial.println(voltaje);
  Serial.println(resultado);  //Sacamos el valor total de las 4 baterias.
  Serial.print("MEDICION: ");
  Serial.println(medicion);
  return resultado;
}
