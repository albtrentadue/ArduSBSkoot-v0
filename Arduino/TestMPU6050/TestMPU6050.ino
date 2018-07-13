/***
 * DIY Self Balancing Scooter ATR
 *
 * Test sketch for reading MPU-6050
 * 
 * By Alberto Trentadue 2017
 */

//Per I2C
#include <Wire.h>

#define STATUS_LED 13

//Indirizzi I2C
#define ADDR_ACCEL 0x68 //0b1101000

//Durata di un ciclo loop circa 5 msec
#define RITARDO_MAIN 5

#define CICLI_CALC 20 // cicli da attendere per effettuare i calcoli
byte cnt_calc = CICLI_CALC;
#define CICLI_HEART 100 // cicli da attendere per l'heartbeat
byte cnt_heart = CICLI_HEART;
byte heart = 0;
#define CICLI_COMM 50 // cicli da attendere per comunicare i dati
byte cnt_comm = CICLI_COMM;

//Riduzione di precisione per divisione
#define PREC_REDUX_X 16
#define PREC_REDUX_Z 8

//Variabili per le misure dell'accelerometro MPU-6050
byte ultimo_campione=0;
int16_t z_acc;
int32_t somma_z_acc = 0;
int16_t serie_z[CICLI_CALC] = {0};
int16_t media_z_acc;
int16_t x_acc;
int32_t somma_x_acc = 0;
int16_t serie_x[CICLI_CALC] = {0};
int16_t media_x_acc;
int16_t t_acc;


//-- COSTANTI PER LA GESTIONE ACCELERAZIONE
//Valore di g misurato dal MPU6050 da calibrare a mano.
#define G_MPU6050 17800
//bx,min: Valore minimo di componente frontale bx per rilevare la pendenza della pedana
#define BY_MINIMO 800
//bz,margine: Valore oltre il quale la componente bz si considera verticale
#define BZ_MARGINE 17650

//------ FUNZIONI -------

void setup() {
  // Inizializzazione I2C
  Wire.begin();

  // Inizializzazione Seriale
  Serial.begin(115200);

  // Inizializzazione I/O
  pinMode(STATUS_LED, OUTPUT);

  //Setup dell'accelerometro
  setup_mpu6050();
  
}

void loop() {
  leggi_accelerometro();
  ultimo_campione = (ultimo_campione + 1) % CICLI_CALC; 
  calcola_medie();
  report_status();
  heartbeat();
  delay(RITARDO_MAIN);
}

/**
 * Legge i valori Z e X e la temperatura dall'accelerometro via I2C
 * I valori sono memorizzati in z_acc, z_acc e t_acc
 * Si considera un accelerometro del tipo MPU-6050
 */
void leggi_accelerometro(void) 
{
  //Esegue la procedura di accesso all'MPU-6050
  leggi_mpu6050();

  z_acc /= PREC_REDUX_Z;
  x_acc /= PREC_REDUX_X;

  //aggiorna i campioni delle accelerazioni
  somma_z_acc += (z_acc - serie_z[ultimo_campione]);
  somma_x_acc += (x_acc - serie_x[ultimo_campione]);

  serie_z[ultimo_campione] = z_acc;
  serie_x[ultimo_campione] = x_acc;    
}

/**
 * Esegue le medie delle misure che devono essere mediate
 */
void calcola_medie(void)
{
  media_z_acc = somma_z_acc / CICLI_CALC;
  media_x_acc = somma_x_acc / CICLI_CALC; 
}

/**
 * Scrive lo stato del programma sulla seriale
 * Eseguito ogni CICLI_REPORT iterazioni
 */
void report_status(void)
{  
  cnt_comm--;
  if (cnt_comm == 0) {
    Serial.print(F("STATUS"));
    Serial.print(F(":ZACC="));Serial.print(z_acc, DEC);
    Serial.print(F(";XACC="));Serial.print(x_acc, DEC);    
    Serial.print(F(":ZMED="));Serial.print(media_z_acc, DEC);
    Serial.print(F(";XMED="));Serial.print(media_x_acc, DEC);        
    Serial.println();

    cnt_comm = CICLI_COMM;
  }  
}

/**
 Heartbeat on the built_in Led + EXT_LED
 */
void heartbeat() 
{
  cnt_heart--;
  if (cnt_heart == 0) {
    heart++;
    digitalWrite(LED_BUILTIN, bitRead(heart,0));
    cnt_heart = CICLI_HEART;
  }
}
