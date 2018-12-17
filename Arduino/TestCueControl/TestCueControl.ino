 /***
 * DIY Self Balancing Scooter ATR
 *
 * Test controllo dello spunto sotto carico.
 * Utilizza le connessioni effettive realizzate dalla scheda di
 * interfaccia Arduino/Controllers
 * 
 * By Alberto Trentadue 2018
 */

//Per I2C
#include <Wire.h>
 
#define STATUS_LED 13
#define PWM_DX 3
#define PWM_SX 6
#define FRENATA_DX 2
#define INVERS_DX 4
#define FRENATA_SX 5
#define INVERS_SX 7

//Gestione rivelatori di presenza
//Pin connesso ai rivelatori
#define PRESENZA 8
//Numero di test per convalidare la presenza
#define CONFERME_PRESENZA 10
byte cnt_presenza = CONFERME_PRESENZA;
bool presenza = false;

//-- COSTANTI PER LA GESTIONE ACCELEROMETRO
//Coefficienti di riduzione di precisione per divisione
#define PREC_REDUX_X 16
#define PREC_REDUX_Z 8
//bx,min: Valore minimo (ridotto) di componente frontale bx per attivare il moto
//TODO: Verificare sperimentalmente
#define BX_MINIMO 20
//Compensazione della pendenza di montaggio dell'accelerometro, per avere 0 secondo livella
#define BX_COMP_MONTAGGIO -39
//Costanti stato inclinazione pedana
#define PP0 0
#define PEND_AV 1
#define PEND_IND 2

#define CICLI_MEDIE 50 // Campioni considerati per le medie
byte cnt_calc = CICLI_MEDIE;


//Variabili per le misure dell'accelerometro MPU-6050
byte ultimo_campione = 0;
int16_t x_acc; //Globale: usata nel MPU-6050
int32_t somma_x_acc = 0;
int16_t serie_x[CICLI_MEDIE];
int16_t media_x_acc;

//Livello di throttle da applicare per il PWM
float throttle_dx = 0.0;
float throttle_sx = 0.0;
//Valore effettivamente applicato al PWM
byte pwm_byte = 0;

#define CICLI_TEST_PWM 1600
#define CICLI_CAMBIO_ANDAM 1200
#define STEP_ANDAM_1 4
int16_t cnt_app_pwm = 0;
#define THROTTLE_MIN 115
#define THROTTLE_INIT 140
#define MAX_THROTTLE 190
#define THROTTLE_SPAN (MAX_THROTTLE - THROTTLE_INIT)


#define CICLI_HEART 100 // cicli da attendere per l'heartbeat
byte cnt_heart = CICLI_HEART;
byte heart = 0;
#define CICLI_COMM 50 // cicli da attendere per comunicare i dati
byte cnt_comm = CICLI_COMM;

//------ FUNZIONI -------

void setup() {
  // Inizializzazione I2C
  Wire.begin();

  // Inizializzazione Seriale
  Serial.begin(115200);

  // Inizializzazione I/O
  pinMode(PWM_DX, OUTPUT);
  pinMode(PWM_SX, OUTPUT);
  pinMode(FRENATA_DX, OUTPUT);
  pinMode(FRENATA_SX, OUTPUT);
  pinMode(INVERS_DX, OUTPUT);
  pinMode(INVERS_SX, OUTPUT);
  pinMode(STATUS_LED, OUTPUT);
  pinMode(PRESENZA, INPUT_PULLUP);

  //Setup dell'accelerometro
  setup_mpu6050();

  //Azzera i campioni da mediare
  byte i;
  for (i = 0; i < CICLI_MEDIE; i++) {    
    serie_x[i] = 0;
  }

  //Stati iniziali switch controllo
  digitalWrite(FRENATA_DX, LOW);
  digitalWrite(FRENATA_SX, LOW);
  digitalWrite(INVERS_DX, LOW);
  digitalWrite(INVERS_SX, LOW);

  //Il controller deve essere tenuto con throttle a zero
  //se l'hoverboard era stato spento in modo brusco
  analogWrite(PWM_DX, 0);
  analogWrite(PWM_SX, 0);
  delay(500);

}

void loop() {
  // put your main code here, to run repeatedly:
  leggi_accelerometro();
  leggi_presenza();
  calcola_medie();
  ultimo_campione = (ultimo_campione + 1) % CICLI_MEDIE;
  applica_pwm();
  report_status();
  heartbeat();
  delay(1);  
}

/**
   Legge i valori Z e X e la temperatura dall'accelerometro via I2C
   I valori sono memorizzati in x_acc e t_acc
   Si considera un accelerometro del tipo MPU-6050
*/
void leggi_accelerometro(void)
{
  //Esegue la procedura di accesso all'MPU-6050
  leggi_mpu6050();
  //Riduzione di precisione  
  x_acc /= PREC_REDUX_X;
  //Compensazione angolo di montaggio X scheda accelerometro
  x_acc += BX_COMP_MONTAGGIO;

  //z_acc e x_acc sono al negativo per come è stato montato l'MPU6050  
  somma_x_acc += (-x_acc - serie_x[ultimo_campione]);  
  serie_x[ultimo_campione] = -x_acc;
  
}


/**
   Verifica lo stato del pulsante di presenza.
   I pulsanti di presenza sono in parallelo, normalmente chiusi.
   Lo stato di presenza deve essere confermato per un numero di cicli = CONFERME_PRESENZA
   per indicare la presenza effettiva sulla pedana (debounce).
*/
void leggi_presenza(void)
{
  if (digitalRead(PRESENZA) == LOW) { //Entrambi pulsanti chiusi
    if (cnt_presenza > 0) --cnt_presenza;
    else //cnt_presenza è 0
      presenza = true;
  }
  else { //Almeno un pulsante aperto
    if (cnt_presenza < CONFERME_PRESENZA) ++cnt_presenza;
    else //cnt_presenza è CONFERME_PRESENZA
      presenza = false;
  }
}


/**
   Esegue le medie delle misure che devono essere mediate
*/
void calcola_medie(void)
{
  media_x_acc = somma_x_acc / CICLI_MEDIE;
}

/**
   Ritorna lo stato di inclinazione della pedana in base
   agli angoli misurati.
   Valori possibili:
   PP0: orizzontale
   PEND_AV: Inclinata in avanti
   PEND_IND: Inclinata all'indietro
*/
byte stato_inclinazione(void) {
  if (media_x_acc <= -BX_MINIMO) return PEND_IND;
  if (media_x_acc >= BX_MINIMO) return PEND_AV;  
  return PP0;
}

/**
   Definisce un'andamentoper i PWM delle ruote e lo applica
   per un tempo massimo di 2 sec
*/
void applica_pwm(void)
{
  if (cnt_app_pwm < CICLI_CAMBIO_ANDAM) cnt_app_pwm += STEP_ANDAM_1;
  else if (cnt_app_pwm < CICLI_TEST_PWM) cnt_app_pwm++;

  byte incl = stato_inclinazione();
  if ((incl == PEND_AV) && presenza ) {
    //Rampa lineare tra THROTTLE_INIT e MAX_THROTTLE
    float perc = 1.0 * cnt_app_pwm / CICLI_TEST_PWM;
    pwm_byte = byte(THROTTLE_INIT + THROTTLE_SPAN * perc); 
  }
  else {
    pwm_byte = THROTTLE_MIN;
    cnt_app_pwm = 0; 
  }
    
  if (cnt_app_pwm < CICLI_TEST_PWM) {
    analogWrite(PWM_DX, pwm_byte);
    analogWrite(PWM_SX, pwm_byte);
  }
  else {
    analogWrite(PWM_DX, 0);
    analogWrite(PWM_SX, 0);    
  }
}


/**
   Scrive lo stato del programma sulla seriale
   Eseguito ogni CICLI_REPORT iterazioni
*/
void report_status(void)
{
  cnt_comm--;
  if (cnt_comm == 0) {
    Serial.print(F(";C;")); Serial.print(cnt_app_pwm, DEC);
    Serial.print(F(";XM;")); Serial.print(media_x_acc, DEC);
    Serial.print(F(";Td;")); Serial.print(pwm_byte, DEC);
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
    digitalWrite(LED_BUILTIN, bitRead(heart, 0));
    cnt_heart = CICLI_HEART;
  }
}
