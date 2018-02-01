/***
 * DIY Self Balancing Scooter ATR
 * Self balancing scooter controller sketch using one single rolling board
 * and one single accelerometer sensor.
 * It drives 2 brushless controllers for both speed and direction and is 
 * controlled by a Wiinunchuck
 * 
 * By Alberto Trentadue 2017
 */

//Per I2C
#include <Wire.h>

#define STATUS_LED 13
#define VALIM_ANALOG A0

// Pin dei PWM di controllo brushless
#define PWM_DX 3
#define FRENATA_DX 2
#define INVERS_DX 4
#define MISVEL_DX A1
#define PWM_SX 6
#define FRENATA_SX 5
#define INVERS_SX 7
#define MISVEL_SX A2
//Pulsante di presenza, gestire con debounce
#define PRESENZA 8
#define CONFERME_PRESENZA 10
byte cnt_presenza = 0;
bool presenza = false;

//Non usato in v1.0
#define FUNC1 9
#define FUNC2 10

//Indirizzi I2C
#define ADDR_NUNCHUCK_W 0xA4
#define ADDR_NUNCHUCK_R 0xA5
#define ADDR_ACCEL 0x68 //0b1101000

//STATI FUNZIONAMENTO PROGRAMMA
#define STATO_FERMO 0
#define STATO_ACCEL_AVANTI 1
#define STATO_REGIME_AVANTI 2
#define STATO_DECEL_AVANTI 3
#define STATO_ACCEL_INDIETRO 4
#define STATO_REGIME_INDIETRO 5

//Stato di funzionamento del programma
byte stato_prog = STATO_FERMO;

// Livelli analogici del partitore batteria
// TODO: Calibrare rispetto al valore massimo della batteria
#define BATT_OK 1000
#define BATT_MIN 800
int livello_batt = 0;

//Livello di throttle via PWM
byte pwm_throttle;
//Stato comando di frenata. Attivo ALTO, pilota un NPN in saturazione
byte frenata = LOW;
//Stati comando di inversione marcia. Attivo ALTO, pilota un NPN in saturazione
byte inversione_dx = LOW;
byte inversione_sx = LOW;

//Durata di un ciclo loop circa 5 msec
#define RITARDO_MAIN 5

#define CICLI_CALC 20 // cicli da attendere per effettuare i calcoli
byte cnt_calc = CICLI_CALC;
#define CICLI_HEART 100 // cicli da attendere per l'heartbeat
byte cnt_heart = CICLI_HEART;
byte heart = 0;
#define CICLI_COMM 50 // cicli da attendere per comunicare i dati
byte cnt_comm = CICLI_COMM;

//Variabili per le misure dell'accelerometro MPU-6050
byte ultimo_campione=0;
int16_t z_acc;
int32_t somma_z_acc = 0;
int16_t serie_z[CICLI_CALC];
int16_t media_z_acc;
int16_t y_acc;
int32_t somma_y_acc = 0;
int16_t serie_y[CICLI_CALC];
int16_t media_y_acc;
int16_t t_acc;

//Variabili per le misure della velocità delle ruote
int16_t velo_dx;
int32_t somma_velo_dx = 0;
int16_t serie_velo_dx[CICLI_CALC];
int16_t media_velo_dx;
int16_t velo_sx;
int32_t somma_velo_sx = 0;
int16_t serie_velo_sx[CICLI_CALC];
int16_t media_velo_sx;

//-- COSTANTI PER LA GESTIONE ACCELERAZIONE
//Valore di g misurato dal MPU6050 da calibrare a mano.
#define G_MPU6050 15320
//Valore di (g*g)
#define G2_MPU6050 234702400
//by,min: Valore minimo di componente frontale by per rilevare la pendenza della pedana
//Ottenuto con g * sen 15°
//TODO: Verificare se 15° vanno bene
#define BY_MINIMO 3965
//bz,margine: Valore oltre il quale la componente bz si considera verticale
//Ottenuto con g * cos 15°
//TODO: Verificare se 15° vanno bene
#define BZ_MARGINE 14798
//Valore del sensore analogico di velocità ad 1 m/s
//Ottenuto: VEL_1MPSEC = Vsens@1m/s (=2,906) * rapp.partizione 1/3 * 1023 / 5V
//NOTA: Vsens@1m/s può cambiare da controller a controller!
#define VEL_1MPSEC 198
//Valore del sensore analogico di velocità ad 0,4 m/s
//Ottenuto: VEL_04MPSEC = Vsens@0,4m/s * rapp.partizione 1/3 * 1023 / 5V
//NOTA: Vsens@0,4m/s può cambiare da controller a controller!
#define VEL_04MPSEC 79
//valore minimo del PWM chge ferma il motore
//Min Vthorttle : 1,4V
//Ottenuto: PWM_THROTTLE_MIN = 1,4V / 5V * 255
#define PWM_THROTTLE_MIN 70
//Valore massimo ammesso per il PWM Vthrottle per la marcia avanti
//Max Vthorttle : 3V (per ora. TODO: dimensionare)
//Ottenuto: PWM_THROTTLE_MAX = 3,1V / 5V * 255
#define PWM_THROTTLE_MAX 160
//Passi di incremento del throttle. 
//Dovrebbe raggiungere il massimo in circa 1/2 secondo
//TODO: da verificare in base al comportamento del motore ed alla durata di un ciclo loop
#define PWM_THROTTLE_STEP 1
//Passi di decremento del throttle: stimato 3*PWM_THROTTLE_STEP
#define PWM_THROTTLE_RED_STEP 3

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
  pinMode(STATUS_LED, OUTPUT);
  pinMode(PRESENZA, INPUT_PULLUP);
  pinMode(FUNC1, OUTPUT);
  pinMode(FUNC2, OUTPUT);

  //Setup dell'accelerometro
  setup_mpu6050();

  //Azzera i campioni da mediare
  byte i;
  for (i=0; i<CICLI_CALC; i++) {
    serie_z[i]=0;
    serie_y[i]=0;
    serie_velo_dx[i]=0;
    serie_velo_sx[i]=0;
  }

  //Il controller deve essere tenuto con throttle a zero
  //se l'hoverboard era stato spento in modo brusco
  analogWrite(PWM_DX, 0);
  analogWrite(PWM_SX, 0);
  delay(500);

  pwm_throttle = PWM_THROTTLE_MIN;
  
  
}

void loop() {
  // put your main code here, to run repeatedly:
  leggi_accelerometro();
  leggi_analogici();  
  leggi_nunchuck();
  leggi_presenza(); 
  calcola_medie();
  ultimo_campione = (ultimo_campione + 1) % CICLI_CALC; 
  ctrl_limiti_max();
  transiz_stato();
  applica_frenata_invers();
  applica_pwm();
  report_status();
  //leggi_comando();
  heartbeat();
  delay(RITARDO_MAIN);
  
}

/**
 * Legge i valori Z e Y e la temperatura dall'accelerometro via I2C
 * I valori sono memorizzati in z_acc, y_acc e t_acc
 * Si considera un accelerometro del tipo MPU-6050
 */
void leggi_accelerometro(void) 
{
  //Esegue la procedura di accesso all'MPU-6050
  leggi_mpu6050();
  
  //aggiorna i campioni delle accelerazioni
  somma_z_acc += (z_acc - serie_z[ultimo_campione]);
  somma_y_acc += (y_acc - serie_y[ultimo_campione]);
  serie_z[ultimo_campione] = z_acc;
  serie_y[ultimo_campione] = y_acc;    
}

/**
 * Legge i valori analogici:
 * A0: stato batteria -> livello_batt
 * A1: velocità ruota DX -> velo_dx
 * A2: velocità ruota SX -> velo_sx
 */
void leggi_analogici(void)
{
  livello_batt = analogRead(VALIM_ANALOG);
  
  velo_dx = analogRead(MISVEL_DX);
  velo_sx = analogRead(MISVEL_SX);
  //aggiorna le medie delle velocità
  somma_velo_dx += (velo_dx - serie_velo_dx[ultimo_campione]);
  somma_velo_sx += (velo_sx - serie_velo_sx[ultimo_campione]);
  serie_velo_dx[ultimo_campione] = velo_dx;
  serie_velo_sx[ultimo_campione] = velo_sx;
}

/**
 * Legge lo stato del nunchuck
 */
void leggi_nunchuck(void)
{
  
}


/**
 * Verifica lo stato del pulsante di presenza
 * Lo stato di presenza deve essere confermato per un numero di cicli = CONFERME_PRESENZA
 * per indicare la presenza effettiva sulla pedana (debounce).
 */
void leggi_presenza(void) 
{
  byte puls = digitalRead(PRESENZA);
  if (puls == HIGH) {
    if (cnt_presenza > 0) --cnt_presenza;
    else //cnt_presenza è 0
      presenza = false;      
  }
  else {
    if (cnt_presenza < CONFERME_PRESENZA) ++cnt_presenza;
    else //cnt_presenza è CONFERME_PRESENZA
      presenza = true;
  }
}


/**
 * Esegue le medie delle misure che devono essere mediate
 */
void calcola_medie(void)
{
  cnt_calc--;
  if (cnt_calc == 0) { 
    media_z_acc = somma_z_acc / CICLI_CALC;
    media_y_acc = somma_y_acc / CICLI_CALC;
  
    media_velo_dx = somma_velo_dx / CICLI_CALC;
    media_velo_sx = somma_velo_sx / CICLI_CALC;

    //Ripristina il contatore dei cicli
    cnt_calc = CICLI_CALC;
  }
}

/**
 * Esegue controlli di valori assoluti che richiedono 
 * intervento immediato al di fuori della transizione di stato:
 * 1. velocità eccessiva dovuto al ribaltamento dell'hoverboard
 */
void   ctrl_limiti_max() {

}

/**
 * Gestisce lo stato di funzionamento del programma e le sue transizioni.
 * Eseguito ogni CICLI_CALC iterazioni
 */
void transiz_stato(void) 
{    
  //Seleziona in base allo stato del programma
  switch(stato_prog) {
    //-- GESTIONE STATO DA FERMO
    case STATO_FERMO:
      if (media_z_acc < BZ_MARGINE) {
        if (media_y_acc > BY_MINIMO) {
          //La pedana è inclinata in avanti per avanzare
          frenata = LOW;
          inversione_sx = LOW;
          inversione_dx = LOW;
          stato_prog = STATO_ACCEL_AVANTI;
        }
        else if (media_y_acc < -BY_MINIMO) {
          //La pedana è inclinata indietro per retromarcia
          frenata = LOW;
          inversione_sx = HIGH;
          inversione_dx = HIGH;
          stato_prog = STATO_ACCEL_INDIETRO;
        }
      }
      break;

    //-- GESTIONE STATO IN ACCELERAZIONE AVANTI
    case STATO_ACCEL_AVANTI:
      if (media_z_acc > BZ_MARGINE) {
        //La pedana è tornata orizzontale          
        stato_prog = STATO_DECEL_AVANTI;
      } else {
        //Si è in fase di accelerazione
        if (((media_velo_dx + media_velo_sx) < 2 * VEL_1MPSEC) && (pwm_throttle < PWM_THROTTLE_MAX))
          //continua l'accelerazione
          pwm_throttle += PWM_THROTTLE_STEP;
        else
          //Raggiunto il massimo si passa al regime
          stato_prog = STATO_REGIME_AVANTI;
      }
      break;

    //-- GESTIONE STATO REGIME AVANTI
    case STATO_REGIME_AVANTI:
      if (media_z_acc > BZ_MARGINE) 
        //La pedana è tornata orizzontale
        stato_prog = STATO_DECEL_AVANTI;
      break;

    //-- GESTIONE STATO DECEL AVANTI
    case STATO_DECEL_AVANTI:
      if (media_z_acc > BZ_MARGINE) {
        //La pedana è ancora orizzontale: riduco la velocità
        if (pwm_throttle > PWM_THROTTLE_MIN)
          //continua la decelerazione
          pwm_throttle -= PWM_THROTTLE_RED_STEP;
        else {
          pwm_throttle=PWM_THROTTLE_MIN;
          frenata = HIGH;
          stato_prog = STATO_FERMO;
        }          
      } else {
        if (media_y_acc > BY_MINIMO) 
          //La pedana è tornata inclinata in avanti: riprendo l'accelerazione
          stato_prog = STATO_ACCEL_AVANTI;
        else if (media_y_acc < -BY_MINIMO) {
          //La pedana è inclinata indietro: fermo repentinamente per gestire successivamente la retromarcia
          pwm_throttle=PWM_THROTTLE_MIN;
          frenata = HIGH;
          stato_prog = STATO_FERMO;
        }
      }

    //-- GESTIONE STATO IN ACCELERAZIONE INDIETRO
    case STATO_ACCEL_INDIETRO:
      if (media_z_acc > BZ_MARGINE) {
        //La pedana è tornata orizzontale
        pwm_throttle=PWM_THROTTLE_MIN;
        frenata = HIGH;
        stato_prog = STATO_FERMO;
      } else {
        //Si è in fase di accelerazione all'indietro
        if (((media_velo_dx + media_velo_sx) < 2 * VEL_04MPSEC) && (pwm_throttle < PWM_THROTTLE_MAX))
          //continua l'accelerazione
          pwm_throttle += PWM_THROTTLE_STEP;
        else
          //Raggiunto il massimo si passa al regime
          stato_prog = STATO_REGIME_INDIETRO;
      }
      break;   

    //-- GESTIONE STATO REGIME INDIETRO
    case STATO_REGIME_INDIETRO:
      if (media_z_acc > BZ_MARGINE) {
        //La pedana è tornata orizzontale
        pwm_throttle=PWM_THROTTLE_MIN;
        frenata = HIGH;
        stato_prog = STATO_FERMO;
      }       
      break;             
  }
  
}

/**
 * Calcola ed applica i valori dei PWM di thorttle delle ruote
 * Il valore dipende da pwm_throttle e dalla lettura del nunchuck
 */
void applica_pwm(void)
{  
  //TODO: applicare le variazioni derivanti dal nunchuck
  //TODO: applicare la calobrazione analogica di compensazione DX/SX
  if (presenza) {
    analogWrite(PWM_DX, pwm_throttle);
    analogWrite(PWM_SX, pwm_throttle);
  }
  else {
    //Non aziona i motori se non è premuto il pulsante di presenza
    analogWrite(PWM_DX, 0);
    analogWrite(PWM_SX, 0);
    digitalWrite(FRENATA_DX, HIGH);
    digitalWrite(FRENATA_SX, HIGH);        
  }
}

/**
 * Applica il valore della frenata e di inversione ai controllers
 */
void applica_frenata_invers(void) 
{
  digitalWrite(FRENATA_DX, frenata);
  digitalWrite(FRENATA_SX, frenata);
  digitalWrite(INVERS_DX, inversione_dx);
  digitalWrite(INVERS_SX, inversione_sx);    
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
    Serial.print(F(":ZACC="));Serial.print(media_z_acc, DEC);
    Serial.print(F(";YACC="));Serial.print(media_y_acc, DEC);    
    Serial.print(F(":VLDX="));Serial.print(velo_dx, DEC);
    Serial.print(F(";VLSX="));Serial.print(velo_sx, DEC);
    Serial.print(F(";PWTH="));Serial.print(pwm_throttle, DEC);    
    Serial.print(F(";FREN="));Serial.print(frenata, DEC);  
    Serial.print(F(";BATT="));Serial.print(livello_batt, DEC);
    Serial.print(F(";TEMP="));Serial.print(t_acc, DEC);  
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
