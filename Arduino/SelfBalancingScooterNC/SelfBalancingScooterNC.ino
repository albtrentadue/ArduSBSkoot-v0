/***
   DIY Self Balancing Scooter ATR
   Self balancing scooter controller sketch using one single rolling board
   and one single accelerometer sensor.
   It drives 2 brushless controllers for both speed and direction and is
   controlled by a Wii Nunchuck

   Prima versione controllo:
   - Velocità angolare proporzionale all'angolo di pendenza, limitata superiormente
   - Diverse costanti per marcia avanti ed indietro

   By Alberto Trentadue 2017-2018
*/

//#define PRESENZA_SEMPRE
//#define SENZA_INERZIA_STATO

//Per I2C
#include <Wire.h>

#define HALT_PROGRAM while(true) {}

#define STATUS_LED 13
#define VALIM_ANALOG A0

// Pin dei PWM di controllo brushless
#define PWM_DX 3
#define FRENATA_DX 2
#define INVERS_DX 4
#define MISVEL_DX A2
#define PWM_SX 6
#define FRENATA_SX 5
#define INVERS_SX 7
#define MISVEL_SX A1

//Gestione rivelatori di presenza
//Pin connesso ai rivelatori
#define PRESENZA 8
//Numero di test per convalidare la presenza
#define CONFERME_PRESENZA 10
byte cnt_presenza = CONFERME_PRESENZA;
bool presenza = false;
bool abilita_dopo_assenza = false;

//Non usati in v1.0
#define FUNC1 9
#define FUNC2 10

#define JUMPER_CAPOVOLTO 11
boolean hb_capovolto = false;

//Indirizzi I2C
#define ADDR_NUNCHUCK_W 0xA4
#define ADDR_NUNCHUCK_R 0xA5
#define ADDR_ACCEL 0x68 //0b1101000

//Stati della macchina a stati del controllo discreto
#define ST_P0_F 0b0000
#define ST_P1_F 0b0010
#define MV_P0_F 0b1000
#define MV_P1_F 0b1010
#define MV_PR_F 0b1100
#define ST_P0_B 0b0001
#define ST_P1_B 0b0011
#define MV_P0_B 0b1001
#define MV_P1_B 0b1011
#define MV_PR_B 0b1101

//Stato di funzionamento del programma
byte stato_prog = ST_P0_F;

//Cicli di inerzia per cambio di stato
#define INERZIA_STATO 8
//Contatore di inerzia per i cambi di stato
byte cnt_inerzia_stato = 0;


//-- COSTANTI PER LA GESTIONE ACCELEROMETRO
//Coefficienti di riduzione di precisione per divisione
#define PREC_REDUX_X 16
#define PREC_REDUX_Z 8
//bx,min: Valore minimo (ridotto) di componente frontale bx per attivare il moto
//TODO: Verificare sperimentalmente
#define BX_MINIMO 60
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
int16_t z_acc; //Globale: usata nel MPU-6050
int32_t somma_z_acc = 0;
int16_t serie_z[CICLI_MEDIE];
int16_t media_z_acc;
int16_t x_acc; //Globale: usata nel MPU-6050
int32_t somma_x_acc = 0;
int16_t serie_x[CICLI_MEDIE];
int16_t media_x_acc;
int16_t t_acc; //Globale: usata nel MPU-6050


//-- COSTANTI E VARIABILI GESTIONE DEL THROTTLE
//valore minimo del PWM che ferma il motore
#define PWM_THROTTLE_MIN 116
//Ampiezza throttle iniziale in marcia avanti
#define PWM_THROTTLE_MAX 250
//Ampiezza throttle iniziale in marcia avanti
//TODO: da regolare
#define THR_INIT_FWD 130
//Ampiezza throttle iniziale in marcia indietro
//TODO: da regolare
#define THR_INIT_BWD 130
//Set point velocità media in marcia avanti
#define VEL_REG_FWD 80
//Set point velocità media in marcia indietro
#define VEL_REG_BWD 60
//Setpoint velocità iniziali, indicizzati dai 2 LSB dello stato
int16_t throttles_init[] = {PWM_THROTTLE_MIN, PWM_THROTTLE_MIN, THR_INIT_FWD,THR_INIT_BWD};
//Setpoint velocità a regime, indicizzati dai 2 LSB dello stato
int vels_reg[] = {0, 0, VEL_REG_FWD, VEL_REG_BWD};
//Costante di controllo proporzionale throttle-errore velocità
#define THR_EV_KAPPA_P 0.003
//costante decremento lineare della decelerazione
#define DEC_THR_LIN 0.35
//Valore di throttle sotto il minimo per resettare la corrente
#define PWM_THROTTLE_ZEROCURR 100
//Bilanciamento throttle ruota SINISTRA (delta rispetto alla destra)
#define PWM_THROTTLE_SX_DELTA 0
//Livello di throttle da applicare per il PWM
float throttle_dx = 0.0;
float throttle_sx = 0.0;
//Valore effettivamente applicato al PWM
byte pwm_byte_dx = PWM_THROTTLE_MIN;
byte pwm_byte_sx = PWM_THROTTLE_MIN;

//Stati comando di inversione marcia. Attivo ALTO, pilota un NPN in saturazione
byte retromarcia_dx = LOW;
byte retromarcia_sx = LOW;
//Flag di attivazione della procedura di inversione
boolean inv_da_applicare_dx = false;
boolean inv_da_applicare_sx = false;

//COSTANTI E VARIABILI DELL'ESTIMATORE DI VELOCITA'
//Tracciaori della fase del sensore di Hall
byte stato_riv_vel_dx = LOW;
byte stato_riv_vel_sx = LOW;
//Contatori di stima del periodo di rotazione
int cnt_periodo_dx = 0;
int cnt_periodo_sx = 0;
//Numero massimo di conteggi oltre il quale la ruota si considera quasi ferma
#define MAX_PERIODO_RIV 120
//Velocità angolari stimate delle ruote
int vel_dx = 0;
int vel_sx = 0;
int vel_media = 0;
//Valore minimo della vel_media che considera il mezzo fermo
//TODO: da regolare
#define VEL_FERMO 2

// Livelli analogici del partitore batteria
// TODO: Calibrare rispetto al valore massimo della batteria
#define BATT_OK 1000
#define BATT_MIN 800
int livello_batt = 0;

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
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PWM_DX, OUTPUT);
  pinMode(PWM_SX, OUTPUT);
  pinMode(MISVEL_DX, INPUT);
  pinMode(MISVEL_SX, INPUT);  
  pinMode(FRENATA_DX, OUTPUT);
  pinMode(FRENATA_SX, OUTPUT);
  pinMode(INVERS_DX, OUTPUT);
  pinMode(INVERS_SX, OUTPUT);
  pinMode(STATUS_LED, OUTPUT);
  pinMode(PRESENZA, INPUT_PULLUP);
  pinMode(FUNC1, OUTPUT);
  pinMode(FUNC2, OUTPUT);
  pinMode(JUMPER_CAPOVOLTO, INPUT_PULLUP);

  //Setup dell'accelerometro
  setup_mpu6050();

  //Azzera i campioni da mediare
  byte i;
  for (i = 0; i < CICLI_MEDIE; i++) {
    serie_z[i] = 0;
    serie_x[i] = 0;
  }

  //Stati iniziali switch controllo
  digitalWrite(FRENATA_DX, LOW);
  digitalWrite(FRENATA_SX, LOW);
  digitalWrite(INVERS_DX, LOW);
  digitalWrite(INVERS_SX, LOW);

  //Controlla il jumper di hoverboard capovolto a massa
  hb_capovolto = (digitalRead(JUMPER_CAPOVOLTO) == LOW);

  //Inizializzazione dei tracciatori della fase dei sensori di Hall
  stato_riv_vel_dx = digitalRead(MISVEL_DX);
  stato_riv_vel_sx = digitalRead(MISVEL_SX);  

  //Il controller deve essere tenuto con throttle a zero
  //se l'hoverboard era stato spento in modo brusco
  analogWrite(PWM_DX, 0);
  analogWrite(PWM_SX, 0);
  delay(500);

}

void loop() {
  // put your main code here, to run repeatedly:
  leggi_accelerometro();
  leggi_analogici();
  stima_velocita();
  leggi_nunchuck();
  leggi_presenza();
  calcola_medie();
  ultimo_campione = (ultimo_campione + 1) % CICLI_MEDIE;
  transiz_stato();
  ctrl_limiti_max();
  applica_inversione();
  applica_pwm();
  report_status();
  //leggi_comando();
  heartbeat();
}

/**
   Legge i valori Z e X e la temperatura dall'accelerometro via I2C
   I valori sono memorizzati in z_acc, x_acc e t_acc
   Si considera un accelerometro del tipo MPU-6050
*/
void leggi_accelerometro(void)
{
  //Esegue la procedura di accesso all'MPU-6050
  leggi_mpu6050();
  //Riduzione di precisione
  z_acc /= PREC_REDUX_Z;
  x_acc /= PREC_REDUX_X;
  //Compensazione angolo di montaggio X scheda accelerometro
  x_acc += BX_COMP_MONTAGGIO;

  //aggiorna i campioni delle accelerazioni
  if (hb_capovolto) {
    //Inversione dei valori se l'hoverboard è capovolto (in test)
    somma_z_acc += (z_acc - serie_z[ultimo_campione]);
    somma_x_acc += (x_acc - serie_x[ultimo_campione]);
    serie_z[ultimo_campione] = z_acc;
    serie_x[ultimo_campione] = x_acc;
  }
  else {
    //z_acc e x_acc sono al negativo per come è stato montato l'MPU6050
    somma_z_acc += (-z_acc - serie_z[ultimo_campione]);
    somma_x_acc += (-x_acc - serie_x[ultimo_campione]);
    serie_z[ultimo_campione] = -z_acc;
    serie_x[ultimo_campione] = -x_acc;
  }
}

/**
   Legge i valori analogici:
   A0: stato batteria -> livello_batt
*/
void leggi_analogici(void)
{
  livello_batt = analogRead(VALIM_ANALOG);
}

/**
 * Stima la velocità delle ruote dal semi-periodo 
 * delle onde quadre dei rivelatori di Hall. 
 * Assume che le onde abbiano un duty cycle circa al 50%
 */
void stima_velocita(void) 
{
  byte x;
  //Transizione ad ogni cambio di livello, si assume duty al 50%
  //Se non c'è transizione si conteggia fino al limite massimo o si assume velocità zero
  //Se c'è transizione, si termina il conteggio e si calcola la velocità angolare
  
  //DESTRA
  x = digitalRead(MISVEL_DX);
  if (stato_riv_vel_dx == x)
    //No transizione      
    if (cnt_periodo_dx < MAX_PERIODO_RIV) cnt_periodo_dx++;
    else vel_dx = 0;          
  else {    
    //Transizione
    vel_dx = (MAX_PERIODO_RIV * 10 / cnt_periodo_dx);
    cnt_periodo_dx = 0;
  }
  stato_riv_vel_dx = x;
 
  //SINISTRA
  x = digitalRead(MISVEL_SX);
  if (stato_riv_vel_sx == x)
    //No transizione 
    if (cnt_periodo_sx < MAX_PERIODO_RIV) cnt_periodo_sx++;
    else vel_sx = 0;          
  else {
    //Transizione
    vel_sx = (MAX_PERIODO_RIV * 10 / cnt_periodo_sx);
    cnt_periodo_sx = 0;
  }
  stato_riv_vel_sx = x;

  //stima della velocità media complessiva.  
  vel_media = (vel_dx + vel_sx) / 2;
             
}

/**
   Legge lo stato del nunchuck
*/
void leggi_nunchuck(void)
{
  //TODO
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
    abilita_dopo_assenza = false;
  }
}


/**
   Esegue le medie delle misure che devono essere mediate
*/
void calcola_medie(void)
{
  media_z_acc = somma_z_acc / CICLI_MEDIE;
  media_x_acc = somma_x_acc / CICLI_MEDIE;
}

/**
   Ferma e frena le ruote e blocca l'esecuzione del programma
   definitivamente.
   Da usare solo in caso di intervento urgente.
*/
void abort_completo(void) {
  Serial.println(F("Abort completo"));
  applica_frenata(HIGH);
  analogWrite(PWM_DX, 0);
  analogWrite(PWM_SX, 0);
  digitalWrite(LED_BUILTIN, HIGH);
  //Blocco del programma. Reset necessario.
  HALT_PROGRAM;
}

/**
   Esegue controlli di valori assoluti che richiedono
   intervento immediato al di fuori della transizione di stato:
   1. velocità eccessiva dovuto al ribaltamento dell'hoverboard
*/
void ctrl_limiti_max() {
  //TODO
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
   Gestisce lo stato di funzionamento del programma e le sue transizioni.
   Eseguito ogni CICLI_MEDIE iterazioni
*/
void transiz_stato(void)
{
  byte incl_pedana = stato_inclinazione();
  byte nuovo_stato_prog = stato_prog;
  byte nuova_retromarcia_sx = retromarcia_sx;
  byte nuova_retromarcia_dx = retromarcia_dx;

  //Seleziona in base allo stato del programma
  switch (stato_prog) {
    case ST_P0_F: //0000
      //Gestione dello sblocco dopo mancata presenza
      if (presenza) abilita_dopo_assenza = true;
      switch (incl_pedana) {
        case PEND_AV:
          nuovo_stato_prog = ST_P1_F;
          break;        
        case PEND_IND:
          nuovo_stato_prog = ST_P0_B;
          inv_da_applicare_dx = true;
          inv_da_applicare_sx = true;
          nuova_retromarcia_sx = HIGH;
          nuova_retromarcia_dx = HIGH;
      }
      break;
    //---------------------------------//
    case ST_P1_F: //0010
      switch (incl_pedana) {
        case PP0:
        case PEND_IND:
          nuovo_stato_prog = ST_P0_F;
          break;
        case PEND_AV:
          if (vel_media > 0)
            nuovo_stato_prog = MV_P1_F;
      }
      break;
    //---------------------------------//
    case MV_P0_F: //1000      
      switch (incl_pedana) {
        case PP0:        
          if (vel_media <= VEL_FERMO)
            nuovo_stato_prog = ST_P0_F;
          break;
        case PEND_IND:
          nuovo_stato_prog = MV_PR_F;
          break;
        case PEND_AV:
          nuovo_stato_prog = MV_P1_F;
      }
      break;
    //---------------------------------//
    case MV_P1_F: //1010
      switch (incl_pedana) {
        case PEND_AV:
          if (vel_media <= VEL_FERMO)
            nuovo_stato_prog = ST_P1_F;
          break;
        case PP0:
          nuovo_stato_prog = MV_P0_F;
          break;
        case PEND_IND:
          nuovo_stato_prog = MV_PR_F;
          break;
      }
      break;
    //---------------------------------//
    case MV_PR_F: //1100
      switch (incl_pedana) {
        case PEND_AV:          
          nuovo_stato_prog = MV_P1_F;
          break;
        case PP0:
          nuovo_stato_prog = MV_P0_F;
          break;
        case PEND_IND:
          if (vel_media <= VEL_FERMO)
            nuovo_stato_prog = ST_P0_F;
          break;
      }
      break;      
    //---------------------------------//
    case ST_P0_B: //0001
      //Gestione dello sblocco dopo mancata presenza
      if (presenza) abilita_dopo_assenza = true;
      switch (incl_pedana) {     
        case PEND_AV:
          nuovo_stato_prog = ST_P0_F;
          inv_da_applicare_dx = true;
          inv_da_applicare_sx = true;
          nuova_retromarcia_sx = LOW;
          nuova_retromarcia_dx = LOW;
          break;
        case PEND_IND:
          nuovo_stato_prog = ST_P1_B;
      }
      break;
    //---------------------------------//
    case ST_P1_B: //0011
      switch (incl_pedana) {
        case PP0:        
        case PEND_AV:        
          nuovo_stato_prog = ST_P0_B;
          break;
        case PEND_IND:
          if (vel_media > 0)
            nuovo_stato_prog = MV_P1_B;
      }
      break;
    //---------------------------------//
    case MV_P0_B: //1001
      switch (incl_pedana) {
        case PP0:        
          if (vel_media <= VEL_FERMO)
            nuovo_stato_prog = ST_P0_B;
          break;
        case PEND_AV:
          nuovo_stato_prog = MV_PR_B;          
          break;
        case PEND_IND:
          nuovo_stato_prog = MV_P1_B;          
      }
      break;
    //---------------------------------//
    case MV_P1_B: //1011
      switch (incl_pedana) {
        case PEND_IND:  
          if (vel_media <= VEL_FERMO)
            nuovo_stato_prog = ST_P1_B;
          break;        
        case PP0:        
          nuovo_stato_prog = MV_P0_B;
          break;        
        case PEND_AV:        
          nuovo_stato_prog = MV_PR_B;
          break;        
      }
      break;
    //---------------------------------//
    case MV_PR_B: //1101
      switch (incl_pedana) {
        case PEND_IND:  
          nuovo_stato_prog = MV_P1_B;
          break;        
        case PP0:        
          nuovo_stato_prog = MV_P0_B;
          break;        
        case PEND_AV:
          if (vel_media <= VEL_FERMO)        
            nuovo_stato_prog = ST_P0_B;
          break;        
      }
      break;      
  }

#ifdef SENZA_INERZIA_STATO
  //SENZA INERZIA
  stato_prog = nuovo_stato_prog;
  retromarcia_sx = nuova_retromarcia_sx;
  retromarcia_dx = nuova_retromarcia_dx;
  cnt_inerzia_stato = 0;
#else
  //INERZIA CAMBIO STATO
  if (nuovo_stato_prog != stato_prog) {
    //Se c'è possibilità di cambio, si controlla l'inerzia
    if (++cnt_inerzia_stato == INERZIA_STATO) {
      //Superata l'inerzia, si cambia stato
      stato_prog = nuovo_stato_prog;
      retromarcia_sx = nuova_retromarcia_sx;
      retromarcia_dx = nuova_retromarcia_dx;
      cnt_inerzia_stato = 0;
    } else {
      //NON cambia stato ed annulla eventuali inversioni
      inv_da_applicare_dx = false;
      inv_da_applicare_sx = false;
    }
  } else
    //resetta il conteggio di inerzia
    cnt_inerzia_stato = 0;
#endif

}
 

/**
   Applica la procedura di inversione di marcia ai controllers:
   - Stop per un certo tempo
   - Applicazione del livello logico
*/
void applica_inversione(void)
{
  if (inv_da_applicare_dx || inv_da_applicare_sx) {
    Serial.println(F("CAMBIO DIREZIONE"));
    if (inv_da_applicare_dx) analogWrite(PWM_DX, 0);
    if (inv_da_applicare_sx) analogWrite(PWM_SX, 0);
    delay(100);
    if (inv_da_applicare_dx) digitalWrite(INVERS_DX, retromarcia_dx);
    if (inv_da_applicare_sx) digitalWrite(INVERS_SX, retromarcia_sx);
    delay(100);
    if (inv_da_applicare_dx) analogWrite(PWM_DX, PWM_THROTTLE_MIN);
    if (inv_da_applicare_sx) analogWrite(PWM_SX, PWM_THROTTLE_MIN);
    inv_da_applicare_dx = false;
    inv_da_applicare_sx = false;    
  }
}

/**
   Calcola ed applica i valori dei PWM di thorttle delle ruote
   Il valore dipende da pwm_throttle e dalla lettura del nunchuck
*/
void applica_pwm(void)
{
  //TODO: applicare le variazioni derivanti dal nunchuck
  //TODO: applicare la calibrazione analogica di compensazione DX/SX

  byte idx = stato_prog & 0b11; 
  float dt = 0.0;
  int vtarget;

  switch (stato_prog) {    
    case ST_P0_F: //0000
    case ST_P0_B: //0001
    case ST_P1_F: //0010  
    case ST_P1_B: //0011
      //Reset alla condizione iniziale al passaggio di stato da fermo a in moto
      throttle_dx = throttles_init[idx];
      throttle_sx = throttles_init[idx];
      break;
  
    case MV_P1_F: //1010
    case MV_P1_B: //1011
      //Profilo controllato in velocità
      //Controllo Prop-ONLY della velocità
      vtarget = vels_reg[idx] + abs(media_x_acc / 10);
      dt = (vtarget - vel_media) * THR_EV_KAPPA_P;      
      throttle_dx = constrain(throttle_dx+dt, 0.0, PWM_THROTTLE_MAX);
      throttle_sx = throttle_dx;  //Per ora
      break;
      
    case MV_PR_F: //1100
    case MV_PR_B: //1101
      //Profilo throttle in decelerazione lineare
      if (vel_media > VEL_FERMO && throttle_dx > PWM_THROTTLE_ZEROCURR)
        throttle_dx = throttle_dx-DEC_THR_LIN;
      else
        throttle_dx = PWM_THROTTLE_ZEROCURR;
      throttle_sx = throttle_dx;  //Per ora
      break;

    case MV_P0_F: //1000
    case MV_P0_B: //1001
      //Profilo throttle in decelerazione lineare più lieve
      throttle_dx = vel_media > VEL_FERMO ? throttle_dx-(DEC_THR_LIN / 2) : PWM_THROTTLE_ZEROCURR;
      throttle_sx = throttle_dx;  //Per ora
      break;
  }
  
  pwm_byte_dx = byte(throttle_dx);
  pwm_byte_sx = byte(throttle_sx)+ PWM_THROTTLE_SX_DELTA ;
    
#ifndef PRESENZA_SEMPRE
  //Non aziona i motori se:
  //1. I pulsanti di presenza non sono entrambi schiacciati
  //2. La pedana non si trova in posizione piana
  if (presenza && abilita_dopo_assenza) {
#else
  if (true) {
#endif
    analogWrite(PWM_DX, pwm_byte_dx);
    analogWrite(PWM_SX, pwm_byte_sx);
  }
  else {
    analogWrite(PWM_DX, PWM_THROTTLE_ZEROCURR);
    analogWrite(PWM_SX, PWM_THROTTLE_ZEROCURR);
  }
}

/**
   Applica i valori di frenata ai controllers
*/
void applica_frenata(byte frenata)
{
  digitalWrite(FRENATA_DX, frenata);
  digitalWrite(FRENATA_SX, frenata);
}

/**
   Scrive lo stato del programma sulla seriale
   Eseguito ogni CICLI_REPORT iterazioni
*/
void report_status(void)
{
  cnt_comm--;
  if (cnt_comm == 0) {
    Serial.print(F("S;")); Serial.print(stato_prog, BIN);
    Serial.print(F(";XM;")); Serial.print(media_x_acc, DEC);
    Serial.print(F(";Td;")); Serial.print(pwm_byte_dx, DEC);
    Serial.print(F(";Ts;")); Serial.print(pwm_byte_sx, DEC);
    Serial.print(F(";VD;")); Serial.print(vel_dx, DEC);
    Serial.print(F(";VS;")); Serial.print(vel_sx, DEC);            
    //Serial.print(F(";Id;")); Serial.print(retromarcia_dx, DEC);
    //Serial.print(F(";Is;")); Serial.print(retromarcia_sx, DEC);
    //Serial.print(F(";B;")); Serial.print(livello_batt, DEC);
    //Serial.print(F(";C;")); Serial.print(t_acc, DEC);
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
