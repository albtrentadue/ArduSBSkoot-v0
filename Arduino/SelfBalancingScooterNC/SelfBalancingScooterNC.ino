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
byte cnt_presenza = 0;
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

//Stati della macchina a stati del controllo discreto - V2
//Fare riferimento alla documentazione "Project.xls"
#define MV_S0_F 0b100
#define MV_S1_F 0b110
#define ST_S0_F 0b000
#define ST_S1_F 0b010
#define MV_S0_B 0b101
#define MV_S1_B 0b111
#define ST_S0_B 0b001
#define ST_S1_B 0b011

//Stato di funzionamento del programma
byte stato_prog = ST_S0_F;

//-- COSTANTI PER LA GESTIONE ACCELERAZIONE
//Valore di g misurato dal MPU6050 da calibrare a mano.
#define G_MPU6050 17800 //Misurato.
//Coefficienti di riduzione di precisione per divisione
#define PREC_REDUX_X 16
#define PREC_REDUX_Z 8
//bx,min: Valore minimo (ridotto) di componente frontale bx per attivare il moto
//TODO: Verificare sperimentalmente
#define BX_MINIMO 50
//bx,dir: Valore minimo (ridotto) di componente frontale bx per decidere la direzione
//TODO: Verificare sperimentalmente
#define BX_DIREZIONE 20
//bx,max: Valore massimo (ridotto) di componente frontale bx
//TODO: Verificare sperimentalmente
#define BX_MASSIMO 200
//Variabilità componente frontale bx
#define SPAN_BX (BX_MASSIMO - BX_MINIMO)
//Compensazione della pendenza di montaggio dell'accelerometro, per avere 0 secondo livella
#define BX_COMP_MONTAGGIO -45

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

//Costanti stato inclinazione pedana
#define PP0 0
#define PF1 1
#define PF2 2
#define PB1 3
#define PB2 4

//Cicli di inerzia per cambio di stato
#define INERZIA_STATO 3
//Contatore di inerzia per i cambi di stato
byte cnt_inerzia_stato = 0;

//valore minimo del PWM che ferma il motore
#define PWM_THROTTLE_MIN 115
//Variazione ampiezza throttle in marcia avanti
//TODO: da regolare
#define SPAN_THR_FWD 30
//Variazione ampiezza throttle in marcia indietro
//TODO: da regolare
#define SPAN_THR_BWD 25
//Setpoint velocità, indicizzati
int16_t throttles_max[] = {0, 0, SPAN_THR_FWD, SPAN_THR_BWD};

//Livello di throttle da applicare per il PWM
float ampiezza_thr_dx = 0.0;
float ampiezza_thr_sx = 0.0;
float ultima_mxamp_thr = 0.0;
//Valore effettivamente applicato al PWM
byte pwm_byte_dx = PWM_THROTTLE_MIN;
byte pwm_byte_sx = PWM_THROTTLE_MIN;

// Livelli analogici del partitore batteria
// TODO: Calibrare rispetto al valore massimo della batteria
#define BATT_OK 1000
#define BATT_MIN 800
int livello_batt = 0;

//Stati comando di inversione marcia. Attivo ALTO, pilota un NPN in saturazione
byte inversione_dx = LOW;
byte inversione_sx = LOW;
//Flag di attivazione della procedura di inversione
boolean inv_da_applicare_dx = false;
boolean inv_da_applicare_sx = false;

//Valore minimo del sensore analogico di velocità che considera il mezzo fermo
//TODO: da regolare
#define VEL_FERMO 10
//Valore massimo assoluto del livello analogico della velocità
#define VEL_ABS_MAX 800

//Variabili per le misure della velocità delle ruote
int32_t somma_velo_dx = 0;
int16_t serie_velo_dx[CICLI_MEDIE];
int16_t media_velo_dx;
int32_t somma_velo_sx = 0;
int16_t serie_velo_sx[CICLI_MEDIE];
int16_t media_velo_sx;
//Velocita media generale - in assenza di nunchuk
//int16_t media_velo;
//Velocità media al ciclo di controllo precedente
int16_t media_velo_prec_dx = 0;
int16_t media_velo_prec_sx = 0;
//Errore rispetto al setpoint
int16_t err_velo_dx = 0;
int16_t err_velo_sx = 0;
//Variabile di controllo: differenziale di velocità
int16_t delta_velo_dx = 0;
int16_t delta_velo_sx = 0;

#define CICLI_HEART 100 // cicli da attendere per l'heartbeat
byte cnt_heart = CICLI_HEART;
byte heart = 0;
#define CICLI_COMM 50 // cicli da attendere per comunicare i dati
byte cnt_comm = CICLI_COMM;

//Durata di un ciclo loop in msec
#define RITARDO_MAIN 2

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
    serie_velo_dx[i] = 0;
    serie_velo_sx[i] = 0;
  }

  //Stati iniziali switch controllo
  digitalWrite(FRENATA_DX, LOW);
  digitalWrite(FRENATA_SX, LOW);
  digitalWrite(INVERS_DX, LOW);
  digitalWrite(INVERS_SX, LOW);

  //Controlla il jumper di hoverboard capovolto a massa
  hb_capovolto = (digitalRead(JUMPER_CAPOVOLTO) == LOW);

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
  //delay(RITARDO_MAIN);
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
   A1: velocità ruota DX -> velo_dx
   A2: velocità ruota SX -> velo_sx
*/
void leggi_analogici(void)
{
  livello_batt = analogRead(VALIM_ANALOG);
  //Scartare la prima lettura nel cambio canale A/D
  analogRead(MISVEL_DX);
  int16_t velo_dx = analogRead(MISVEL_DX);
  somma_velo_dx += (velo_dx - serie_velo_dx[ultimo_campione]);
  serie_velo_dx[ultimo_campione] = velo_dx;
  //Scartare la prima lettura nel cambio canale A/D
  analogRead(MISVEL_SX);
  int16_t velo_sx = analogRead(MISVEL_SX);
  somma_velo_sx += (velo_sx - serie_velo_sx[ultimo_campione]);
  serie_velo_sx[ultimo_campione] = velo_sx;
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
  byte puls = digitalRead(PRESENZA);
  if (puls == HIGH) { //Entrambi pulsanti schiacciati (aperti)
    if (cnt_presenza > 0) --cnt_presenza;
    else //cnt_presenza è 0
      presenza = true;
  }
  else { //Almeno un pulsante sollevato (chiuso)
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

  media_velo_dx = somma_velo_dx / CICLI_MEDIE;
  media_velo_sx = somma_velo_sx / CICLI_MEDIE;

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
  if ( media_velo_dx >= VEL_ABS_MAX || media_velo_sx >= VEL_ABS_MAX )
    abort_completo();
}

/**
   Ritorna lo stato di inclinazione della pedana in base
   agli angoli misurati.
   Valori possibili:
   PP0: orizzontale
   PF1: Inclinata lievemente in avanti
   PB1: Inclinata lievemente all'indietro
   PF2: Inclinata in avanti
   PB2: Inclinata all'indietro
*/
byte stato_inclinazione(void) {
  if (media_x_acc <= -BX_MINIMO) return PB2;
  if (media_x_acc >= BX_MINIMO) return PF2;
  if (media_x_acc <= -BX_DIREZIONE) return PB1;
  if (media_x_acc >= BX_DIREZIONE) return PF1;

  return PP0;
}

/**
   Ritorna lo stato di velocità media tra le due ruote
   true: c'è movimento

   TODO: con l'introduzione del controllo direzione, deve essere rivisto!
*/
boolean stato_velo_ruote(void) {
  return ((media_velo_dx > VEL_FERMO) || (media_velo_sx > VEL_FERMO));
}

/**
   Gestisce lo stato di funzionamento del programma e le sue transizioni.
   Eseguito ogni CICLI_MEDIE iterazioni
*/
void transiz_stato(void)
{
  byte incl_pedana = stato_inclinazione();
  bool stato_velo_corr = stato_velo_ruote();
  byte nuovo_stato_prog = stato_prog;
  byte nuova_inversione_sx = inversione_sx;
  byte nuova_inversione_dx = inversione_dx;

  //Seleziona in base allo stato del programma
  switch (stato_prog) {
    case ST_S0_F:
      //Gestione dello sblocco dopo mancata presenza
      if (presenza) abilita_dopo_assenza = true;

      switch (incl_pedana) {
        case PF2:
          nuovo_stato_prog = ST_S1_F;
          break;
        case PB1:
        case PB2:
          nuovo_stato_prog = ST_S0_B;
          inv_da_applicare_dx = true;
          inv_da_applicare_sx = true;
          nuova_inversione_sx = HIGH;
          nuova_inversione_dx = HIGH;
      }
      break;
    //---------------------------------//
    case ST_S1_F:
      switch (incl_pedana) {
        case PP0:
        case PF1:
        case PB1:
        case PB2:
          nuovo_stato_prog = ST_S0_F;
          break;
        case PF2:
          if (stato_velo_corr)
            nuovo_stato_prog = MV_S1_F;
      }
      break;
    //---------------------------------//
    case MV_S0_F:
      switch (incl_pedana) {
        case PP0:
        case PF1:
        case PB1:
        case PB2:
          if (!stato_velo_corr)
            nuovo_stato_prog = ST_S0_F;
          break;
        case PF2:
          nuovo_stato_prog = MV_S1_F;
      }
      break;
    //---------------------------------//
    case MV_S1_F:
      switch (incl_pedana) {
        case PP0:
        case PF1:
        case PB1:
        case PB2:
          nuovo_stato_prog = MV_S0_F;
      }
      break;
    //---------------------------------//
    case ST_S0_B:
      //Gestione dello sblocco dopo mancata presenza
      if (presenza) abilita_dopo_assenza = true;

      switch (incl_pedana) {
        case PF1:
        case PF2:
          nuovo_stato_prog = ST_S0_F;
          inv_da_applicare_dx = true;
          inv_da_applicare_sx = true;
          nuova_inversione_sx = LOW;
          nuova_inversione_dx = LOW;
          break;
        case PB2:
          nuovo_stato_prog = ST_S1_B;
      }
      break;
    //---------------------------------//
    case ST_S1_B:
      switch (incl_pedana) {
        case PP0:
        case PF1:
        case PF2:
        case PB1:
          nuovo_stato_prog = ST_S0_B;
          break;
        case PB2:
          if (stato_velo_corr)
            nuovo_stato_prog = MV_S1_B;
      }
      break;
    //---------------------------------//
    case MV_S0_B:
      switch (incl_pedana) {
        case PP0:
        case PF1:
        case PF2:
        case PB1:
          if (!stato_velo_corr)
            nuovo_stato_prog = ST_S0_B;
          break;
        case PB2:
          nuovo_stato_prog = MV_S1_B;
      }
      break;
    //---------------------------------//
    case MV_S1_B:
      switch (incl_pedana) {
        case PP0:
        case PF1:
        case PF2:
        case PB1:
          nuovo_stato_prog = MV_S0_B;
      }
      break;
  }

  /*
    SENZA INERZIA
    stato_prog = nuovo_stato_prog;
    inversione_sx = nuova_inversione_sx;
    inversione_dx = nuova_inversione_dx;
    cnt_inerzia_stato = 0;
  */

  //INERZIA CAMBIO STATO
  if (nuovo_stato_prog != stato_prog) {
    //Se c'è possibilità di cambio, si controlla l'inerzia
    if (++cnt_inerzia_stato == INERZIA_STATO) {
      //Superata l'inerzia, si cambia stato
      stato_prog = nuovo_stato_prog;
      inversione_sx = nuova_inversione_sx;
      inversione_dx = nuova_inversione_dx;
      cnt_inerzia_stato = 0;
    } else {
      //NON cambia stato ed annulla eventuali inversioni
      inv_da_applicare_dx = false;
      inv_da_applicare_sx = false;
    }
  } else
    //resetta il conteggio di inerzia
    cnt_inerzia_stato = 0;
}

/**
   Calcola ed applica i valori dei PWM di thorttle delle ruote
   Il valore dipende da pwm_throttle e dalla lettura del nunchuck
*/
void applica_pwm(void)
{
  //TODO: applicare le variazioni derivanti dal nunchuck
  //TODO: applicare la calibrazione analogica di compensazione DX/SX

  //Approccio di controllo del moto: QUADRATICO RISPETTO ALLA PENDENZA
  byte idx = stato_prog & 0b11;
  /*
    if (throttles_max[idx]) { //Se diverso di zero
    int16_t ax_lim = min(abs(media_x_acc), BX_MASSIMO);
    float fatt_b = float(ax_lim - BX_MINIMO) / SPAN_BX;
    ampiezza_thr_dx = byte(map(ax_lim, BX_MINIMO, BX_MASSIMO, 0, throttles_max[idx]) * fatt_b);
    ampiezza_thr_sx = byte(map(ax_lim, BX_MINIMO, BX_MASSIMO, 0, throttles_max[idx]) * fatt_b);
    pwm_byte_dx = PWM_THROTTLE_MIN + ampiezza_thr_dx;
    pwm_byte_sx = PWM_THROTTLE_MIN + ampiezza_thr_sx;
    }
    else {
    //reset del grado di throttle
    ampiezza_thr_dx = 0;
    ampiezza_thr_sx = 0;
    pwm_byte_dx = 0;
    pwm_byte_sx = 0;
    }
  */
  if (throttles_max[idx] != 0) {
    ampiezza_thr_dx = constrain(ampiezza_thr_dx+0.1, 0, throttles_max[idx]);
    ampiezza_thr_sx = constrain(ampiezza_thr_sx+0.1, 0, throttles_max[idx]);
    ultima_mxamp_thr = throttles_max[idx];
  }
  else {
    ampiezza_thr_dx = constrain(ampiezza_thr_dx-0.075, -15, ultima_mxamp_thr);
    ampiezza_thr_sx = constrain(ampiezza_thr_sx-0.075, -15, ultima_mxamp_thr);
  }
  pwm_byte_dx = byte(PWM_THROTTLE_MIN + ampiezza_thr_dx);
  pwm_byte_sx = byte(PWM_THROTTLE_MIN + ampiezza_thr_sx);

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
    analogWrite(PWM_DX, 0);
    analogWrite(PWM_SX, 0);
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
    if (inv_da_applicare_dx) digitalWrite(INVERS_DX, inversione_dx);
    if (inv_da_applicare_sx) digitalWrite(INVERS_SX, inversione_sx);
    delay(100);
    inv_da_applicare_dx = false;
    inv_da_applicare_sx = false;
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
    Serial.print(F("S;")); Serial.print(stato_prog, BIN);
    Serial.print(F(";XM;")); Serial.print(media_x_acc, DEC);
    Serial.print(F(";Td;")); Serial.print(pwm_byte_dx, DEC);
    Serial.print(F(";Ts;")); Serial.print(pwm_byte_sx, DEC);
    Serial.print(F(";VD;")); Serial.print(media_velo_dx, DEC);
    Serial.print(F(";VS;")); Serial.print(media_velo_sx, DEC);    
    //Serial.print(F(";Id;")); Serial.print(inversione_dx, DEC);
    //Serial.print(F(";Is;")); Serial.print(inversione_sx, DEC);
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
