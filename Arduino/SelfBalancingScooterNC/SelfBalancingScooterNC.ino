/***
 * DIY Self Balancing Scooter ATR
 * Self balancing scooter controller sketch using one single rolling board
 * and one single accelerometer sensor.
 * It drives 2 brushless controllers for both speed and direction and is 
 * controlled by a Wii Nunchuck
 * 
 * Prima versione controllo:
 * - 2 possibili pendenze definiscono 2 set point in avanti
 * - 1 solo set-point per la marcia all'indietro
 * - Grandezza controllata: tensione del throttle (Vt), considerando la velocità proporzionale a Vt
 * - Controreazione sul throttle: gli incrementi di Vt sono propoprzionali all'errore Vset - Vt 
 * 
 * By Alberto Trentadue 2017-2018
 */

//Per I2C
#include <Wire.h>

#define HALT_PROGRAM while(true) {}

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

//Stati della macchina a stati del controllo discreto
//Fare riferimento alla documentazione "Project.xls"
#define F_LV0_PW0 0b000000
#define F_LV0_PW1 0b000001
#define F_LV0_PW2 0b000010
#define F_LV1_PW0 0b000100
#define F_LV1_PW1 0b000101
#define F_LV1_PW2 0b000110
#define F_LV2_PW0 0b001000
#define F_LV2_PW1 0b001001
#define F_LV2_PW2 0b001010
#define F_LVX_PW0 0b010000
#define F_LVX_PW1 0b010001
#define F_LVX_PW2 0b010010
#define B_LV0_PW0 0b100000
#define B_LV0_PW1 0b100001
#define B_LV1_PW0 0b100100
#define B_LV1_PW1 0b100101
#define B_LVX_PW0 0b110000
#define B_LVX_PW1 0b110001
//Stato di funzionamento del programma
byte stato_prog = F_LV0_PW0;

//-- COSTANTI PER LA GESTIONE ACCELERAZIONE
//Valore di g misurato dal MPU6050 da calibrare a mano.
#define G_MPU6050 17800 //Misurato.
//bx,min: Valore minimo di componente frontale bx per rilevare la pendenza della pedana
//TODO: Verificare sperimentalmente
#define BX_MINIMO 1700
//bx,veloce: Valore oltre il quale l'asse è più inclinata
//TODO: Verificare sperimentalmente
#define BX_VELOCE 2800
//bz,margine: Valore oltre il quale la componente bz si considera verticale
//TODO: Verificare sperimentalmente
#define BZ_MARGINE 17700
//Valore limite del sensore analogico al livello velocità 2 
//Ottenuto: VEL_LIM2 = Vsens,lim2 * rapp.partizione 1/3 * 1023 / 5V
//TODO: da regolare
#define VEL_LIM2 198
//Valore limite del sensore analogico al livello velocità 1
//Ottenuto: VEL_LIM1 = Vsens,lim1 * rapp.partizione 1/3 * 1023 / 5V
//TODO: da regolare
#define VEL_LIM1 79
//Valore minimo del sensore analogico di velocità che considera il mezzo fermo
//TODO: da regolare
#define VEL_FERMO 8
//Valore massimo assoluto del livello analogico della velocità
#define VEL_ABS_MAX 300

//Costanti livello di velocità
#define LV0 0b000
#define LV1 0b001
#define LV2 0b010
#define LVX 0b100

//Costanti inclinazione pedana
#define PF0 0
#define PF1 1
#define PF2 2
#define PFB 255

// Livelli analogici del partitore batteria
// TODO: Calibrare rispetto al valore massimo della batteria
#define BATT_OK 1000
#define BATT_MIN 800
int livello_batt = 0;

//valore minimo del PWM che ferma il motore
//Min Vthorttle : 1,4V
//Ottenuto: PWM_THROTTLE_MIN = 1,4V / 5V * 255
#define PWM_THROTTLE_MIN 20.0
//Valore massimo ammesso per il PWM Vthrottle per la marcia avanti
//Max Vthorttle : 3V (per ora. TODO: dimensionare)
//Ottenuto: PWM_THROTTLE_MAX = 3,1V / 5V * 255
#define PWM_THROTTLE_MAX 160.0
//Valore del throttle per l'accelerazione moderata
#define PWM_THROTTLE_MODERATE 100.0
//Valore del throttle per l'accelerazione rapida
#define PWM_THROTTLE_FAST 130.0
//Moltiplicatore del feedback per il throttle: THR[i+1] = THR[i] + THROTTLE_GAIN * err(THR[i])
#define THROTTLE_GAIN 0.05
//Livello di throttle asintotico - via PWM
float pwm_throttle[] = {PWM_THROTTLE_MIN, PWM_THROTTLE_MODERATE, PWM_THROTTLE_FAST};
//Livello di throttle istantaneo - via PWM
float pwm_throttle_istantaneo = PWM_THROTTLE_MIN;

//Stato comando di frenata. Attivo ALTO, pilota un NPN in saturazione
byte frenata = LOW;
//Stati comando di inversione marcia. Attivo ALTO, pilota un NPN in saturazione
byte inversione_dx = LOW;
byte inversione_sx = LOW;
//Flag di attivazione della procedura di inversione
boolean inv_da_applicare_dx = false;
boolean inv_da_applicare_sx = false;
//Cicli di inerzia per cambio di stato
#define INERZIA_STATO 3
//Contatore di inerzia per i cambi di stato
byte cnt_inerzia_stato = 0;

//Durata di un ciclo loop in msec
#define RITARDO_MAIN 4

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
int16_t x_acc;
int32_t somma_x_acc = 0;
int16_t serie_x[CICLI_CALC];
int16_t media_x_acc;
int16_t t_acc;

//Variabili per le misure della velocità delle ruote
int32_t somma_velo_dx = 0;
int16_t serie_velo_dx[CICLI_CALC];
int16_t media_velo_dx;
int32_t somma_velo_sx = 0;
int16_t serie_velo_sx[CICLI_CALC];
int16_t media_velo_sx;

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
    serie_x[i]=0;
    serie_velo_dx[i]=0;
    serie_velo_sx[i]=0;
  }

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
  ultimo_campione = (ultimo_campione + 1) % CICLI_CALC;   
  transiz_stato();
  ctrl_limiti_max();
  applica_frenata();
  applica_inversione();
  applica_pwm();
  report_status();
  //leggi_comando();
  heartbeat();
  delay(RITARDO_MAIN);
}

/**
 * Legge i valori Z e X e la temperatura dall'accelerometro via I2C
 * I valori sono memorizzati in z_acc, x_acc e t_acc
 * Si considera un accelerometro del tipo MPU-6050
 */
void leggi_accelerometro(void) 
{
  //Esegue la procedura di accesso all'MPU-6050
  leggi_mpu6050();
  
  //aggiorna i campioni delle accelerazioni
  //z_acc e x_acc sono al negativo per come è stato montato l'MPU6050
  somma_z_acc += (-z_acc - serie_z[ultimo_campione]);
  somma_x_acc += (-x_acc - serie_x[ultimo_campione]);
  serie_z[ultimo_campione] = -z_acc;
  serie_x[ultimo_campione] = -x_acc;    
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
  //Ritardo MUX
  delayMicroseconds(500);
  
  //int16_t velo_dx=0;  
  int16_t velo_dx = analogRead(MISVEL_DX);
  somma_velo_dx += (velo_dx - serie_velo_dx[ultimo_campione]);
  serie_velo_dx[ultimo_campione] = velo_dx;
  //Ritardo MUX
  delayMicroseconds(500);
  //int16_t velo_sx=0;
  int16_t velo_sx = analogRead(MISVEL_SX);  
  somma_velo_sx += (velo_sx - serie_velo_sx[ultimo_campione]);
  serie_velo_sx[ultimo_campione] = velo_sx;
}

/**
 * Legge lo stato del nunchuck
 */
void leggi_nunchuck(void)
{
  //TODO
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
    media_x_acc = somma_x_acc / CICLI_CALC;
  
    media_velo_dx = somma_velo_dx / CICLI_CALC;
    media_velo_sx = somma_velo_sx / CICLI_CALC;

    //Ripristina il contatore dei cicli
    cnt_calc = CICLI_CALC;
  }
}

/**
 * Ferma e frena le ruote e blocca l'esecuzione del programma
 * definitivamente. 
 * Da usare solo in caso di intervento urgente.
 */
void abort_completo(void) {
    digitalWrite(FRENATA_DX, HIGH);
    digitalWrite(FRENATA_SX, HIGH);
    analogWrite(PWM_DX, 0);
    analogWrite(PWM_SX, 0);
    digitalWrite(LED_BUILTIN, HIGH);
    //Blocco del programma. Reset necessario.
    HALT_PROGRAM;
}

/**
 * Esegue controlli di valori assoluti che richiedono 
 * intervento immediato al di fuori della transizione di stato:
 * 1. velocità eccessiva dovuto al ribaltamento dell'hoverboard
 */
void ctrl_limiti_max() {
  if ( media_velo_dx >= VEL_ABS_MAX || media_velo_sx >= VEL_ABS_MAX )
    abort_completo();    
}

/**
 * Ritorna lo stato di inclinazione della pedana in base
 * agli angoli misurati.
 * Valori possibili:
 * PF0: orizzontale
 * PF1: Poco inclinata
 * PF2: Inclinata
 * PFB: Inclinata all'indietro
 */
byte inclinazione_pedana(void) {
  if (media_z_acc <= BZ_MARGINE) {
    if (media_x_acc <= -BX_MINIMO) return PFB;
    if (media_x_acc >= BX_VELOCE) return PF2;
    if (media_x_acc >= BX_MINIMO) return PF1;
  }
  return PF0;
}

/**
 * Ritorna lo stato di velocità media tra le due ruote
 */
byte stato_velo_ruote(void) {
  int16_t mv = media_velo_dx + media_velo_sx;
  if (mv < (2 * VEL_FERMO)) return LV0;
  if (mv < (2 * VEL_LIM1)) return LV1;
  if (mv < (2 * VEL_LIM2)) {
    if (pwm_throttle_istantaneo == PWM_THROTTLE_FAST) return LV2;
  }
  return LVX;
}
/**
 * Gestisce lo stato di funzionamento del programma e le sue transizioni.
 * Eseguito ogni CICLI_CALC iterazioni
 */
void transiz_stato(void) 
{ 
  byte incl_pedana = inclinazione_pedana();  
  byte stato_velo_corr = stato_velo_ruote();
  byte nuovo_stato_prog = stato_prog;
  byte nuova_inversione_sx = inversione_sx;
  byte nuova_inversione_dx = inversione_dx;
     
  //Seleziona in base allo stato del programma
  switch(stato_prog) {
    case F_LV0_PW0:
      switch (incl_pedana) {
        case PF1: 
          nuovo_stato_prog = F_LV0_PW1;          
          break;
        case PF2:
          nuovo_stato_prog = F_LV0_PW2;          
          break;
        case PFB:
          nuovo_stato_prog = B_LV0_PW0;
          inv_da_applicare_dx = true;
          inv_da_applicare_sx = true;
          nuova_inversione_sx = HIGH;
          nuova_inversione_dx = HIGH;
      }
      break;
    //---------------------------------//
    case F_LV0_PW1:
      switch (incl_pedana) {
        case PF0: 
        case PFB:
          nuovo_stato_prog = F_LV0_PW0;          
          break;
        case PF1:
          if (stato_velo_corr != LV0)
              nuovo_stato_prog = F_LV1_PW1;
          break;
        case PF2:
          nuovo_stato_prog = F_LV0_PW2;                
      }
      break;
    //---------------------------------//      
    case F_LV0_PW2:
      switch (incl_pedana) {
        case PF0: 
        case PFB:
          nuovo_stato_prog = F_LV0_PW0;          
          break;
        case PF1:
          nuovo_stato_prog = F_LV0_PW1;          
          break;
        case PF2:
          if (stato_velo_corr != LV0)
            nuovo_stato_prog = F_LV1_PW2;
      }
      break;
    //---------------------------------//
    case F_LV1_PW0:
      switch (incl_pedana) {
        case PF0: 
        case PFB:
          if (stato_velo_corr == LV0) 
            nuovo_stato_prog = F_LV0_PW0;
          break;
        case PF1:
          nuovo_stato_prog = F_LV1_PW1;          
          break;
        case PF2:                
          nuovo_stato_prog = F_LV1_PW2;                 
      }
      break;
    //---------------------------------//
    case F_LV1_PW1:
      switch (incl_pedana) {
        case PF0: 
        case PFB:
          nuovo_stato_prog = F_LV1_PW0;          
          break;
        case PF1:
          if (stato_velo_corr == LVX)
            nuovo_stato_prog = F_LVX_PW1;
          break;
        case PF2:                
          nuovo_stato_prog = F_LV1_PW2;          
      }
      break;
    //---------------------------------//
    case F_LV1_PW2:
      switch (incl_pedana) {
        case PF0:
        case PF1: 
        case PFB:
          nuovo_stato_prog = F_LV1_PW1;          
          break;
        case PF2:                
          if (stato_velo_corr > LV1)
            nuovo_stato_prog = F_LV2_PW2;       
      }
      break;
    //---------------------------------//
    case F_LV2_PW0:
      switch (incl_pedana) {
        case PF0:
        case PFB:
          if (stato_velo_corr < LV2)
            nuovo_stato_prog = F_LV1_PW0;         
          break;
        case PF1:
          nuovo_stato_prog = F_LV2_PW1;
          break;
        case PF2:
          nuovo_stato_prog = F_LV2_PW2;       
      }
      break;
    //---------------------------------//
    case F_LV2_PW1:
      switch (incl_pedana) {
        case PF0:
        case PFB:
          nuovo_stato_prog = F_LV2_PW0;
          break;
        case PF1:
          if (stato_velo_corr < LV2)
            nuovo_stato_prog = F_LV1_PW1;
          break;
        case PF2:
          nuovo_stato_prog = F_LV2_PW2;
      }
      break;
    //---------------------------------//
    case F_LV2_PW2:
      switch (incl_pedana) {
        case PF0:
        case PF1:
        case PFB:
          nuovo_stato_prog = F_LV2_PW1;
          break;
        case PF2:
          if (stato_velo_corr == LVX)
             nuovo_stato_prog = F_LVX_PW2;
      }
      break;
    //---------------------------------//
    case F_LVX_PW0:
      switch (incl_pedana) {
        case PF0:
        case PF1:
        case PFB:
          if (stato_velo_corr != LVX) {
            if (stato_velo_corr == LV2)
              nuovo_stato_prog = F_LV2_PW0;
            else
              nuovo_stato_prog = F_LV1_PW0;
          }
          break;
        case PF2:
          if (stato_velo_corr != LVX)
            nuovo_stato_prog = F_LV2_PW0; 
      }
      break;
    //---------------------------------//
    case F_LVX_PW1:
      switch (incl_pedana) {
        case PF0:
        case PF1:
        case PFB:
          nuovo_stato_prog = F_LVX_PW0;
          break;
        case PF2:
          if (stato_velo_corr < LV2) 
            nuovo_stato_prog = F_LV1_PW1; 
      }
      break;
    //---------------------------------//
    case F_LVX_PW2:
      switch (incl_pedana) {
        case PF0:
        case PFB:
          nuovo_stato_prog = F_LVX_PW0;
          break;
        case PF1:
        case PF2:
          nuovo_stato_prog = F_LVX_PW1;
      }
      break;
    //---------------------------------//
    case B_LV0_PW0:
      switch (incl_pedana) {
        case PF0:
        case PF1:
        case PF2:        
          nuovo_stato_prog = F_LV0_PW0;
          inv_da_applicare_dx = true;
          inv_da_applicare_sx = true;
          nuova_inversione_sx = LOW;
          nuova_inversione_dx = LOW;
          break;
        case PFB:
          nuovo_stato_prog = B_LV0_PW1;
      }
      break; 
    //---------------------------------//
    case B_LV0_PW1:
      switch (incl_pedana) {
        case PF0:
        case PF1:
        case PF2:        
          nuovo_stato_prog = F_LV0_PW0;
          inv_da_applicare_dx = true;
          inv_da_applicare_sx = true;          
          nuova_inversione_sx = LOW;
          nuova_inversione_dx = LOW;
          break;
        case PFB:
          if (stato_velo_corr > LV0)
            nuovo_stato_prog = F_LV1_PW1;          
      }
      break;
    //---------------------------------//
    case B_LV1_PW0:
      switch (incl_pedana) {
        case PF0:
        case PF1:
        case PF2:        
          if (stato_velo_corr == LV0)
            nuovo_stato_prog = B_LV0_PW0;
        case PFB:
          nuovo_stato_prog = B_LV1_PW1;
      }
      break;
    //---------------------------------//
    case B_LV1_PW1:
      switch (incl_pedana) {
        case PF0:
        case PF1:
        case PF2:        
          nuovo_stato_prog = B_LV1_PW0;
        case PFB:
          if (stato_velo_corr > LV1)
            nuovo_stato_prog = B_LVX_PW1;          
      }
      break;
    //---------------------------------//
    case B_LVX_PW0: 
      if (stato_velo_corr != LVX)
        nuovo_stato_prog = B_LV1_PW0;

      break;            
    //---------------------------------//
    case B_LVX_PW1:
      switch (incl_pedana) {
        case PF0:
          if (stato_velo_corr != LVX)
            nuovo_stato_prog = B_LV1_PW1;
        case PF1:
        case PF2:        
        case PFB:
          nuovo_stato_prog = B_LVX_PW0;
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
 * Calcola ed applica i valori dei PWM di thorttle delle ruote
 * Il valore dipende da pwm_throttle e dalla lettura del nunchuck
 */
void applica_pwm(void)
{  
  //TODO: applicare le variazioni derivanti dal nunchuck
  //TODO: applicare la calibrazione analogica di compensazione DX/SX
  //Applicazione retroazione: t[i+1] = THROTTLE_GAIN * ERR(t[stato],t[i])
  //l'indice del throttle target è dato dai 2 bit meno significativi dello stato     
  pwm_throttle_istantaneo += (THROTTLE_GAIN * (pwm_throttle[stato_prog & 0b11] - pwm_throttle_istantaneo));  
  if (presenza) {    
    analogWrite(PWM_DX, byte(pwm_throttle_istantaneo));
    analogWrite(PWM_SX, byte(pwm_throttle_istantaneo));
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
 * Applica la procedura di inversione di marcia ai controllers:
 * - Stop per un certo tempo
 * - Applicazione del livello logico
 */
void applica_inversione(void) 
{
  if (inv_da_applicare_dx || inv_da_applicare_sx) {
    if (inv_da_applicare_dx) analogWrite(PWM_DX, 0);
    if (inv_da_applicare_sx) analogWrite(PWM_SX, 0);
    delay(450);  
    if (inv_da_applicare_dx) digitalWrite(INVERS_DX, inversione_dx);
    if (inv_da_applicare_sx) digitalWrite(INVERS_SX, inversione_sx);
    inv_da_applicare_dx = false;
    inv_da_applicare_sx = false;      
  }
}

/**
 * Applica i valori di frenata ai controllers
 */
void applica_frenata(void)
{
    digitalWrite(FRENATA_DX, frenata);
    digitalWrite(FRENATA_SX, frenata);    
}

/**
 * Scrive lo stato del programma sulla seriale
 * Eseguito ogni CICLI_REPORT iterazioni
 */
void report_status(void)
{
  cnt_comm--;
  if (cnt_comm == 0) {
    Serial.print(F("S;"));Serial.print(stato_prog, BIN);
    Serial.print(F(";ZM;"));Serial.print(media_z_acc, DEC);     
    Serial.print(F(";XM;"));Serial.print(media_x_acc, DEC);    
    Serial.print(F(";VD;"));Serial.print(media_velo_dx, DEC);
    Serial.print(F(";VS;"));Serial.print(media_velo_sx, DEC);
    Serial.print(F(";TH;"));Serial.print(pwm_throttle_istantaneo, 1);    
    Serial.print(F(";ID;"));Serial.print(inversione_dx, DEC);
    Serial.print(F(";IS;"));Serial.print(inversione_sx, DEC);  
    Serial.print(F(";B;"));Serial.print(livello_batt, DEC);
    Serial.print(F(";C;"));Serial.print(t_acc, DEC);    
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
