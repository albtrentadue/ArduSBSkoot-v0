/***
 * DIY Self Balancing Scooter ATR
 * Self balancing scooter controller sketch using one single rolling board
 * and one single accelerometer sensor.
 * It drives 2 brushless controllers for both speed and direction and is 
 * controlled by a Wii Nunchuck
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
#define F_LV0_PW0 0
#define F_LV0_PW1 1
#define F_LV0_PW2 2
#define F_LV1_PW0 3
#define F_LV1_PW1 4
#define F_LV1_PW2 5
#define F_LV2_PW0 6
#define F_LV2_PW1 7
#define F_LV2_PW2 8
#define F_LVX_PW0 9
#define F_LVX_PW1 10
#define F_LVX_PW2 11
#define B_LV0_PW0 12
#define B_LV0_PW1 13
#define B_LV1_PW0 14
#define B_LV1_PW1 15
#define B_LVX_PW0 16
#define B_LVX_PW1 17
//Stato di funzionamento del programma
byte stato_prog = F_LV0_PW0;

//Costanti inclinazione pedana
#define PF0 0
#define PF1 1
#define PF2 2
#define PFB -1

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
//Flag di attivazione della procedura di inversione
boolean inv_da_applicare_dx = false;
boolean inv_da_applicare_sx = false;

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
//Ottenuto con g * sen 10°
//TODO: Verificare se 10° vanno bene
#define BY_MINIMO 2660
//by,veloce: Valore oltre il quale l'asse è più inclinata
//Ottenuto con g * sen 20°
//TODO: Verificare se 20° vanno bene
#define BY_VELOCE 5239
//bz,margine: Valore oltre il quale la componente bz si considera verticale
//Ottenuto con g * cos 10°
//TODO: Verificare se 10° vanno bene
#define BZ_MARGINE 15087
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
#define LV0 0
#define LV1 1
#define LV2 2
#define LVX 3

//valore minimo del PWM che ferma il motore
//Min Vthorttle : 1,4V
//Ottenuto: PWM_THROTTLE_MIN = 1,4V / 5V * 255
#define PWM_THROTTLE_MIN 70
//Valore massimo ammesso per il PWM Vthrottle per la marcia avanti
//Max Vthorttle : 3V (per ora. TODO: dimensionare)
//Ottenuto: PWM_THROTTLE_MAX = 3,1V / 5V * 255
#define PWM_THROTTLE_MAX 160
//Valore del throttle per l'accelerazione moderata
#define PWM_THROTTLE_MODERATE 100
//Valore del throttle per l'accelerazione rapida
#define PWM_THROTTLE_FAST 150


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
    media_y_acc = somma_y_acc / CICLI_CALC;
  
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
    if (media_y_acc <= -BY_MINIMO) return PFB;
    if (media_y_acc >= BY_VELOCE) return PF2;
    if (media_y_acc >= BY_MINIMO) return PF1;
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
    if (pwm_throttle == PWM_THROTTLE_FAST) return LV2;
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
  //Seleziona in base allo stato del programma
  switch(stato_prog) {
    case F_LV0_PW0:
      switch (incl_pedana) {
        case PF1: 
          stato_prog = F_LV0_PW1;
          pwm_throttle = PWM_THROTTLE_MODERATE;
          break;
        case PF2:
          stato_prog = F_LV0_PW2;
          pwm_throttle = PWM_THROTTLE_FAST;
          break;
        case PFB:
          stato_prog = B_LV0_PW0;
          inv_da_applicare_dx = true;
          inv_da_applicare_sx = true;
          inversione_sx = HIGH;
          inversione_dx = HIGH;
      }
      break;
    //---------------------------------//
    case F_LV0_PW1:
      switch (incl_pedana) {
        case PF0: 
        case PFB:
          stato_prog = F_LV0_PW0;
          pwm_throttle = 0;
          break;
        case PF1:
          if (stato_velo_corr != LV0)
              stato_prog = F_LV1_PW1;
          break;
        case PF2:
          stato_prog = F_LV0_PW2;
          pwm_throttle = PWM_THROTTLE_FAST;          
      }
      break;
    //---------------------------------//      
    case F_LV0_PW2:
      switch (incl_pedana) {
        case PF0: 
        case PFB:
          stato_prog = F_LV0_PW0;
          pwm_throttle = 0;
          break;
        case PF1:
          stato_prog = F_LV0_PW1;
          pwm_throttle = PWM_THROTTLE_MODERATE;
          break;
        case PF2:
          if (stato_velo_corr != LV0)
            stato_prog = F_LV1_PW2;
      }
      break;
    //---------------------------------//
    case F_LV1_PW0:
      switch (incl_pedana) {
        case PF0: 
        case PFB:
          if (stato_velo_corr == LV0) 
            stato_prog = F_LV0_PW0;
          break;
        case PF1:
          stato_prog = F_LV1_PW1;
          pwm_throttle = PWM_THROTTLE_MODERATE;
          break;
        case PF2:                
          stato_prog = F_LV1_PW2;
          pwm_throttle = PWM_THROTTLE_FAST;        
      }
      break;
    //---------------------------------//
    case F_LV1_PW1:
      switch (incl_pedana) {
        case PF0: 
        case PFB:
          stato_prog = F_LV1_PW0;
          pwm_throttle = 0;          
          break;
        case PF1:
          if (stato_velo_corr == LVX)
            stato_prog = F_LVX_PW1;
          break;
        case PF2:                
          stato_prog = F_LV1_PW2;
          pwm_throttle = PWM_THROTTLE_FAST;        
      }
      break;
    //---------------------------------//
    case F_LV1_PW2:
      switch (incl_pedana) {
        case PF0:
        case PF1: 
        case PFB:
          stato_prog = F_LV1_PW1;
          pwm_throttle = PWM_THROTTLE_MODERATE;          
          break;
        case PF2:                
          if (stato_velo_corr > LV1)
            stato_prog = F_LV2_PW2;       
      }
      break;
    //---------------------------------//
    case F_LV2_PW0:
      switch (incl_pedana) {
        case PF0:
        case PFB:
          if (stato_velo_corr < LV2)
            stato_prog = F_LV1_PW0;         
          break;
        case PF1:
          stato_prog = F_LV2_PW1;
          pwm_throttle = PWM_THROTTLE_MODERATE;          
          break;
        case PF2:
          stato_prog = F_LV2_PW2;
          pwm_throttle = PWM_THROTTLE_FAST;          
      }
      break;
    //---------------------------------//
    case F_LV2_PW1:
      switch (incl_pedana) {
        case PF0:
        case PFB:
          stato_prog = F_LV2_PW0;
          pwm_throttle = 0;         
          break;
        case PF1:
          if (stato_velo_corr < LV2)
            stato_prog = F_LV1_PW1;
          pwm_throttle = PWM_THROTTLE_MODERATE;          
          break;
        case PF2:
          stato_prog = F_LV2_PW2;
          pwm_throttle = PWM_THROTTLE_FAST;          
      }
      break;
    //---------------------------------//
    case F_LV2_PW2:
      switch (incl_pedana) {
        case PF0:
        case PF1:
        case PFB:
          stato_prog = F_LV2_PW1;
          pwm_throttle = PWM_THROTTLE_MODERATE;
          break;
        case PF2:
          if (stato_velo_corr == LVX)
             stato_prog = F_LVX_PW2;
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
              stato_prog = F_LV2_PW0;
            else
              stato_prog = F_LV1_PW0;
          }
          break;
        case PF2:
          if (stato_velo_corr != LVX)
            stato_prog = F_LV2_PW0; 
      }
      break;
    //---------------------------------//
    case F_LVX_PW1:
      switch (incl_pedana) {
        case PF0:
        case PF1:
        case PFB:
          stato_prog = F_LVX_PW0;
          pwm_throttle = 0;
          break;
        case PF2:
          if (stato_velo_corr < LV2) 
            stato_prog = F_LV1_PW1; 
      }
      break;
    //---------------------------------//
    case F_LVX_PW2:
      switch (incl_pedana) {
        case PF0:
        case PFB:
          stato_prog = F_LVX_PW0;
          pwm_throttle = 0;
          break;
        case PF1:
        case PF2:
          stato_prog = F_LVX_PW1;
          pwm_throttle = PWM_THROTTLE_MODERATE;          
      }
      break;
    //---------------------------------//
    case B_LV0_PW0:
      switch (incl_pedana) {
        case PF0:
        case PF1:
        case PF2:        
          stato_prog = F_LV0_PW0;
          inv_da_applicare_dx = true;
          inv_da_applicare_sx = true;
          inversione_sx = LOW;
          inversione_dx = LOW;
          break;
        case PFB:
          stato_prog = B_LV0_PW1;
          pwm_throttle = PWM_THROTTLE_MODERATE;          
      }
      break; 
    //---------------------------------//
    case B_LV0_PW1:
      switch (incl_pedana) {
        case PF0:
        case PF1:
        case PF2:        
          stato_prog = F_LV0_PW0;
          inv_da_applicare_dx = true;
          inv_da_applicare_sx = true;          
          inversione_sx = LOW;
          inversione_dx = LOW;
          break;
        case PFB:
          if (stato_velo_corr > LV0)
            stato_prog = F_LV1_PW1;          
      }
      break;
    //---------------------------------//
    case B_LV1_PW0:
      switch (incl_pedana) {
        case PF0:
        case PF1:
        case PF2:        
          if (stato_velo_corr == LV0)
            stato_prog = B_LV0_PW0;
        case PFB:
          stato_prog = B_LV1_PW1;
          pwm_throttle = PWM_THROTTLE_MODERATE;          
      }
      break;
    //---------------------------------//
    case B_LV1_PW1:
      switch (incl_pedana) {
        case PF0:
        case PF1:
        case PF2:        
          stato_prog = B_LV1_PW0;
          pwm_throttle = 0;
        case PFB:
          if (stato_velo_corr > LV1)
            stato_prog = B_LVX_PW1;          
      }
      break;
    //---------------------------------//
    case B_LVX_PW0: 
      if (stato_velo_corr != LVX)
        stato_prog = B_LV1_PW0;

      break;            
    //---------------------------------//
    case B_LVX_PW1:
      switch (incl_pedana) {
        case PF0:
          if (stato_velo_corr != LVX)
            stato_prog = B_LV1_PW1;
        case PF1:
        case PF2:        
        case PFB:
          stato_prog = B_LVX_PW0;
          pwm_throttle = 0;
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
    Serial.print(F("S="));Serial.print(stato_prog, DEC);
    Serial.print(F(";Z="));Serial.print(media_z_acc, DEC);
    Serial.print(F(";Y="));Serial.print(media_y_acc, DEC);    
    Serial.print(F(";VD="));Serial.print(velo_dx, DEC);
    Serial.print(F(";VS="));Serial.print(velo_sx, DEC);
    Serial.print(F(";T="));Serial.print(pwm_throttle, DEC);    
    Serial.print(F(";ID="));Serial.print(inversione_dx, DEC);
    Serial.print(F(";IS="));Serial.print(inversione_sx, DEC);  
    Serial.print(F(";B="));Serial.print(livello_batt, DEC);
    Serial.print(F(";C="));Serial.print(t_acc, DEC);  
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
