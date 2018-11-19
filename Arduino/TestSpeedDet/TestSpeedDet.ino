  /***
 * DIY Self Balancing Scooter ATR
 *
 * Test lettura detectors di velocità.
 * Utilizza le connessioni effettive realizzate dalla scheda di
 * interfaccia Arduino/Controllers
 * 
 * By Alberto Trentadue 2018
 */

// Pin dei PWM di controllo brushless
#define PWM_DX 3
#define FRENATA_DX 2
#define INVERS_DX 4
#define MISVEL_DX A2
#define PWM_SX 6
#define FRENATA_SX 5
#define INVERS_SX 7
#define MISVEL_SX A1
#define JUMPER 11

//Livello di throttle via PWM
int pwm_throttle = 120;

//Costanti del rivelatore di velocità
byte stato_riv_vel_dx = LOW;
byte stato_riv_vel_sx = LOW;
int cnt_periodo_dx = 0;
int cnt_periodo_sx = 0;
//Numero massimo di conteggi oltre il quale la ruota si considera quasi ferma
#define MAX_PERIODO_RIV 120
int vel_dx = 0;
int vel_sx = 0;

//Livello batteria
int livello_batt = 0;

#define CICLI_HEART 100 // cicli da attendere per l'heartbeat
byte cnt_heart = CICLI_HEART;
byte heart = 0;
#define CICLI_COMM 5 // cicli da attendere per comunicare i dati
byte cnt_comm = CICLI_COMM;

void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(115200);

  pinMode(FRENATA_DX, OUTPUT);
  pinMode(FRENATA_SX, OUTPUT);      
  pinMode(INVERS_DX, OUTPUT);     
  pinMode(INVERS_SX, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(JUMPER, INPUT_PULLUP);
  pinMode(MISVEL_DX, INPUT);
  pinMode(MISVEL_SX, INPUT);
  
  digitalWrite(FRENATA_DX, 0);  
  digitalWrite(FRENATA_SX, 0);
  digitalWrite(INVERS_SX, 0);      
  digitalWrite(INVERS_DX, 0);

  //Il pwm parte da zero
  analogWrite(PWM_DX, 0);
  analogWrite(PWM_SX, 0); 

  stato_riv_vel_dx = digitalRead(MISVEL_DX);
  stato_riv_vel_sx = digitalRead(MISVEL_SX);
}

void loop() {
  if (digitalRead(JUMPER) == LOW) {
      analogWrite(PWM_DX, pwm_throttle);
      analogWrite(PWM_SX, pwm_throttle);      
  }
  else {
      analogWrite(PWM_DX, 0);
      analogWrite(PWM_SX, 0);           
  }
  
  rivela_velocita();  
  report_status();
  heartbeat();
  
  delay(3);
}

//calcola il periodo dalle onde quadre dei rivelatori
void rivela_velocita(void) 
{
  byte x;

  //Transizione ad ogni cambio di livello, si assume duty al 50%
  //DESTRA
  x = digitalRead(MISVEL_DX);
  if (stato_riv_vel_dx == x) 
    if (cnt_periodo_dx < MAX_PERIODO_RIV) cnt_periodo_dx++;
    else vel_dx = 0;          
  else {    
    vel_dx = (MAX_PERIODO_RIV * 10 / cnt_periodo_dx);
    cnt_periodo_dx = 0;
  }
  stato_riv_vel_dx = x;
 
  //SINISTRA
  x = digitalRead(MISVEL_SX);
  if (stato_riv_vel_sx == x) 
    if (cnt_periodo_sx < MAX_PERIODO_RIV) cnt_periodo_sx++;
    else vel_sx = 0;          
  else {
    vel_sx = (MAX_PERIODO_RIV * 10 / cnt_periodo_sx);
    cnt_periodo_sx = 0;
  }
  stato_riv_vel_sx = x;
            
}


/**
   Scrive lo stato del programma sulla seriale
   Eseguito ogni CICLI_REPORT iterazioni
*/
void report_status(void)
{
  cnt_comm--;
  if (cnt_comm == 0) {
    //Serial.print(F(";Td;")); Serial.print(pwm_throttle, DEC);
    Serial.print(F(";VD;")); Serial.print(vel_dx, DEC);
    Serial.print(F(";VS;")); Serial.print(vel_sx, DEC);        
    Serial.println();

    cnt_comm = CICLI_COMM;
  }
}


/**
  Heartbeat on the built_in Led
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

