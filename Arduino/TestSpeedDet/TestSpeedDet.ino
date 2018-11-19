  /***
 * DIY Self Balancing Scooter ATR
 *
 * Test lettura detectors di velocità.
 * Utilizza le connessioni effettive realizzate dalla scheda di
 * interfaccia Arduino/Controllers
 * 
 * By Alberto Trentadue 2017-2018
 */

#define STATUS_LED 13

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
int pwm_throttle = 135;

//Costanti del rivelatore di velocità
#define RIV_VEL_HIGH 430 //valore limite alto A/D dell'onda quadra
#define RIV_VEL_LEV 470
#define RIV_VEL_LOW 370 //valore limite basso A/D dell'onda quadra
int xs, xd;
bool stato_riv_vel_dx = false;
bool stato_riv_vel_sx = false;
int cnt_periodo_dx = 0;
int cnt_periodo_sx = 0;
//Numero massimo di conteggi oltre il quale la ruota si considera quasi ferma
#define MAX_PERIODO_RIV 120
int last_periodo_dx = MAX_PERIODO_RIV;
int last_periodo_sx = MAX_PERIODO_RIV;

//Livello batteria
int livello_batt = 0;

#define CICLI_HEART 100 // cicli da attendere per l'heartbeat
byte cnt_heart = CICLI_HEART;
byte heart = 0;
#define CICLI_COMM 50 // cicli da attendere per comunicare i dati
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
  
  digitalWrite(FRENATA_DX, 0);  
  digitalWrite(FRENATA_SX, 0);
  digitalWrite(INVERS_SX, 0);      
  digitalWrite(INVERS_DX, 0);

  //Il pwm parte da zero
  analogWrite(PWM_DX, 0);
  analogWrite(PWM_SX, 0);  
}

void loop() {
  if (digitalRead(JUMPER) == LOW) {
      //analogWrite(PWM_DX, pwm_throttle);
      analogWrite(PWM_SX, pwm_throttle);      
  }
  else {
      //analogWrite(PWM_DX, 0);
      analogWrite(PWM_SX, 0);           
  }
  
  rivela_velocita();
  livello_batt=analogRead(A0);       
  report_status();
  heartbeat();
  
  delay(3);
}

//calcola il periodo dalle onde quadre dei rivelatori
void rivela_velocita(void) 
{
  bool riv_vel_active;

  //SINISTRA
  //scarta la prima lettura
  analogRead(MISVEL_SX);
  xs=analogRead(MISVEL_SX);
  if (abs(xs - RIV_VEL_LEV) < 50) {
    //Zona conteggio
    if (stato_riv_vel_sx) cnt_periodo_sx = constrain(++cnt_periodo_sx, 0, MAX_PERIODO_RIV);
    else {
      cnt_periodo_sx = 0;
      stato_riv_vel_sx = true;
    }
  }
  else //Zona non conteggio
    if (stato_riv_vel_sx) {
      last_periodo_sx = cnt_periodo_sx;
      stato_riv_vel_sx = false;
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
    //Serial.print(F(";Td;")); Serial.print(pwm_throttle, DEC);
    //Serial.print(F(";XD;")); Serial.print(xd, DEC);
    //Serial.print(F(";XS;")); Serial.print(xs, DEC);        
    Serial.print(F(";PD;")); Serial.print(last_periodo_dx, DEC);
    Serial.print(F(";PS;")); Serial.print(last_periodo_sx, DEC);        
    //Serial.print(F(";B;")); Serial.print(livello_batt, DEC);
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

