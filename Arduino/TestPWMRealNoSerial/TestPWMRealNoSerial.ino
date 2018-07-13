  /***
 * DIY Self Balancing Scooter ATR
 *
 * Test controllo PWM del motore brushless.
 * Utilizza le connessioni effettive realizzate dalla scheda di
 * interfaccia Arduino/Controllers
 * Genera una sequenza di valori PWM proporzionali ai primi 5 numeri interi
 * Ogni valore resta per 5 secondi 
 * 
 * By Alberto Trentadue 2018
 */

#define STATUS_LED 13

// Pin dei PWM di controllo brushless
#define PWM_DX 3
#define FRENATA_DX 2
#define INVERS_DX 4
#define MISVEL_DX A1
#define PWM_SX 6
#define FRENATA_SX 5
#define INVERS_SX 7
#define MISVEL_SX A2

//Livello di throttle via PWM
#define MAX_TEST 5
int pwm_throttle = 0;
int liv = 0;
bool up = true;

void setup() {

  pinMode(FRENATA_DX, OUTPUT);
  pinMode(FRENATA_SX, OUTPUT);      
  pinMode(INVERS_DX, OUTPUT);     
  pinMode(INVERS_SX, OUTPUT);
  digitalWrite(FRENATA_DX, 0);  
  digitalWrite(FRENATA_SX, 0);
  digitalWrite(INVERS_SX, 0);      
  digitalWrite(INVERS_DX, 0); 
}

void loop() {

  if (liv == 0) 
    pwm_throttle = 0;
  else
    pwm_throttle = liv * 6 + 120;
  // change the analog out value:
  analogWrite(PWM_DX, pwm_throttle);
  analogWrite(PWM_SX, pwm_throttle);

  delay(5000);

  if (up) {
    liv++;
    if (liv == MAX_TEST) up = false;
  }
  else {
    liv--;
    if (liv == 0) up = true;
  }
}
