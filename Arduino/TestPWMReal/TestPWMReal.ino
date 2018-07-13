  /***
 * DIY Self Balancing Scooter ATR
 *
 * Test controllo PWM del motore brushless.
 * Utilizza le connessioni effettive realizzate dalla scheda di
 * interfaccia Arduino/Controllers
 * Decodifica i tasti numerici tra 1 e 9 e li tramuta in un PWM tra 0,25V e 2,3V 
 * 
 * 
 * By Alberto Trentadue 2017
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
int pwm_throttle = 0;

char buf[2];

void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);
  Serial.setTimeout(200);

  pinMode(FRENATA_DX, OUTPUT);
  pinMode(FRENATA_SX, OUTPUT);      
  pinMode(INVERS_DX, OUTPUT);     
  pinMode(INVERS_SX, OUTPUT);
  digitalWrite(FRENATA_DX, 0);  
  digitalWrite(FRENATA_SX, 0);
  digitalWrite(INVERS_SX, 0);      
  digitalWrite(INVERS_DX, 0);

  //Il pwm parte da zero
  analogWrite(PWM_DX, 0);
  analogWrite(PWM_SX, 0);  
}

void loop() {
  int l = Serial.readBytes(buf,1);
  if (l > 0) {
    byte v = int(buf[0]);
    if (v >= 48 && v <= 57) {
      if (v == 48) 
        pwm_throttle = 0;
      else
        //ATTN: considera il trimmer che introduce almeno un fattore di attenuazione
        pwm_throttle = (v - 48) * 9 + 130;
      Serial.print("Valore PWM = ");Serial.println(pwm_throttle);        
      Serial.print("Tensione SENZA trimmer = ");Serial.println(float(pwm_throttle)*5/255);        
      // change the analog out value:
      analogWrite(PWM_DX, pwm_throttle);
      analogWrite(PWM_SX, pwm_throttle);
    }
    
  }

  int x1=analogRead(MISVEL_DX);
  delay(10);
  int x2=analogRead(MISVEL_SX);
  delay(10);
  int x3=analogRead(A0);
  Serial.print("Valore VDX = ");Serial.println(x1);        
  Serial.print("Valore VSX = ");Serial.println(x2);        
  Serial.print("Batt = ");Serial.println(x3);        
  
  delay(500);
}
