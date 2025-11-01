// Demonstrate how to use the gyro and encoders to go straight
// using a PD controller to correct the angle
// The application use a simple state machine to change the state
// Author: Nicolas Bourré

#include <MeAuriga.h>

enum AppState {SETUP, STOP, STRAIGHT, TURNING,DELIVERY};

AppState currentState = STOP;

#define DIST_WHEEL 151
#define DIA_WHEEL 64.5
#define PULSE 9
#define RATIO 39.267
#define FULL_TURN_CIRC 948.8
#define FULL_SPIN_CIRC 474.4
#define CIRC_WHEEL 202.6
#define LEDNUM  12
#define LEDPIN  44
#define RINGALLLEDS 0

MeRGBLed led( PORT0, LEDNUM );

MeGyro gyro(0, 0x69);

MeEncoderOnBoard encoderRight(SLOT1);
MeEncoderOnBoard encoderLeft(SLOT2);
MeUltrasonicSensor ultraSensor(PORT_10);

unsigned long currentTime = 0;

unsigned long serialPrintPrevious = 0;
int serialPrintInterval = 500;
String msg = "";


float TargetDist = 1500;
float DeliveryDist = 45;
float speed1 = 10;
unsigned long pulsationsTotal = 0;
unsigned long pulsationsTotal1 = 0;
unsigned long pulsationsTourner = 0;
static short turnSpeed=0;
static unsigned long pulsationsFait = 0;
unsigned long pulsationsDeliver = 0;
float TurnDist1 = 294.5;
float TurnDist2 = 600;
float deliveryDistance = 0;

int stopCounter = 0;
float thresholdDist = 15.0;
float zero = 0;
int stateCounter = 1;



// ********* INTERRUPTIONS ***********

void rightEncoderInterrupt(void)
{
  if(digitalRead(encoderRight.getPortB()) == 0)
  {
    encoderRight.pulsePosMinus();
  }
  else
  {
    encoderRight.pulsePosPlus();;
  }
}

void leftEncoderInterrupt(void) {
  if(digitalRead(encoderLeft.getPortB()) == 0)
  {
    encoderLeft.pulsePosMinus();
  }
  else
  {
    encoderLeft.pulsePosPlus();
  }
}

// ************* DÉBUT ************

void encoderConfig() {
  attachInterrupt(encoderRight.getIntNum(), rightEncoderInterrupt, RISING);
  attachInterrupt(encoderLeft.getIntNum(), leftEncoderInterrupt, RISING);
  
  encoderRight.setPulse(PULSE);
  encoderLeft.setPulse(PULSE);
  
  encoderRight.setRatio(RATIO);
  encoderLeft.setRatio(RATIO);
  
  encoderRight.setPosPid(1.8,0,1.2);
  encoderLeft.setPosPid(1.8,0,1.2);
  
  encoderRight.setSpeedPid(0.18,0,0);
  encoderLeft.setSpeedPid(0.18,0,0);
  
  // DÉBUT : Ne pas modifier ce code!
  // Configuration de la fréquence du PWM
  // Copier-coller ce code si on désire
  // travailler avec les encodeurs
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);

  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);
  // FIN : Ne pas modifier ce code!
}
long distToPulse(float Dist) {
  float nb_tours_roue = Dist / CIRC_WHEEL;
  float nb_tours_moteur = nb_tours_roue * RATIO;
  long pulsations = (long)(nb_tours_moteur * PULSE + 0.5);
  return pulsations;
}
void updatePulsationsFait() {
  long leftPos = encoderLeft.getPulsePos();
  long rightPos = encoderRight.getPulsePos();
  long nbreMoteur = 2;
  pulsationsFait = (abs(leftPos) + abs(rightPos)) / nbreMoteur;
}

void ledTask(unsigned long ct){
  
  static int nbreLed = 0;
  updatePulsationsFait();

  if (stateCounter ==1){
    nbreLed = map(pulsationsFait,0,pulsationsTotal,1,LEDNUM);
    nbreLed = ceil(nbreLed)+1; 
    for(int i = 1; i < nbreLed;i++){
    led.setColor(i,0,255,0);
    }

  }else if(stateCounter > 1){
    return;
  }


  led.show();

}


void setup() {
  Serial.begin(115200);
  encoderConfig();
  gyro.begin();
   led.setpin(LEDPIN);
  currentState = SETUP;
  pulsationsTotal = distToPulse(TargetDist);
}

void SerialCommand() {
  if (Serial.available()) {
    command = Serial.readStringUntil('\n');
    command.trim();
    if (command.length() > zero) {
      traiterCommande(command);
    }
  }
}

void traiterCommande(const String& commande) {
  if (commande == "w") {
    
   
    return;
  }

  if (commande == "s") {
    
   
    return;
  }

   if (commande == "a") {
    
   
    return;
  }

   if (commande == "d") {
    
   
    return;
  }
}



void gyroTask(unsigned long ct) {
  gyro.update();
}

void encodersTask(unsigned long ct) {
  encoderRight.loop();
  encoderLeft.loop();
}
bool spin(short speed, double targetAngle){
  static bool firstRun = true;
  static double tolerance = 2.0;
  static double zAngleGoal = 0.0;

  if (firstRun) {
   firstRun = false;
    gyro.resetData();

    zAngleGoal = gyro.getAngleZ() - targetAngle;
    return false;
  }

  encoderLeft.setMotorPwm(-speed);
  encoderRight.setMotorPwm(-speed);

  return (gyro.getAngleZ()- zAngleGoal ) < tolerance;

}



void turnState (short speed = 80){
  static double zAngleGoal = 0.0;
  static double targetAngle = 0.0;
    static bool firstRun = true;
    static double error = 0.0;
    static double previousError = 0.0;
    static double output = 0;
    
    // PD Controller
    // Change les valeurs selon tes besoins
    // higher kp = plus réactive, peu osciller
    // lowewr kp = sluggish, moins d'oscillation
    // higher kd = limite l'oscillation, la bonne valeur arrête l'oscillation
    const double kp = 48;
    //const double ki = 1.0;
    const double kd = 0;
    
    if (firstRun) {
      firstRun = false;
      stateCounter +=1;
      led.setColor(0,0,0);
      led.show();
      zAngleGoal = gyro.getAngleZ();
     targetAngle = zAngleGoal + 90.0;
      Serial.println ("Setting speed");
      encoderLeft.setPulsePos(0);
      encoderRight.setPulsePos(0);
      //encoderLeft.setMotorPwm(-speed);
      //encoderRight.setMotorPwm(-speed);
      
      return;
    }
    stateCounter = 2;
    error = gyro.getAngleZ() - zAngleGoal;

    // Google : ELI5 PID
    // Astuce web : ELI5 = Explain Like I'm 5
    output = kp * error + kd * (error - previousError);
    speed = abs(-speed - output);
    previousError = error;        
    
   bool stop = spin(speed,targetAngle);

   if (stop){
    stopCmd();
    deliveryCmd();
   }

}

void delivery(unsigned long ct) {
  static bool deliverStarted = false;
  static unsigned long deliverStartTime = 0;
  static unsigned long lastBlinkTime = 0;
  static bool ledState = false;

  const unsigned long totalTime = 3000;      
  const unsigned long blinkInterval = 500;    

  // Démarrage du cycle
  if (!deliverStarted) {
    deliverStarted = true;
    deliverStartTime = ct;
    lastBlinkTime = ct;
    ledState = false;
    Serial.println("Début livraison - clignotement 3 secondes");
  }

  if (ct - deliverStartTime >= totalTime) {
    led.setColor(0, 0, 0, 0); // Éteint
    led.show();
    deliverStarted = false;
    Serial.println("Fin livraison");
    return;
  }

  // Gestion du clignotement
  // if (ct - lastBlinkTime >= blinkInterval) {
  //   ledState = !ledState; 
  //   lastBlinkTime = ct;

   
      led.setColor(0, 0, 255, 0); 

    led.show();
  // }
}


void deliveryState(unsigned long ct ){
  
  
  static short speed = 80;
  static double zAngleGoal = 0.0;
    static bool firstRun = true;
    static double error = 0.0;
    static double previousError = 0.0;
    static double output = 0;
    
    // PD Controller
    // Change les valeurs selon tes besoins
    // higher kp = plus réactive, peu osciller
    // lowewr kp = sluggish, moins d'oscillation
    // higher kd = limite l'oscillation, la bonne valeur arrête l'oscillation
    const double kp = 6.75;
    //const double ki = 1.0;
    const double kd = 1.0;
    
    if (firstRun) {
      firstRun = false;
      led.setColor(0,0,0);
      zAngleGoal = gyro.getAngleZ();
      Serial.println ("Setting speed");
      encoderLeft.setMotorPwm(speed);
      encoderRight.setMotorPwm(-speed);
      
      return;
    }
    updatePulsationsFait();
    error = gyro.getAngleZ() - zAngleGoal;
    
    // Google : ELI5 PID
    // Astuce web : ELI5 = Explain Like I'm 5
    output = kp * error + kd * (error - previousError);
    
    previousError = error;        
    
    msg = "z : ";
    msg += gyro.getAngleZ();
    msg += "\tleft : ";
    msg += encoderLeft.getCurPwm();
    msg += "\tright : ";
    msg += encoderRight.getCurPwm();
    msg += "\toutput :";
    msg += output;
    
    encoderLeft.setMotorPwm(speed - output);
    encoderRight.setMotorPwm(-speed - output);

    float distanceCm = ultraSensor.distanceCm();
      if(distanceCm <= DeliveryDist && distanceCm > 0) {
        pulsationsDeliver = pulsationsFait;
        stopCmd();
        delivery(ct);
      }
}
void goStraight(short speed = 100) {
    static double zAngleGoal = 0.0;
    static bool firstRun = true;
    static double error = 0.0;
    static double previousError = 0.0;
    static double output = 0;
    
    // PD Controller
    // Change les valeurs selon tes besoins
    // higher kp = plus réactive, peu osciller
    // lowewr kp = sluggish, moins d'oscillation
    // higher kd = limite l'oscillation, la bonne valeur arrête l'oscillation
    const double kp = 6.75;
    //const double ki = 1.0;
    const double kd = 1.0;
    
    if (firstRun) {
      firstRun = false;
      led.setColor(0,0,0);
      zAngleGoal = gyro.getAngleZ();
      Serial.println ("Setting speed");
      encoderLeft.setMotorPwm(speed);
      encoderRight.setMotorPwm(-speed);
      
      return;
    }
    updatePulsationsFait();
    error = gyro.getAngleZ() - zAngleGoal;
    
    // Google : ELI5 PID
    // Astuce web : ELI5 = Explain Like I'm 5
    output = kp * error + kd * (error - previousError);
    
    previousError = error;        
    
    msg = "z : ";
    msg += gyro.getAngleZ();
    msg += "\tleft : ";
    msg += encoderLeft.getCurPwm();
    msg += "\tright : ";
    msg += encoderRight.getCurPwm();
    msg += "\toutput :";
    msg += output;
    
    encoderLeft.setMotorPwm(speed - output);
    encoderRight.setMotorPwm(-speed - output);

    if(pulsationsFait >= pulsationsTotal) {
        stopCmd();
        turningCmd();
    }
}

void stopCmd() {
  Serial.println ("Stopping");
  msg = "";
  
  currentState = STOP;
}

void turningCmd(){
  Serial.println ("turning");
  currentState = TURNING;
}
void deliveryCmd(){
  Serial.println ("turning");
  currentState = DELIVERY;
}

void straightCmd() {
  Serial.println ("Going straight");
 
  currentState = STRAIGHT;
  goStraight();
}

void setupState(unsigned long ct) {
  static bool firstTime = true;
  static unsigned long lastTime = 0;
  static unsigned long exitTime = 0;
  
  const int timeout = 3000;
  
  if (firstTime) {
    firstTime = false;
    exitTime = ct + timeout;
    
    Serial.println("Attente de 3 secondes avant de démarrer.");
  }
  
  // Là la là... j'attends
  
  if (ct >= exitTime) {
    firstTime = true;
    straightCmd();
  }
}

void stopState(unsigned long ct) { 
  encoderLeft.setMotorPwm(0);
  encoderRight.setMotorPwm(0);
}

void straightState(unsigned long ct) {  
  goStraight();
}

void serialEvent() {
  while (Serial.available()) {
    char cmd = Serial.read();
    
    switch (cmd) {
      case '0':
        stopCmd();
        break;
      case '1':
        straightCmd();
        break;
    }
  }
}

void serialPrintTask(unsigned long cT) {
  static unsigned long lastTime = 0;
  const int rate = 500;
  
  if (cT - lastTime < rate) return;

  lastTime = cT;

  if (msg != "") {
    Serial.println(msg);
    msg = "";
  }
}
void loop() {
  ledTask(currentTime);
  currentTime = millis();
  
  switch (currentState) {
    case SETUP:
      setupState(currentTime);
      break;
    case STOP:
      stopState(currentTime);
      break;
    case STRAIGHT:
      straightState(currentTime);
      break;
    case TURNING:
      turnState();
      break;
    case DELIVERY:
       deliveryState(currentTime);
       break;
    default:
      stopState(currentTime);
      break;
  }
  
  gyroTask(currentTime);
  encodersTask(currentTime);
  serialPrintTask(currentTime);
}