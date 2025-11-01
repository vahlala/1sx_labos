#include <MeAuriga.h>

#define LEDNUM 12
#define LEDPIN 44
#define RINGALLLEDS 0
#define BUZZER_PIN 45

MeEncoderOnBoard encoderLeft(SLOT1);
MeEncoderOnBoard encoderRight(SLOT2);
MeGyro gyro(0, 0x69);

bool debugMode = true;

MeRGBLed led(PORT0, LEDNUM);
MeUltrasonicSensor ultraSensor(PORT_10);

void isr_process_encoder1() {
  if (digitalRead(encoderLeft.getPortB()) == 0) encoderLeft.pulsePosMinus();
  else encoderLeft.pulsePosPlus();
}
void isr_process_encoder2() {
  if (digitalRead(encoderRight.getPortB()) == 0) encoderRight.pulsePosMinus();
  else encoderRight.pulsePosPlus();
}
static bool automatic = false;
static bool debugFlag = false;
static int speed = 100;
static int speedBack = speed;
static int speedTurn = speed;
int distance;
String PreviousCmd = "S";
static int dist;
static String receivedData;


enum state {
  NORMAL,
  AUTO,
  DEBUG
};
state current_state = NORMAL;

void setup() {
  Serial.begin(115200);
  led.setpin(LEDPIN);
  pinMode(BUZZER_PIN, OUTPUT);
  attachInterrupt(encoderLeft.getIntNum(), isr_process_encoder1, RISING);
  attachInterrupt(encoderRight.getIntNum(), isr_process_encoder2, RISING);
  gyro.begin();
  gyro.resetData();
  encoderLeft.setPulse(9);
  encoderRight.setPulse(9);
  encoderLeft.setRatio(39.267);
  encoderRight.setRatio(39.267);
  encoderLeft.setPosPid(1.8, 0, 1.2);
  encoderRight.setPosPid(1.8, 0, 1.2);
  encoderLeft.setSpeedPid(0.18, 0, 0);
  encoderRight.setSpeedPid(0.18, 0, 0);
}

void loop() {

  if(debugFlag){
    modedebug();
  }

    calculeDistance();

  if(automatic){
      if (distance <= dist) {
      automatic=false;
      off();
      offLed();
      current_state = NORMAL;
    } else {
      completeLed();
    }
  }
  encoderLeft.loop();
  encoderRight.loop();
  gyro.update();
}

void backward(int speedBack, short firstRun = 0) {

  static double zAngleGoal = 0.0;

  static double error = 0.0;
  static double previousError = 0.0;
  static double output = 0;

  const double kp = 3.0;
  const double kd = 1.0;

  if (firstRun) {
    gyro.resetData();

    zAngleGoal = gyro.getAngleZ();
    firstRun = 0;
    Serial.println("Setting speed");

    encoderLeft.setMotorPwm(-speed);
    encoderRight.setMotorPwm(speed);

    return;
  }

  error = gyro.getAngleZ() - zAngleGoal;

  output = kp * error + kd * (error - previousError);

  previousError = error;

  encoderLeft.setMotorPwm(-speed - output);
  encoderRight.setMotorPwm(speed - output);
  // gyro.resetData();
}

void forward(int speed, short firstRun = 0) {
  static double zAngleGoal = 0.0;

  static double error = 0.0;
  static double previousError = 0.0;
  static double output = 0;

  const double kp = 16.75;
  const double kd = 3.0;
 
  if (firstRun) {
    gyro.resetData();

    zAngleGoal = gyro.getAngleZ();
    firstRun = 0;
    Serial.println("Setting speed");

    encoderLeft.setMotorPwm(speed);
    encoderRight.setMotorPwm(-speed);

    return;
  }

  error = gyro.getAngleZ() - zAngleGoal;

  output = kp * error + kd * (error - previousError);

  previousError = error;

  encoderLeft.setMotorPwm(-speed - output);
  encoderRight.setMotorPwm(speed - output);
}


void turnRight(int speedTurn, bool isTurning = false) {
  if (!isTurning) {
    isTurning = true;

    encoderLeft.setMotorPwm(speed);
    encoderRight.setMotorPwm(speed);
  }
  isTurning = false;
}

void turnLeft(int speedTurn, bool isTurning = false) {
  if (!isTurning) {
    isTurning = true;

    encoderLeft.setMotorPwm(-speed);
    encoderRight.setMotorPwm(-speed);
  }
  isTurning = false;
}

// Événement qui se déclenche automatiquement lorsqu'il y a
// réception de données via le port série (Bluetooth ou Moniteur)
void serialEvent() {
  receivedData = "";

  if (!Serial.available()) return;

  receivedData = Serial.readStringUntil('\n');
  parseData(receivedData);
}

void parseData(String& receivedData) {
  bool isFromBLE = false;

  if (receivedData.length() >= 2) {
    // Vérifier si les deux premiers octets sont 0xFF55 (BLE) --> (Header)
    if ((uint8_t)receivedData[0] == 0xFF && (uint8_t)receivedData[1] == 0x55) {
      // Supprimer les deux premiers octets du header (0xFF55)
      receivedData.remove(0, 2);
      isFromBLE = true;
    }
    // Vérifier si les deux premiers caractères sont "!!" (Moniteur Série)
    else if (receivedData.startsWith("!!")) {
      // Supprimer les deux premiers octets du header (!!)
      receivedData.remove(0, 2);
    } else {
      // En-tête non reconnue
      Serial.print(F("Données non reconnues : "));
      Serial.println(receivedData);
      return;
    }
  } else {
    Serial.print(F("Données trop courtes : "));
    Serial.println(receivedData);
    return;
  }
 if(PreviousCmd != receivedData){
  if (debugMode) {
    Serial.print(F("Reçu du "));
    Serial.print(isFromBLE ? F("BLE") : F("Moniteur Série"));
    Serial.print(" : ");
    Serial.println(receivedData);
  }
 }
  int index = receivedData.indexOf(',');
  if (index == -1) {
    gererCommandeSimple(receivedData);
  } else {
    String action = receivedData.substring(0, index);
    String parametre = receivedData.substring(index + 1);
    gererCommandeComposee(action, parametre);
  }
}

void gererCommandeSimple(const String& cmd) {
    automatic = false;
    offLed();
    if (cmd == "F") {
      forwardCmd();
    } else if (cmd == "B") {
      backwardCmd();
    } else if (cmd == "R") {
      turnRightCmd();
    } else if (cmd == "L") {
      turnLeftCmd();
    } else if (cmd == "K") {
      analogWrite(BUZZER_PIN, 127);
    }else if (cmd == "d"){
        debugFlag =!debugFlag;
    }else if (cmd == "S") {
      off();
      analogWrite(BUZZER_PIN, 0);
    }
 
  PreviousCmd = cmd;
}

void gererCommandeComposee(const String& action, const String& parametre) {
calculeDistance();
  if (action == "P") {
    speed = parametre.toInt();
    speedBack = speed / 2;
    speedTurn = speed / 3;
  } else if (action == "AUTO") {
    current_state == AUTO;
     dist = parametre.toInt();
     forwardCmd();
    automatic = true;
  
  
  }
}

int calculeDistance(){
  static unsigned long previousMillis=0;
  unsigned long ct = millis();
  int rate = 200;

  if(ct-previousMillis>=rate){
    previousMillis = ct;

  distance = ultraSensor.distanceCm();
  }
  return distance;
}

void forwardCmd() {
  static bool firstTime = true;

  if (PreviousCmd == "S") firstTime = true;


  if (firstTime) {
    forward(speed, 1);
    firstTime = false;
  }

  forward(speed);
}

void backwardCmd() {
  static bool firstTime = true;

  if (PreviousCmd == "S") firstTime = true;

  if (firstTime) {
    forward(-speedBack, 1);
    firstTime = false;
  }
  forward(-speedBack);
  bip();
}

void turnRightCmd() {
  static bool firstTime = true;

  if (PreviousCmd == "S") firstTime = true;

  if (firstTime) {
    turnRight(speedTurn, 1);
    firstTime = false;
  }

  turnRight(speedTurn);
  ledRight();
}

void turnLeftCmd() {
  static bool firstTime = true;

  if (PreviousCmd == "S") firstTime = true;

  if (firstTime) {
    turnLeft(speedTurn, 1);
    firstTime = false;
  }

  turnLeft(speedTurn);
  ledLeft();
}


void bip() {
  int rate = 250;
  static unsigned long previousTime = 0;
  unsigned long ct = millis();
  static bool buzz = false;

  if (ct - previousTime >= rate) {
    previousTime = ct;
    if (buzz) {
      analogWrite(BUZZER_PIN, 0);
    } else {
      analogWrite(BUZZER_PIN, 127);
    }
  }
  buzz = !buzz;
}

void off() {
  encoderLeft.setMotorPwm(0);
  encoderRight.setMotorPwm(0);
  ledBack();
}

void offLed() {
  const int off = 0;
  led.setColor(off, off, off);
  led.show();
}

void ledLeft() {
  static unsigned long previousTime = 0;
  int rate = 250;
  static bool ledState = false;
 unsigned long ct = millis();
 const int off = 0;
  const int red = 125;
  const int green = 125;
  const int blue = 0;

  if (ct - previousTime >= rate) {
    previousTime = ct;
    if (ledState) {
      led.setColor(off, off, off);
    } else {
      led.setColorAt(11, red, green, blue);
      led.setColorAt(0, red, green, blue);
      led.setColorAt(1, red, green, blue);
      led.setColorAt(2, red, green, blue);
    }
  }
  led.show();
  ledState = !ledState;
}

void ledRight() {
  static unsigned long previousTime = 0;
  int rate = 250;
  static bool ledState = false;
  unsigned long ct = millis();
   const int off = 0;
  const int red = 125;
  const int green = 125;
  const int blue = 0;

  if (ct - previousTime >= rate) {
    previousTime = ct;

    if (ledState) {
      led.setColor(off, off, off);
    } else {

      for (int idx = 2; idx <=5; idx++) {
        led.setColorAt(idx, red, green,blue);
      }
    }
  }
  led.show();
  ledState = !ledState;
}

void ledBack() {
  const int off = 0;
  const int red = 125;
  const int green = 0;
  const int blue = 0;
  led.setColor(off, off, off);
  for (int idx = 5; idx < LEDNUM; idx++) {
    led.setColorAt(idx, red, green, blue);
  }
  led.show();
}
void completeLed() {
  static unsigned long previousTime = 0;
  int rate = 300;
  static bool ledState = false;
  unsigned long ct = millis(); 
  const int red = 125;
  const int green = 125;
  const int blue = 0;
  if (ct - previousTime >= rate) {
    previousTime = ct;
    if (ledState) {
      led.setColor(0, 0, 0);
    } else {
      for (int idx = 0; idx <LEDNUM; idx++) {
        led.setColorAt(idx,red, green,blue);
      }
    }
  }
  led.show();
  ledState = !ledState;
}

void modedebug(){
  static unsigned long previousTime = 0;
  int rate = 200;
  unsigned long ct = millis();
   
  if (ct - previousTime >= rate) {
    previousTime = ct;
    Serial.println(receivedData);
    Serial.println(distance);
  }
}
