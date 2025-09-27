
#include <MeAuriga.h>
#define LEDNUM 12
#define LEDPIN 44
MeRGBLed led(PORT0, LEDNUM);

MeUltrasonicSensor ultraSensor(PORT_10);
enum State { NORMAL,
             RALENTI,
             DANGER,
             RONDE };
State currentState = NORMAL;

static bool rondeActive = false;
unsigned long currentTime = 0;
int dist = 400;
int maxPwm = 190;
int halfPwm = 127;
int turnPwm = 180;
int halfled = 6;

int maxDist = 80;
int normalDist = 40;


//Motor Left
const int m1_pwm = 11;
const int m1_in1 = 49;  // M1 ENA
const int m1_in2 = 48;  // M1 ENB

//Motor Right
const int m2_pwm = 10;
const int m2_in1 = 46;  // M2 ENA
const int m2_in2 = 47;  // M2 ENB

void setup() {
  Serial.begin(9600);
  led.setpin(LEDPIN);
  pinMode(m1_pwm, OUTPUT);
  pinMode(m1_in2, OUTPUT);
  pinMode(m1_in1, OUTPUT);
  pinMode(m2_pwm, OUTPUT);
  pinMode(m2_in2, OUTPUT);
  pinMode(m2_in1, OUTPUT);
  currentTime = millis();
}

void loop() {
  currentTime = millis();
  distanceTask(currentTime);
  stateManager(currentTime);
}

void distanceTask(unsigned long ct) {
  static unsigned long previousTime = ct;
  int rate = 250;
  int maxDist = 400;
  int lastValidDist;
  int validDist = 0;
  if (ct - previousTime >= rate) {
    validDist = ultraSensor.distanceCm();
    if (validDist > maxDist) {
      dist = lastValidDist;
    } else {
      dist = validDist;
      lastValidDist = validDist;
    }
    Serial.println("dist: ");
    Serial.println(dist);
    previousTime = ct;
  }
}

void normalState(unsigned long ct) {
  static unsigned long previousTime = ct;
  static bool firstTime = true;
  unsigned long rondeStart = 0;
  int rondeTime = 10000;


  if (firstTime) {
    firstTime = false;
    Serial.println("Entrée État : NORMALE");
    previousTime = ct;
    led.setColor(0, 0, 0);
    for (int i = 0; i <= 6; i++) {
      led.setColorAt(i, 0, 255, 0);
    }
    led.show();
  }

  if (dist >= 80) {
    if (!rondeActive) {
      rondeStart = ct - previousTime;
    }
  } else {
    rondeActive = false;
  }
  if (rondeStart >= rondeTime) {
    currentState = RONDE;
    rondeStart = 0;
    previousTime = ct;
    rondeActive = true;
  }




  NormalSpeedMode();
  bool transition = dist <= 80;

  if (transition) {
    currentState = RALENTI;
    Serial.println("Sortie État : NORMALE");
    firstTime = true;
    return;
  }

  bool transArrt = dist <= 40;

  if (transArrt) {
    currentState = DANGER;
    Serial.println("Sortie État : NORMALE");
    firstTime = true;
    return;
  }
}

void ralentiState() {
  static bool firstTime = true;

  if (firstTime) {
    firstTime = false;
    Serial.println("Entrée État : RALENTI");
    led.setColor(0, 0, 0);
    for (int i = 0; i <= halfled; i++) {
      led.setColorAt(i, 0, 0, 255);
    }
    led.show();
  }


  ReduceSpeed();
  bool transition = dist >= maxDist;

  if (transition) {
    currentState = NORMAL;
    Serial.println("Sortie État : RALENTI");

    firstTime = true;
    return;
  }

  bool transArrt = dist <= normalDist;

  if (transArrt) {
    currentState = DANGER;
    Serial.println("Sortie État : RALENTI");

    firstTime = true;
    return;
  }
}

void dangerState(unsigned long ct) {
  static bool firstTime = true;
  static unsigned long phaseStart = 0;
  static int phase = 0;
  int stopTime = 500;
  int rearTime = 1000;
  int turnTime = 800;

  if (firstTime) {
    firstTime = false;
    Serial.println("Entrée État : DANGER");
    led.setColor(0, 0, 0);
    for (int i = 0; i < LEDNUM; i++) {
      led.setColorAt(i, 255, 0, 0);
    }
    led.show();

    phase = 0;
    phaseStart = ct;
  }

  switch (phase) {
    case 0:
      Brake();
      if (ct - phaseStart >= stopTime) {
        phase = 1;
        phaseStart = ct;
      }
      break;

    case 1:
      Rear();
      if (ct - phaseStart >= rearTime) {
        phase = 2;
        phaseStart = ct;
      }
      break;

    case 2:

      TurnRight();

      if (ct - phaseStart >= turnTime) {
        phase = 3;
        phaseStart = ct;
      }
      break;

    case 3:
      firstTime = true;
      if (dist >= maxDist) {
        currentState = NORMAL;
      } else if (dist >= normalDist) {
        currentState = RALENTI;
      }
      Serial.println("Sortie État : DANGER");
      break;
  }
}

void rondeState(unsigned long ct) {
  static bool firstTime = true;
  static unsigned long previousTime = ct;
  int rondeDuration = 2000;
  int F_ledCycle = 500;
  int H_ledCycle = 250;
  if (firstTime) {
    firstTime = false;
    Serial.println("Entrée État : RONDE");
    previousTime = ct;
  }

  if ((ct - previousTime) % F_ledCycle < H_ledCycle) {
    for (int i = 0; i < LEDNUM; i++) {
      led.setColorAt(i, 0, 255, 0);
    }
  } else {

    for (int i = 0; i < LEDNUM; i++) {
      led.setColorAt(i, 0, 0, 0);
    }
  }
  led.show();


  if (ct - previousTime >= rondeDuration) {
    currentState = NORMAL;
    firstTime = true;
    rondeActive = false;
    Serial.println("Sortie État : RONDE");
  }

  if (dist <= normalDist) {
    currentState = DANGER;
    firstTime = true;
    rondeActive = false;
    Serial.println("Sortie État : RONDE");
  } else if (dist <= maxDist) {
    currentState = RALENTI;
    firstTime = true;
    rondeActive = false;
    Serial.println("Sortie État : RONDE");
  }
}


void stateManager(unsigned long ct) {
  switch (currentState) {
    case NORMAL:
      normalState(ct);
      break;
    case RALENTI:
      ralentiState();
      break;
    case DANGER:
      dangerState(ct);
      break;
    case RONDE:
      rondeState(ct);
      break;
    default:
      currentState = NORMAL;
  }
}


void NormalSpeedMode() {
  digitalWrite(m1_in2, HIGH);
  digitalWrite(m1_in1, LOW);
  analogWrite(m1_pwm, maxPwm);

  digitalWrite(m2_in2, HIGH);
  digitalWrite(m2_in1, LOW);
  analogWrite(m2_pwm, maxPwm);
}

void ReduceSpeed() {
  digitalWrite(m1_in2, HIGH);
  digitalWrite(m1_in1, LOW);
  analogWrite(m1_pwm, halfPwm);

  digitalWrite(m2_in2, HIGH);
  digitalWrite(m2_in1, LOW);
  analogWrite(m2_pwm, halfPwm);
}

void Brake() {
  digitalWrite(m1_in2, HIGH);
  digitalWrite(m1_in1, LOW);
  digitalWrite(m1_pwm, LOW);

  digitalWrite(m2_in2, HIGH);
  digitalWrite(m2_in1, LOW);
  digitalWrite(m2_pwm, LOW);
}

void Stop() {
  analogWrite(m1_pwm, 0);
  analogWrite(m2_pwm, 0);
  Serial.println("Stop");
}

void TurnRight() {
  digitalWrite(m1_in2, LOW);
  digitalWrite(m1_in1, HIGH);
  analogWrite(m1_pwm, turnPwm);

  digitalWrite(m2_in2, HIGH);
  digitalWrite(m2_in1, LOW);
  analogWrite(m2_pwm, halfPwm);
}

void Rear() {
  digitalWrite(m1_in2, LOW);
  digitalWrite(m1_in1, HIGH);
  analogWrite(m1_pwm, halfPwm);

  digitalWrite(m2_in2, LOW);
  digitalWrite(m2_in1, HIGH);
  analogWrite(m2_pwm, halfPwm);
}
