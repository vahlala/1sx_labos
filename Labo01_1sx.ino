//NUEKUMO SIMO ARTHUR DURAND
#include <HCSR04.h>             
#include <Adafruit_GFX.h>       
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

HCSR04 hc(8, 9);  

unsigned long lastDistTime = 0;
unsigned long lastAnimTime = 0;

float distance ;     
float lastDistance;  
int sunY ;            


const char* nom = "Arthur";  
     
enum Task { READ_DIST, UPDATE_ANIM, UPDATE_DISPLAY };
Task currentTask = READ_DIST;

void dessinerMaison() {
  display.drawRect(10, 30, 30, 20, SSD1306_WHITE);   
  display.drawTriangle(10, 30, 40, 30, 25, 15, SSD1306_WHITE); 
}

void dessinerSoleil(int x, int y) {
  display.drawCircle(x, y, 7, SSD1306_WHITE);  
}

void distTask(unsigned long ct) {
  const unsigned long rate = 250;
  if (ct - lastDistTime >= rate) {
    lastDistTime = ct;

    float d = hc.dist();
    if (d > 0 && d < 200) {
      distance = d;
      lastDistance = d;
    } else {
      distance = lastDistance;
    }

    Serial.print("Distance : ");
    Serial.print(distance);
    Serial.println(" cm");
  }
}


void animationTask(unsigned long ct) {
  const unsigned long rate = 100;
  if (ct - lastAnimTime >= rate) {
    lastAnimTime = ct;
    int maxDist = 50;
    int minDist = 10;
    int minY = 40;  
    int maxY = 10;  
    
    if (distance >=minDist && distance<=maxDist){
    int mappedY = map(distance, minDist, maxDist, minY, maxY);
    sunY = mappedY;
    }
  }
}

void displayTask() {
  display.clearDisplay();
  dessinerMaison();
  dessinerSoleil(100, sunY);
  display.setCursor(0, SCREEN_HEIGHT - 8);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.print(nom);
  display.display();
}

void setup() {
  Serial.begin(9600);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("Erreur OLED SSD1306");
  }
  display.clearDisplay();
  display.display();
}


void loop() {
  unsigned long ct = millis();

  switch (currentTask) {
    case READ_DIST:
      distTask(ct);
      currentTask = UPDATE_ANIM;
      break;

    case UPDATE_ANIM:
      animationTask(ct);
      currentTask = UPDATE_DISPLAY;
      break;

    case UPDATE_DISPLAY:
      displayTask();
      currentTask = READ_DIST;
      break;
  }
}
