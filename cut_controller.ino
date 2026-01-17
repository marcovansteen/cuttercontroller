/*
  cut-controller.ino

  - Leest sensors uit
  - Toggelt het mes (=de grote groene LED)
  - Geeft status weer op seriële monitor.

*/

#define S1 10   // arduino pin, eerste sensor die de worst ziet
#define S2 11   // arduino pin, tweede sensor die de worst ziet
#define MES 12  // uitgang: zowel laag-->hoog als hoog-->laag snijdt worst door

#include "CutterController.h"

CutterController cutterController;

void controller_log_callback(const char* message) {
  Serial.println(message);
}

// deze functie wordt alleen uitgevoerd bij reset, en bij het inschakelen van de spanning
void setup() {
  // stel de arduino pinnen in als in- of uitgang
  pinMode(MES, OUTPUT);
  pinMode(S1, INPUT_PULLUP);
  pinMode(S2, INPUT_PULLUP);

  Serial.begin(115200);
  cutterController.setLogCallback(controller_log_callback);
  cutterController.enableLogging(true);
}

// oneindige lus
void loop() { 

  bool worstVoorS1 = digitalRead(S1) == LOW;  // deeg voor sensor geeft "LOW", geen deeg = "HIGH"
  bool worstVoorS2 = digitalRead(S2) == LOW;  // deeg voor sensor geeft "LOW", geen deeg = "HIGH"
  double nu = (double) micros() / 1000000;   // in seconden

  CutterResult resultaat = cutterController.update(worstVoorS1, worstVoorS2, nu);

  digitalWrite(MES, resultaat.mesStand ? HIGH : LOW);

}
