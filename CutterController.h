#ifndef CUTTER_CONTROLLER_H
#define CUTTER_CONTROLLER_Hc

// #include <Arduino.h>
#include <cstdint>
#include <cstddef>
#include <cstdarg>

//---------------------------
// CutterController.h
// Dit is de interface naar de code die beslist wanneer het mes moet worden bediend.
// De werkelijke code is te vinden in CutterController.cpp
// De worst passeert eerst sensor S1, dan sensor S2 en daarna de Referentie positie.
// De Referentie positie is een markering op of aan de sensor beugel, die zichtbaar is voor de operator.
//
// Het regelalgoritme heeft twee modi: Autohold uit en Autohold aan
//---------------------------

// Runtime parameters die lengte en timing bepalen.
// Alle eenheden zijn volgens S.I., dus meters, seconden en meters per seconde. 
// Bv. een afstand van 0.05 is 5 cm, en een tijd van 0.02 is 20 milliseconden.
struct CutterConfig {
   
  const bool autoHoldMogelijk = true;         // Zet deze op  = false (alleen kleine letters) om autohold uit te schakelen (org true)
  const float afstandS2totReferentie = 0.065; // In meters. De vaste afstand van S2 tot de referentie hier noteren. Moet groter dan 0 zijn !!(org 0.065)
  const float afstandTussenS1enS2 = 0.050;    // In meters. Afstand tussen de twee sensoren, gebruikt om de invoersnelheid te schatten.(org 0.050)
  const double snijVertragingOpInit = 0.03;   // In seconden. De tijd tussen opgaande stuurflank en mes (laag -> hoog), na inschakelen. (org 0.03)
  const double snijVertragingNeerInit = 0.03; // In seconden. De tijd tussen neergaande stuurflank en mes (laag -> hoog), na inschakelen.(org 0.03)
  const double maxLengteCorrectie = 100.0;    // In meters. Maximale lengtecorrectie die per snede mag worden toegepast. (org 100.0)
  const double maxTeWachtenTijd = 4.0;        // In seconden. Maximum toegestane interval tussen snedes. Dit bepaalt de laagst mogelijke snelheid van de lijn !!! (org 2.0)
  const uint8_t aantalLengtes = 3;            // Over dit aantal lengtes wordt bepaald of lengte Constant is (voor auto-hold). Bij meer dan 100, dat getal ook verderop aanpassen in de regel die "lengtes[100]" bevat. (org 3)
  float minSnijVertraging = -0.4;             // In seconden, autohold regelt niet lager dan deze tijd terug (org -0.4)
  float maxSnijVertraging = 0.4;              // In seconden, autohold regelt niet hoger dan deze tijd op (org 0.4)
  const float autoHoldBijRegelStap = 0.001;   // In seconden per "even" worst per stap. (org 0.001)
  const float autoHoldAanDrempel = 0.05;      // In meters. Als gemeten spreiding kleiner is dan deze lengte dan springt 'autohold' aan. (org 0.05)
  const float autoHoldUitDrempel = 0.10;      // In meters. Als gemeten spreiding over laatste AantalLengtes groter is dan deze drempel, springt 'autohold'
 
};

// Output state waarmee de aanroepende functie bv het mes kan besturen
struct CutterResult {

  bool mesStand;
  float worstSnelheid;
  float snijMoment;
  float snijVertragingOp;
  float snijVertragingNeer;
  float worstLengte;
  uint8_t controller_state;

};

// Stateful controller that shapes cut timing based on sensor feedback and latency estimates.
class CutterController {
public:
  explicit CutterController(const CutterConfig& config = CutterConfig());
  using LogCallback = void (*)(const char* message);

  CutterResult update(bool sensor1Active, bool sensor2Active, double now);
  // Expose the MES bit state for debugging or telemetry reporting.
  uint8_t currentCutterState() const {
    return mesStand;
  }
  // Attach an optional logging callback that receives formatted messages.
  void setLogCallback(LogCallback callback);
  void enableLogging(bool enabled);

private:
  enum class ControllerState {
    Geen,
    AlleenS1,
    Beide,
    AlleenS2,
    SnijCommandoVerstuurd,
    Abnormaal
  };

  void logMessage(const char* format, ...) const;
  CutterConfig config;
  ControllerState controller_state;
  LogCallback log_callback_;
  bool logging_enabled_;
  void transitionToState(ControllerState nieuweState, double nu);
  static const char* controllerStateName(ControllerState state);

  void leerLengte(float lengte);            // verwerkt nieuw gemeten lengte
  float lengte[100];                        // circulaire buffer
  void updateAutoHoldTiming(float lengte);  // past timing aan bij autohold
  void updateAutoHold();                    // schakelt autoHold aan/uit
  float autoHoldSpreiding = 0.1;            // laatst gemeten spreiding
  float autoHoldGemiddeldeLengte = 0.4;     // laatst bepaalde gemiddelde
  float autoHoldDoelLengte = 0.4;           // doel lengte
  bool autoHoldActief = false;
  float snijVertragingOp;
  float snijVertragingNeer;
  float geleerdeLengte;
  double tijdLaatsteStateWijziging = 0.0;   // voor time-out
  
  bool lengteBufferVol = false;             // geeft aan of er al voldoende metingen ontvangen zijn
  uint8_t lengteIdx = 0;
    
  bool mesStand;
  float worstSnelheid;
  float snijMoment;
  float msTotSnijMoment;

  float worstLengte;

  float beginVanWorstLaatstGezienDoorS1;
  
};

#endif  // CUTTER_CONTROLLER_H
