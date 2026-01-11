#ifndef CUTTER_CONTROLLER_H
#define CUTTER_CONTROLLER_Hc

// #include <Arduino.h>
#include <cstdint>
#include <cstddef>
#include <cstdarg>

// Runtime parameters die lengte en timing bepalen
struct CutterConfig {
   
  float afstandS2totPijl = 0.065f;    // De vaste afstand van S2 tot de pijl, in meters, hier noteren.
  float afstandTussenS1enS2 = 0.05f;  // Afstand tussen de twee sensoren, gebruikt om de invoersnelheid te schatten.
  double snijVertragingOp = 0.05;     // De tijd tussen opgaande stuurflank en mes (laag -> hoog) [s]
  double snijVertragingNeer = 0.07;   // De tijd tussen neergaande stuurflank en mes (laag -> hoog) [s]
  double maxLengteCorrectie = 100.0;  // Maximale lengtecorrectie die per snede mag worden toegepast.
  double minSnijInterval = 0.05;      // Minimum toegestane interval tussen snedes om overruns te voorkomen.
  double maxSnijInterval = 4.0;       // Maximum toegestane interval tussen snedes. Dit bepaalt de langzaamste snelheid van de lijn !!!

};

// Output state waarmee de aanroepende functie bv het mes kan besturen
struct CutterResult {

  bool mesStand;
  float worstSnelheid;
  float snijMoment;
  float msTotSnijMoment;
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
  };

  void logMessage(const char* format, ...) const;

  CutterConfig config;

  ControllerState controller_state;
  LogCallback log_callback_;
  bool logging_enabled_;

  void transitionToState(ControllerState next_state);

  static const char* controllerStateName(ControllerState state);

  bool mesStand;
  float worstSnelheid;
  float snijMoment;
  float msTotSnijMoment;
  float snijVertragingOp;
  float snijVertragingNeer;
  float worstLengte;

  bool worstLengteLeren;
  float geleerdeLengte;
  float tijdCorrectieOmlaag;
  float tijdCorrectieOmhoog;
  float beginVanWorstLaatstGezienDoorS1;
  
};

#endif  // CUTTER_CONTROLLER_H
