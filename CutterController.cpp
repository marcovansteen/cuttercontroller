#include <cstdint>
#include "CutterController.h"

#include <algorithm>
#include <cstdarg>
#include <cstdio>

// Initialize controller state from the provided configuration and bootstrap timing.
CutterController::CutterController(const CutterConfig& config)
  : config(config),
    controller_state(ControllerState::Geen),
    log_callback_(nullptr),
    logging_enabled_(false) {
  mesStand = false;
  worstLengteLeren = true;
  snijVertragingOp = config.snijVertragingOp;
  snijVertragingNeer = config.snijVertragingNeer;
  geleerdeLengte = 0.4; // standaard waarde van 40 cm voor het geval dat
}

// Sample sensors, timing, and latency feedback to determine whether a cut should fire and
// to update telemetry that downstream systems (MES, pneumatics) consume.
CutterResult CutterController::update(bool worstVoorS1, bool worstVoorS2, double nu) {

  CutterResult resultaat{};
  
  switch (controller_state) {

    case ControllerState::Geen:
      if (worstVoorS1 and !worstVoorS2) {
      // if (worstVoorS1) {
        beginVanWorstLaatstGezienDoorS1 = nu;
        transitionToState(ControllerState::AlleenS1);
      }
      break;
    
    case ControllerState::AlleenS1:
      if (worstVoorS2) {
        transitionToState(ControllerState::Beide);

        float beginVanWorstLaatstGezienDoorS2 = nu;
        float tijdTussenS1enS2 = beginVanWorstLaatstGezienDoorS2 - beginVanWorstLaatstGezienDoorS1;
        if (tijdTussenS1enS2 < 1e-3) {  // dit voorkomt delen door een heel klein getal
          logMessage("Diff '%d' is kleiner dan 1.0ms, afgerond naar 1.0ms", tijdTussenS1enS2);
          tijdTussenS1enS2 = 1e-3;
        }
        worstSnelheid = config.afstandTussenS1enS2 / tijdTussenS1enS2;
        logMessage("v               : %.3f m/s,", worstSnelheid);

        float dodeTijd;
        if (mesStand) {
          dodeTijd = snijVertragingNeer;
        } else {
          dodeTijd = snijVertragingOp;
        }
        
        // bereken snij tijdstip zodat de worst wordt doorgesneden zodra de voorkant bij "pijl" aankomt,
        // hou via 'dodeTijd' rekening met pneumatiek en mes mechaniek
        snijMoment = beginVanWorstLaatstGezienDoorS2 + config.afstandS2totPijl / worstSnelheid - dodeTijd;
        
        float teWachtenTijd = snijMoment - nu;
        if(teWachtenTijd > config.maxSnijInterval ) {   // dit bepaalt de langzaamste snelheid van de lijn !!!
          teWachtenTijd = config.maxSnijInterval;
        }
        logMessage("Wacht nog       : %.0f ms", 1000 * teWachtenTijd);
      }
      break;
    
    case ControllerState::Beide:
      if (nu >= snijMoment) {
        transitionToState(ControllerState::SnijCommandoVerstuurd);

        mesStand = !mesStand;
        // logMessage("Changing cutter state from %d to %d", !cutterState, cutterState);
      }
      break;
    
    case ControllerState::SnijCommandoVerstuurd:
      if (!worstVoorS1) {
        transitionToState(ControllerState::AlleenS2);

        float eindVanWorstLaatstGezienDoorS1 = nu;
        float tijdsduurWorstVoorS1 = eindVanWorstLaatstGezienDoorS1 - beginVanWorstLaatstGezienDoorS1;
        worstLengte = tijdsduurWorstVoorS1 * worstSnelheid;
        logMessage("Lengteschatting : %.2f mm\n", 1000 * worstLengte);
      }
      break;
    
    case ControllerState::AlleenS2:

      transitionToState(ControllerState::Geen);

      // todo: measure actual size of the string, based on the speed and time se wee S1 detect front and end of string.
      // logMessage("TODO: implement string length doublecheck calculation and adjust up/down latency of the cutter");
      break;
  }

  resultaat.mesStand = mesStand;
  resultaat.worstSnelheid = worstSnelheid;
  resultaat.snijMoment = snijMoment;
  resultaat.msTotSnijMoment = msTotSnijMoment;
  resultaat.snijVertragingOp = snijVertragingOp;
  resultaat.snijVertragingNeer = snijVertragingNeer;
  resultaat.worstLengte = worstLengte;
  resultaat.controller_state = static_cast<uint8_t>(controller_state);
  
  return resultaat;
}

const char* CutterController::controllerStateName(ControllerState state) {
  switch (state) {
    case ControllerState::Geen:
      return "Geen";
    case ControllerState::AlleenS1:
      return "AlleenS1";
    case ControllerState::Beide:
      return "Beide";
    case ControllerState::AlleenS2:
      return "AlleenS2";
    case ControllerState::SnijCommandoVerstuurd:
      return "SnijCommandoVerstuurd";
  }
  return "unknown";
}

void CutterController::transitionToState(ControllerState newState) {
  controller_state = newState;
  printf("* %s\n", controllerStateName(controller_state));
}

void CutterController::setLogCallback(LogCallback callback) {
  log_callback_ = callback;
}

void CutterController::enableLogging(bool enabled) {
  logging_enabled_ = enabled;
}

void CutterController::logMessage(const char* format, ...) const {
  if (!logging_enabled_ || !log_callback_ || !format) {
    return;
  }
  char buffer[256];
  va_list args;
  va_start(args, format);
  std::vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  log_callback_(buffer);

}
