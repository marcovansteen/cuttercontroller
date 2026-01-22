#include <cstdint>
#include "CutterController.h"
#include <cstdarg>
#include <cstdio>

// ---------------------------------------------------------------------------------------------
// Initialize controller state from the provided configuration and bootstrap timing.
CutterController::CutterController(const CutterConfig& config): config(config), controller_state(ControllerState::Geen), log_callback_(nullptr), logging_enabled_(false) {
  mesStand = false;
  autoHoldActief= false;
  snijVertragingOp = config.snijVertragingOpInit;
  snijVertragingNeer = config.snijVertragingNeerInit;
  geleerdeLengte = 0.4; // standaard waarde van 40 cm voor het geval dat
}


// ---------------------------------------------------------------------------------------------
// Sample sensors, timing, and latency feedback to determine whether a cut should fire and
// to update telemetry that downstream systems (MES, pneumatics) consume.
CutterResult CutterController::update(bool worstVoorS1, bool worstVoorS2, double nu) {

  CutterResult resultaat{};
 
  static bool prevWorstVoorS1 = false;
  static bool prevWorstVoorS2 = false;
  // detect rising edges on sensors
  if (worstVoorS1 && !prevWorstVoorS1) {
    beginVanWorstLaatstGezienDoorS1 = nu;
//    logMessage("\nS1 rising edge at %.3f s. ", nu);
    transitionToState(ControllerState::AlleenS1, nu);
  }
  
  if (worstVoorS2 && !prevWorstVoorS2) {
  //  logMessage("\nS2 rising edge at %.3f s. ", nu);
    if (controller_state == ControllerState::AlleenS1) {
      float beginVanWorstLaatstGezienDoorS2 = nu;
      float tijdTussenS1enS2 = beginVanWorstLaatstGezienDoorS2 - beginVanWorstLaatstGezienDoorS1;
      if (tijdTussenS1enS2 < 1e-3) {  // dit voorkomt delen door een heel klein getal
        logMessage("Diff '%.3f' is kleiner dan 1.0ms, afgerond naar 1.0ms", tijdTussenS1enS2);
        tijdTussenS1enS2 = 1e-3;
      } 
      worstSnelheid = config.afstandTussenS1enS2 / tijdTussenS1enS2;
      logMessage("\nv: %.3f m/s, ", worstSnelheid);

      float dodeTijd;
      if (mesStand) {
        dodeTijd = snijVertragingNeer;    // aangezien de autohold regelaar hier ingrijpt,
      } else {                            // kan de dodetijd ook negatief worden !
        dodeTijd = snijVertragingOp;      // 
      }

      // bereken snij tijdstip zodat de worst wordt doorgesneden zodra de voorkant bij "pijl" aankomt,
      // hou via 'dodeTijd' rekening met pneumatiek, mes mechaniek en autohold bijsturing
      snijMoment = beginVanWorstLaatstGezienDoorS2 + config.afstandS2totReferentie / worstSnelheid + dodeTijd;
      
      float teWachtenTijd = snijMoment - nu;
      if(teWachtenTijd > config.maxTeWachtenTijd ) {     // Dit bepaalt de laagst mogelijke snelheid, en de maximale worstlengte.
        teWachtenTijd = config.maxTeWachtenTijd;
        snijMoment = nu + config.maxTeWachtenTijd;       // Is aan te passen in CutterController.h !!!
      }
      
      logMessage("Wacht: %.0f ms, ", 1000 * teWachtenTijd);
      transitionToState(ControllerState::Beide, nu);
    }
  }

  if(nu >= snijMoment && controller_state == ControllerState::Beide) {
    transitionToState(ControllerState::SnijCommandoVerstuurd, nu);
    mesStand = !mesStand;

    updateAutoHold();
  }

  if (!worstVoorS1 && prevWorstVoorS1) {
//    logMessage("\nS1 falling edge at %.3f s. ", nu);
    if (controller_state == ControllerState::SnijCommandoVerstuurd) {
      transitionToState(ControllerState::AlleenS2, nu);

      float eindVanWorstLaatstGezienDoorS1 = nu;
      float tijdsduurWorstVoorS1 = eindVanWorstLaatstGezienDoorS1 - beginVanWorstLaatstGezienDoorS1;
      worstLengte = tijdsduurWorstVoorS1 * worstSnelheid;
      logMessage("Worstlengte: %.1f mm. ", 1000 * worstLengte);
      leerLengte(worstLengte);
    }
  }

  prevWorstVoorS2 = worstVoorS2;
  prevWorstVoorS1 = worstVoorS1; 

  // time-out, voor het geval dat
  if (nu - tijdLaatsteStateWijziging > config.maxTeWachtenTijd * 2) { 
    logMessage("** State timeout, terug naar Geen **\n");
    transitionToState(ControllerState::Abnormaal, nu);
    autoHoldActief = false;
  }

  resultaat.mesStand = mesStand;
  resultaat.worstSnelheid = worstSnelheid;
  resultaat.snijMoment = snijMoment;
  resultaat.snijVertragingOp = snijVertragingOp;
  resultaat.snijVertragingNeer = snijVertragingNeer;
  resultaat.worstLengte = worstLengte;
  resultaat.controller_state = static_cast<uint8_t>(controller_state);
  
  return resultaat;
}

// ---------------------------------------------------------------------------------------------
// verwerk nieuw gemeten lengte
void CutterController::leerLengte(float nieuwe_lengte) {

  // bewaar laatst gemeten lengte in circulaire buffer
  lengte[lengteIdx++] = nieuwe_lengte;
  if(lengteIdx >= config.aantalLengtes) {
    lengteIdx = 0;
    lengteBufferVol = true;
  }

  // bereken spreiding en gemiddelde van laatste lengtes
  float min = 10000;
  float max = -1;
  float totaleLengte = 0;

  if (lengteBufferVol) {
    logMessage("Buffer: ");
    for (uint8_t i = 0; i < config.aantalLengtes; i++) {
      logMessage("%.1f, ", 1000 * lengte[i]);
      totaleLengte += lengte[i];  // voor bepaling gemiddelde
      if(lengte[i] > max) max = lengte[i];    // voor bepaling spreiding
      if(lengte[i] < min) min = lengte[i];
    }
    autoHoldSpreiding = max - min;
    autoHoldGemiddeldeLengte = totaleLengte / config.aantalLengtes;
    logMessage("max: %.1f mm, min: %.1f mm, spreiding: %.1f mm, gem.: %.1f mm, snijVertragingOp: %.3f s, snijVertragingNeer: %.3f s.", 1000 * max, 1000 * min, 1000 * autoHoldSpreiding, 1000 * autoHoldGemiddeldeLengte, snijVertragingOp, snijVertragingNeer);
  }

  updateAutoHoldTiming(nieuwe_lengte);

}

// ---------------------------------------------------------------------------------------------
// schakelt autoHold aan/uit
void CutterController::updateAutoHold() {

  if(!config.autoHoldMogelijk) return;

  if(!autoHoldActief) {
    if(autoHoldSpreiding < config.autoHoldAanDrempel) {   // schakel auto-hold in
      autoHoldActief = true;
      autoHoldDoelLengte = autoHoldGemiddeldeLengte;
      logMessage(" ** AUTOHOLD AAN (doel: %.2f mm) ** ", 1000 * autoHoldDoelLengte);
    }
  } else { 
    if(autoHoldSpreiding > config.autoHoldUitDrempel) {   // schakel auto-hold uit
      autoHoldActief= false;        // reset buffer om na afwijking te snel terug inschakelen te voorkomen
      lengteBufferVol = false;
      lengteIdx = 0;
      snijVertragingOp = config.snijVertragingOpInit;
      snijVertragingNeer = config.snijVertragingNeerInit;

      logMessage(" ** AUTOHOLD UIT ** ");
    }
  }
}

// ---------------------------------------------------------------------------------------------
// past timing aan bij autohold
void CutterController::updateAutoHoldTiming(float nieuweLengte) {

  // Een PID regeling zou te gek zijn maar afregelen daarvan is lastig.
  // Vandaar een stomme op/neer regeling. Worst te lang? Tandje terug.
  // En omgekeerd. Dit apart regelen voor mes omhoog en omlaag. 
  if(autoHoldActief) {

    float tijdStap;

    // tijdStap = config.autoHoldBijRegelStap / worstSnelheid;
    tijdStap = (autoHoldDoelLengte - nieuweLengte) / worstSnelheid;

    if(mesStand) {
      // if(nieuweLengte > autoHoldDoelLengte) snijVertragingOp -= tijdStap; else snijVertragingOp += tijdStap;
      snijVertragingOp += tijdStap;
    } else {
      // if(nieuweLengte > autoHoldDoelLengte) snijVertragingNeer -= tijdStap; else snijVertragingNeer += tijdStap;
      snijVertragingNeer += tijdStap;
    }
      
    // begrens de regelaar om puinhoop in de fabriek te voorkomen bij op hol geslagen regeling
    if(snijVertragingOp < config.minSnijVertraging)   snijVertragingOp = config.minSnijVertraging;
    if(snijVertragingOp > config.maxSnijVertraging)   snijVertragingOp = config.maxSnijVertraging;
    if(snijVertragingNeer < config.minSnijVertraging) snijVertragingNeer = config.minSnijVertraging;
    if(snijVertragingNeer > config.maxSnijVertraging) snijVertragingNeer = config.maxSnijVertraging; 
  }

}

// ---------------------------------------------------------------------------------------------
// helper-functie voor logging: zet state om in leesbare tekst
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
    case ControllerState::Abnormaal:
      return "Abnormaal";
  }
  return "unknown";
}

// ---------------------------------------------------------------------------------------------
// ga naar volgende state
void CutterController::transitionToState(ControllerState nieuweState, double nu) {
  controller_state = nieuweState;
  tijdLaatsteStateWijziging = nu;
  // logMessage("* %s", controllerStateName(controller_state));
}

// ---------------------------------------------------------------------------------------------
// stel functie in die logging uitprint
void CutterController::setLogCallback(LogCallback callback) {
  log_callback_ = callback;
}

// ---------------------------------------------------------------------------------------------
// zet logging aan of uit
void CutterController::enableLogging(bool enabled) {
  logging_enabled_ = enabled;
}

// ---------------------------------------------------------------------------------------------
// schrijf boodschap voor diagnostiek naar seriele poort
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
