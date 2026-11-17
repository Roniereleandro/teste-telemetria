#pragma once
#include <stdint.h>
#include <math.h>
namespace CommandPolicy {
enum Result { NONE, RECEIVED, ACCEPTED, EXECUTING, SUCCEEDED, REJECTED, FAILED, IGNORED };
constexpr bool rejected(Result r) { return r==REJECTED || r==FAILED || r==IGNORED; }
constexpr bool accepted(bool sameCommand, Result r) {
  return sameCommand && (r==ACCEPTED || r==EXECUTING || r==SUCCEEDED);
}
constexpr bool needsGps(bool required, bool noGps) {
  return required || !noGps; // sem dispensa explicita, exigir GPS; conflito tambem exige GPS
}
constexpr bool gpsStartReady(bool available, bool radioFix, uint32_t sampleAge,
                          uint32_t sats, double hdop, bool validAngle, double distance) {
  return available && radioFix && sampleAge<=15000UL && sats>=5 &&
         isfinite(hdop) && hdop>0 && hdop<=4.0 && validAngle &&
         isfinite(distance) && distance>=10.0;
}
}
