#pragma once
#include <stdint.h>
#include "UplinkPolicy.h"
namespace UplinkHealthPolicy {
using UplinkPolicy::Transport;using UplinkPolicy::NONE;using UplinkPolicy::WIFI;using UplinkPolicy::CABLE;
struct State {
 static constexpr uint32_t MIN_HOLD_MS=15000, PREFERRED_PROBE_MS=120000;
 static constexpr uint8_t FAILURE_LIMIT=3;
 Transport active=NONE; uint8_t failures[3]={}; uint32_t switchedAt=0,switches=0;
 void result(Transport t,int httpCode){if(t==NONE)return;if(httpCode>0)failures[t]=0;else if(failures[t]<255)++failures[t];}
 Transport choose(uint32_t now,bool cable,bool wifi)const {
  if(!cable&&!wifi)return NONE;
  if(active==CABLE&&cable){if(wifi&&failures[CABLE]>=FAILURE_LIMIT&&uint32_t(now-switchedAt)>=MIN_HOLD_MS)return WIFI;return CABLE;}
  if(active==WIFI&&wifi){if(cable&&((failures[WIFI]>=FAILURE_LIMIT&&uint32_t(now-switchedAt)>=MIN_HOLD_MS)||uint32_t(now-switchedAt)>=PREFERRED_PROBE_MS))return CABLE;return WIFI;}
  return cable?CABLE:WIFI;
 }
 void commit(Transport t,uint32_t now){if(t==active)return;active=t;switchedAt=now;++switches;if(t!=NONE)failures[t]=0;}
};
}
