#pragma once
#include <stdint.h>
namespace MaintenancePolicy {
struct Window {
 static constexpr uint32_t DURATION_MS=900000;
 bool opened=false;uint32_t openedAt=0;
 bool active(uint32_t now)const{return opened&&uint32_t(now-openedAt)<DURATION_MS;}
 bool open(uint32_t now,bool safelyStopped){if(!safelyStopped)return false;openedAt=now;opened=true;return true;}
 void close(){opened=false;}
 uint32_t remaining(uint32_t now)const{return active(now)?DURATION_MS-uint32_t(now-openedAt):0;}
};
}
