#pragma once
#include <stdint.h>
namespace PortalLatencyPolicy {
constexpr uint32_t POLL_MS=1000;
constexpr uint32_t ACK_PRIORITY_MS=500;
constexpr bool fetchDue(uint32_t now,uint32_t last,bool ackPending) {
  return uint32_t(now-last)>=POLL_MS &&
         (!ackPending || uint32_t(now-last)>=POLL_MS+ACK_PRIORITY_MS);
}
constexpr bool postAllowed(uint32_t now,uint32_t finished,bool previous,bool success) {
  return !previous || uint32_t(now-finished)>=(success?250UL:1000UL);
}
constexpr bool reuseAllowed(bool connected,bool sameOrigin,bool sameNetwork,uint32_t idle,uint32_t age) {
  return connected && sameOrigin && sameNetwork && idle<10000UL && age<60000UL;
}
}
