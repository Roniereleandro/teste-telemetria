#pragma once
namespace UplinkPolicy {
enum Transport { NONE, WIFI, CABLE };
constexpr Transport choose(bool cableReady, bool wifiReady) {
  return cableReady ? CABLE : wifiReady ? WIFI : NONE;
}
}
