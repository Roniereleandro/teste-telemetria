#pragma once
#include <ArduinoJson.h>
namespace PortalResponsePolicy {
inline bool successCompatible(JsonVariantConst root){return root.is<JsonObjectConst>()&&(!root["success"].isUnbound()?root["success"].is<bool>()&&root["success"].as<bool>():true);}
inline bool commandEnvelope(JsonVariantConst root){return successCompatible(root)&&root["items"].is<JsonArrayConst>();}
inline bool heartbeatEnvelope(JsonVariantConst root){return successCompatible(root)&&(root["success"].is<bool>()?root["success"].as<bool>():root["ackEventIds"].is<JsonArrayConst>());}
}
