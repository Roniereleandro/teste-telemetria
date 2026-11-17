#pragma once
#include <stdint.h>
namespace RadioProtocolPolicy {
inline bool validAck(uint32_t peerBoot,uint32_t ownBoot,uint32_t ack,uint32_t sent,uint32_t lastAck){return ownBoot!=0&&peerBoot==ownBoot&&ack!=0&&ack==sent&&(lastAck==0||int32_t(ack-lastAck)>0);}
inline bool txDue(uint32_t now,uint32_t last,bool sent,uint32_t jitter){return uint32_t(now-last)>=(sent?3000UL+(jitter%201UL):1800UL);}
}
