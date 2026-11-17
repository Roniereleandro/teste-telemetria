#pragma once
#include <stdint.h>
#include <stddef.h>
namespace PortalBodyPolicy {
struct Budget {
 static constexpr size_t MAX_BYTES=8192;static constexpr uint32_t MAX_MS=8000;
 uint32_t started; size_t used=0;bool failed=false;
 explicit Budget(uint32_t now):started(now){}
 bool append(size_t n,uint32_t now){if(failed||uint32_t(now-started)>MAX_MS||n>MAX_BYTES-used){failed=true;return false;}used+=n;return true;}
};
}
