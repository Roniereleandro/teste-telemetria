#pragma once
#include <stdlib.h>
#include <math.h>
namespace AngularTargetPolicy {
inline bool parse(const char* text, double& target) {
  if (!text || !*text) return false;
  char* end=nullptr;
  double value=strtod(text,&end);
  if (end==text) return false;
  while (*end==' ' || *end=='\t' || *end=='\r' || *end=='\n') ++end;
  if (*end || !isfinite(value) || value<0.0 || value>=360.0) return false;
  target=value;
  return true;
}
}
