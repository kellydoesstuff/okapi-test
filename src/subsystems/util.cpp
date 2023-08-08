#include "main.h"

namespace util {
  
  double clip_num(double input, double max, double min) {
    if (input > max)
      return max;
    else if (input < min)
      return min;
    return input;
  }
  
  int sgn(double input) {
    if (input > 0)
      return 1;
    else if (input < 0)
      return -1;
    return 0;
  }
}