#include "main.h"

namespace util {
  
  double clip_num(double input, double max, double min) {
    if (input > max)
      return max;
    else if (input < min)
      return min;
    return input;
  }
  
  const int DELAY_TIME{10}; // in ms
}