#define ENABLE_VHACD_IMPLEMENTATION 1
#include <VHACD.h>

#include <string>

extern "C" {
  void printNumber(int num);
}

void printNumber(int num) {
  const std::string numS = std::to_string(num);
  printf("num is %s\n", numS.c_str());
}
