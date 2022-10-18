
#pragma once

// TODO: find a cleaner way or simplify to one file
#define NUCLEO_BUILD
//#define MYCO_BUILD
#ifdef NUCLEO_BUILD
  #include "nucleo_Configuration_adv.h"
#elif defined(MYCO_BUILD)
  #include "myco_Configuration_adv.h"
#endif