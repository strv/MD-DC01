#include "main.h"

const static CtrlConfs MtConfDefault =
{
  1.f * (1<<CtrlQ),
  1.0f * (1<<CtrlQ),
  0.1f * (1<<CtrlQ),
  10,
  300,
  3000,
  3000
};

// super high gain for super low velocity
const static CtrlConfs MtConfHighGain =
{
  1.f * (1<<CtrlQ),
  50.0f * (1<<CtrlQ),
  1.0f * (1<<CtrlQ),
  300,
  12000,
  3000
};
