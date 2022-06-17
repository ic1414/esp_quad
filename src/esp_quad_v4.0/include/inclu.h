#ifndef _inclu_h_
#define _inclu_h_

/*
<SimpleKalmanFilter.h>
-author
Denys Sene
https://github.com/denyssene/SimpleKalmanFilter

*/

enum PIDD{P, I, D};
enum XYZT{X, Y, Z, T};
enum V1234{V1, V2, V3, V4};
enum ABCD{mA, mB, mC, mD};

#include <Arduino.h>
#include <SimpleKalmanFilter.h>
#include "pid.h"
#include "config.h"
#include "mpu.h"
#include "vtg.h"
#include "motor.h"
#include "ble.h"
#include "reciever.h"
#include "seriale.h"

#endif