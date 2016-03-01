//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: attitude_controller.h
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 21-Sep-2015 14:29:20
//
#ifndef __ATTITUDE_CONTROLLER_H__
#define __ATTITUDE_CONTROLLER_H__

// Include Files
#include <math.h>
#include <stddef.h>
#include <stdlib.h>

#include "rtwtypes.h"
#include "attitude_controller_types.h"

// Function Declarations
extern void attitude_controller(const float R_B[9], float yaw, const float
  F_des[3], const float Kp[3], short kumar_variant, float rates_des[3], float
  *thrust);
extern void attitude_controller_initialize();
extern void attitude_controller_terminate();

#endif

//
// File trailer for attitude_controller.h
//
// [EOF]
//
