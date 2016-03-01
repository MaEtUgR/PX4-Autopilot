//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: attitude_controller.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 21-Sep-2015 14:29:20
//

// Include Files
#include "rt_nonfinite.h"
#include "attitude_controller.h"

#include "rt_nonfinite.h"

// Function Declarations
static float norm(const float x[3]);

// Function Definitions

//
// Arguments    : const float x[3]
// Return Type  : float
//
static float norm(const float x[3])
{
  float y;
  float scale;
  int k;
  float absxk;
  float t;
  y = 0.0F;
  scale = 1.17549435E-38F;
  for (k = 0; k < 3; k++) {
    absxk = (real32_T)fabs((real_T)x[k]);
    if (absxk > scale) {
      t = scale / absxk;
      y = 1.0F + y * t * t;
      scale = absxk;
    } else {
      t = absxk / scale;
      y += t * t;
    }
  }

  return scale * (real32_T)sqrt((real_T)y);
}

//
// R_B=R_B'; % why this is transposed ???
// Arguments    : const float R_B[9]
//                float yaw
//                const float F_des[3]
//                const float Kp[3]
//                short kumar_variant
//                float rates_des[3]
//                float *thrust
// Return Type  : void
//
void attitude_controller(const float R_B[9], float yaw, const float F_des[3],
  const float Kp[3], short kumar_variant, float rates_des[3], float *thrust)
{
  float B;
  float z_B_des[3];
  int i;
  float x_C_des[3];
  float y_C_des[3];
  float b_z_B_des[3];
  float b_y_C_des[3];
  float R_des1[9];
  float R_des2[9];
  int i0;
  float y[9];
  int i1;
  float mtrc;
  float b_mtrc;
  float f0;
  float f1;
  float b_R_B[9];
  float b_R_des1[3];
  B = norm(F_des);
  for (i = 0; i < 3; i++) {
    z_B_des[i] = F_des[i] / B;
  }

  x_C_des[0] = (real32_T)cos((real_T)yaw);
  x_C_des[1] = (real32_T)sin((real_T)yaw);
  y_C_des[0] = (real32_T)cos((real_T)(yaw + 1.57079637F));
  y_C_des[1] = (real32_T)sin((real_T)(yaw + 1.57079637F));
  if (kumar_variant != 0) {
    b_z_B_des[0] = z_B_des[1] * 0.0F - z_B_des[2] * x_C_des[1];
    b_z_B_des[1] = z_B_des[2] * x_C_des[0] - z_B_des[0] * 0.0F;
    b_z_B_des[2] = z_B_des[0] * x_C_des[1] - z_B_des[1] * x_C_des[0];
    B = norm(b_z_B_des);
    y_C_des[0] = (z_B_des[1] * 0.0F - z_B_des[2] * x_C_des[1]) / B;
    y_C_des[1] = (z_B_des[2] * x_C_des[0] - z_B_des[0] * 0.0F) / B;
    y_C_des[2] = (z_B_des[0] * x_C_des[1] - z_B_des[1] * x_C_des[0]) / B;
    x_C_des[0] = y_C_des[1] * z_B_des[2] - y_C_des[2] * z_B_des[1];
    x_C_des[1] = y_C_des[2] * z_B_des[0] - y_C_des[0] * z_B_des[2];
    x_C_des[2] = y_C_des[0] * z_B_des[1] - y_C_des[1] * z_B_des[0];
  } else {
    b_y_C_des[0] = y_C_des[1] * z_B_des[2] - 0.0F * z_B_des[1];
    b_y_C_des[1] = 0.0F * z_B_des[0] - y_C_des[0] * z_B_des[2];
    b_y_C_des[2] = y_C_des[0] * z_B_des[1] - y_C_des[1] * z_B_des[0];
    B = norm(b_y_C_des);
    x_C_des[0] = (y_C_des[1] * z_B_des[2] - 0.0F * z_B_des[1]) / B;
    x_C_des[1] = (0.0F * z_B_des[0] - y_C_des[0] * z_B_des[2]) / B;
    x_C_des[2] = (y_C_des[0] * z_B_des[1] - y_C_des[1] * z_B_des[0]) / B;
    b_z_B_des[0] = z_B_des[1] * x_C_des[2] - z_B_des[2] * x_C_des[1];
    b_z_B_des[1] = z_B_des[2] * x_C_des[0] - z_B_des[0] * x_C_des[2];
    b_z_B_des[2] = z_B_des[0] * x_C_des[1] - z_B_des[1] * x_C_des[0];
    B = norm(b_z_B_des);
    y_C_des[0] = (z_B_des[1] * x_C_des[2] - z_B_des[2] * x_C_des[1]) / B;
    y_C_des[1] = (z_B_des[2] * x_C_des[0] - z_B_des[0] * x_C_des[2]) / B;
    y_C_des[2] = (z_B_des[0] * x_C_des[1] - z_B_des[1] * x_C_des[0]) / B;
  }

  for (i = 0; i < 3; i++) {
    R_des1[i] = x_C_des[i];
    R_des1[3 + i] = y_C_des[i];
    R_des1[6 + i] = z_B_des[i];
  }

  // r1 = vrrotmat2vec(R_des1' * R_B);
  // angle_err1 = acos(0.5 * ((trace(R_des1) - 1)));
  for (i = 0; i < 3; i++) {
    for (i0 = 0; i0 < 3; i0++) {
      y[i + 3 * i0] = 0.0F;
      for (i1 = 0; i1 < 3; i1++) {
        y[i + 3 * i0] += R_des1[i1 + 3 * i] * R_B[i1 + 3 * i0];
      }
    }

    R_des2[i] = -x_C_des[i];
    R_des2[3 + i] = -y_C_des[i];
    R_des2[6 + i] = z_B_des[i];
  }

  mtrc = (y[0] + y[4]) + y[8];

  // r2 = vrrotmat2vec(R_des2' * R_B);
  // angle_err2 = acos(0.5 * ((trace(R_des2) - 1)));
  for (i = 0; i < 3; i++) {
    for (i0 = 0; i0 < 3; i0++) {
      y[i + 3 * i0] = 0.0F;
      for (i1 = 0; i1 < 3; i1++) {
        y[i + 3 * i0] += R_des2[i1 + 3 * i] * R_B[i1 + 3 * i0];
      }
    }
  }

  b_mtrc = (y[0] + y[4]) + y[8];

  // angle_err1 = r1(4);
  // angle_err2 = r2(4);
  if ((real32_T)fabs((real_T)(mtrc - 3.0F)) <= 1.0E-6F) {
    f0 = 0.0F;
  } else if ((real32_T)fabs((real_T)(mtrc + 1.0F)) <= 1.0E-6F) {
    f0 = 3.14159274F;
  } else {
    f0 = (real32_T)acos((real_T)((mtrc - 1.0F) / 2.0F));
  }

  if ((real32_T)fabs((real_T)(b_mtrc - 3.0F)) <= 1.0E-6F) {
    f1 = 0.0F;
  } else if ((real32_T)fabs((real_T)(b_mtrc + 1.0F)) <= 1.0E-6F) {
    f1 = 3.14159274F;
  } else {
    f1 = (real32_T)acos((real_T)((b_mtrc - 1.0F) / 2.0F));
  }

  if (f0 <= f1) {
    for (i = 0; i < 9; i++) {
      R_des2[i] = R_des1[i];
    }
  }

  for (i = 0; i < 3; i++) {
    for (i0 = 0; i0 < 3; i0++) {
      y[i + 3 * i0] = 0.0F;
      for (i1 = 0; i1 < 3; i1++) {
        y[i + 3 * i0] += R_des2[i1 + 3 * i] * R_B[i1 + 3 * i0];
      }

      b_R_B[i + 3 * i0] = 0.0F;
      for (i1 = 0; i1 < 3; i1++) {
        b_R_B[i + 3 * i0] += R_B[i1 + 3 * i] * R_des2[i1 + 3 * i0];
      }
    }
  }

  for (i = 0; i < 3; i++) {
    for (i0 = 0; i0 < 3; i0++) {
      R_des1[i0 + 3 * i] = 0.5F * (y[i0 + 3 * i] - b_R_B[i0 + 3 * i]);
    }
  }

  b_R_des1[0] = R_des1[5];
  b_R_des1[1] = R_des1[6];
  b_R_des1[2] = R_des1[1];
  for (i = 0; i < 3; i++) {
    rates_des[i] = Kp[i] * b_R_des1[i];
  }

  *thrust = norm(F_des);

  // thrust = F_des' * (R_B(:,3));
}

//
// Arguments    : void
// Return Type  : void
//
void attitude_controller_initialize()
{
  rt_InitInfAndNaN(8U);
}

//
// Arguments    : void
// Return Type  : void
//
void attitude_controller_terminate()
{
  // (no terminate code required)
}

//
// File trailer for attitude_controller.cpp
//
// [EOF]
//
