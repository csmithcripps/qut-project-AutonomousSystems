#ifndef __c2_sl_quadrotorAttitudeGuidance_h__
#define __c2_sl_quadrotorAttitudeGuidance_h__

/* Include files */
#include "sf_runtime/sfc_sf.h"
#include "sf_runtime/sfc_mex.h"
#include "rtwtypes.h"
#include "multiword_types.h"

/* Type Definitions */
#ifndef typedef_SFc2_sl_quadrotorAttitudeGuidanceInstanceStruct
#define typedef_SFc2_sl_quadrotorAttitudeGuidanceInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c2_sfEvent;
  boolean_T c2_isStable;
  boolean_T c2_doneDoubleBufferReInit;
  uint8_T c2_is_active_c2_sl_quadrotorAttitudeGuidance;
  real_T *c2_x_star;
  real_T *c2_y_star;
  real_T *c2_z_star;
  real_T *c2_psi_star;
  real_T (*c2_X)[12];
  real_T *c2_loitercmd;
  real_T *c2_VerrorInt;
  real_T *c2_pitch_star;
  real_T *c2_roll_star;
  real_T *c2_yaw_star;
  real_T *c2_hgt_star;
  real_T *c2_lambda;
  real_T *c2_lambdadot;
  real_T *c2_rtilde;
  real_T *c2_Verror;
  real_T *c2_heading_nav;
  real_T *c2_track_angle;
  real_T *c2_roll;
  real_T *c2_pitch;
  real_T *c2_yaw;
  real_T *c2_x;
  real_T *c2_y;
  real_T *c2_z;
  real_T *c2_Vg_n;
  real_T *c2_heading_body;
  real_T *c2_ay_n;
  real_T *c2_ay_b;
  real_T *c2_lambdaprev;
  real_T *c2_Vg_star;
  real_T *c2_tgo;
  real_T *c2_heading_body_prev;
  real_T *c2_heading_nav_prev;
} SFc2_sl_quadrotorAttitudeGuidanceInstanceStruct;

#endif                                 /*typedef_SFc2_sl_quadrotorAttitudeGuidanceInstanceStruct*/

/* Named Constants */

/* Variable Declarations */
extern struct SfDebugInstanceStruct *sfGlobalDebugInstanceStruct;

/* Variable Definitions */

/* Function Declarations */
extern const mxArray
  *sf_c2_sl_quadrotorAttitudeGuidance_get_eml_resolved_functions_info(void);

/* Function Definitions */
extern void sf_c2_sl_quadrotorAttitudeGuidance_get_check_sum(mxArray *plhs[]);
extern void c2_sl_quadrotorAttitudeGuidance_method_dispatcher(SimStruct *S,
  int_T method, void *data);

#endif
