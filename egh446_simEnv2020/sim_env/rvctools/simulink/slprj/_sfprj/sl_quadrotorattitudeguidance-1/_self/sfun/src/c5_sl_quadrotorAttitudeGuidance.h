#ifndef __c5_sl_quadrotorAttitudeGuidance_h__
#define __c5_sl_quadrotorAttitudeGuidance_h__

/* Include files */
#include "sf_runtime/sfc_sf.h"
#include "sf_runtime/sfc_mex.h"
#include "rtwtypes.h"
#include "multiword_types.h"

/* Type Definitions */
#ifndef typedef_SFc5_sl_quadrotorAttitudeGuidanceInstanceStruct
#define typedef_SFc5_sl_quadrotorAttitudeGuidanceInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c5_sfEvent;
  boolean_T c5_isStable;
  boolean_T c5_doneDoubleBufferReInit;
  uint8_T c5_is_active_c5_sl_quadrotorAttitudeGuidance;
  real_T *c5_x_wp;
  real_T *c5_y_wp;
  real_T *c5_z_wp;
  real_T (*c5_X)[12];
} SFc5_sl_quadrotorAttitudeGuidanceInstanceStruct;

#endif                                 /*typedef_SFc5_sl_quadrotorAttitudeGuidanceInstanceStruct*/

/* Named Constants */

/* Variable Declarations */
extern struct SfDebugInstanceStruct *sfGlobalDebugInstanceStruct;

/* Variable Definitions */

/* Function Declarations */
extern const mxArray
  *sf_c5_sl_quadrotorAttitudeGuidance_get_eml_resolved_functions_info(void);

/* Function Definitions */
extern void sf_c5_sl_quadrotorAttitudeGuidance_get_check_sum(mxArray *plhs[]);
extern void c5_sl_quadrotorAttitudeGuidance_method_dispatcher(SimStruct *S,
  int_T method, void *data);

#endif
