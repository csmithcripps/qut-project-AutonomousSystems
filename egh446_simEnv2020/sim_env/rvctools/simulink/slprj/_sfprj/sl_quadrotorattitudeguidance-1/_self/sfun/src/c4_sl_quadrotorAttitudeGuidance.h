#ifndef __c4_sl_quadrotorAttitudeGuidance_h__
#define __c4_sl_quadrotorAttitudeGuidance_h__

/* Include files */
#include "sf_runtime/sfc_sf.h"
#include "sf_runtime/sfc_mex.h"
#include "rtwtypes.h"
#include "multiword_types.h"

/* Type Definitions */
#ifndef typedef_SFc4_sl_quadrotorAttitudeGuidanceInstanceStruct
#define typedef_SFc4_sl_quadrotorAttitudeGuidanceInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c4_sfEvent;
  boolean_T c4_isStable;
  boolean_T c4_doneDoubleBufferReInit;
  uint8_T c4_is_active_c4_sl_quadrotorAttitudeGuidance;
  real_T c4_firstentry;
  boolean_T c4_firstentry_not_empty;
  real_T (*c4_X)[12];
  real_T *c4_WP_indexOut;
  real_T *c4_WP_index;
  real_T (*c4_route)[16];
  real_T (*c4_Xwp_route)[15];
  real_T (*c4_Ywp_route)[15];
  real_T (*c4_Zwp_route)[15];
  real_T (*c4_PSIwp_route)[15];
  real_T *c4_x_star;
  real_T *c4_y_star;
  real_T *c4_z_star;
  real_T *c4_psi_star;
  real_T *c4_stop_simulation;
} SFc4_sl_quadrotorAttitudeGuidanceInstanceStruct;

#endif                                 /*typedef_SFc4_sl_quadrotorAttitudeGuidanceInstanceStruct*/

/* Named Constants */

/* Variable Declarations */
extern struct SfDebugInstanceStruct *sfGlobalDebugInstanceStruct;

/* Variable Definitions */

/* Function Declarations */
extern const mxArray
  *sf_c4_sl_quadrotorAttitudeGuidance_get_eml_resolved_functions_info(void);

/* Function Definitions */
extern void sf_c4_sl_quadrotorAttitudeGuidance_get_check_sum(mxArray *plhs[]);
extern void c4_sl_quadrotorAttitudeGuidance_method_dispatcher(SimStruct *S,
  int_T method, void *data);

#endif
