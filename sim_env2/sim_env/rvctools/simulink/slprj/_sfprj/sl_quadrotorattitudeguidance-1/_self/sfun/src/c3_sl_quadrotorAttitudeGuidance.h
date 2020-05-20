#ifndef __c3_sl_quadrotorAttitudeGuidance_h__
#define __c3_sl_quadrotorAttitudeGuidance_h__

/* Include files */
#include "sf_runtime/sfc_sf.h"
#include "sf_runtime/sfc_mex.h"
#include "rtwtypes.h"
#include "multiword_types.h"

/* Type Definitions */
#ifndef typedef_SFc3_sl_quadrotorAttitudeGuidanceInstanceStruct
#define typedef_SFc3_sl_quadrotorAttitudeGuidanceInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c3_sfEvent;
  boolean_T c3_isStable;
  boolean_T c3_doneDoubleBufferReInit;
  uint8_T c3_is_active_c3_sl_quadrotorAttitudeGuidance;
  real_T c3_firstentry;
  boolean_T c3_firstentry_not_empty;
  real_T (*c3_routeIn)[16];
  real_T *c3_route_costIn;
  real_T (*c3_Xwp_routeIn)[15];
  real_T (*c3_Ywp_routeIn)[15];
  real_T (*c3_Zwp_routeIn)[15];
  real_T (*c3_PSIwp_routeIn)[15];
  real_T (*c3_route)[16];
  real_T *c3_route_cost;
  real_T (*c3_Xwp)[15];
  real_T (*c3_Xwp_route)[15];
  real_T (*c3_Ywp)[15];
  real_T (*c3_Zwp)[15];
  real_T (*c3_PSIwp)[15];
  real_T (*c3_Ywp_route)[15];
  real_T (*c3_Zwp_route)[15];
  real_T (*c3_PSIwp_route)[15];
} SFc3_sl_quadrotorAttitudeGuidanceInstanceStruct;

#endif                                 /*typedef_SFc3_sl_quadrotorAttitudeGuidanceInstanceStruct*/

/* Named Constants */

/* Variable Declarations */
extern struct SfDebugInstanceStruct *sfGlobalDebugInstanceStruct;

/* Variable Definitions */

/* Function Declarations */
extern const mxArray
  *sf_c3_sl_quadrotorAttitudeGuidance_get_eml_resolved_functions_info(void);

/* Function Definitions */
extern void sf_c3_sl_quadrotorAttitudeGuidance_get_check_sum(mxArray *plhs[]);
extern void c3_sl_quadrotorAttitudeGuidance_method_dispatcher(SimStruct *S,
  int_T method, void *data);

#endif
