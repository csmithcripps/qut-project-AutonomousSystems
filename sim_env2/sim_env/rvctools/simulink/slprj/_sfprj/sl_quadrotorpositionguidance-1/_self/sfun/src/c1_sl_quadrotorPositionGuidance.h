#ifndef __c1_sl_quadrotorPositionGuidance_h__
#define __c1_sl_quadrotorPositionGuidance_h__

/* Include files */
#include "sf_runtime/sfc_sf.h"
#include "sf_runtime/sfc_mex.h"
#include "rtwtypes.h"
#include "multiword_types.h"

/* Type Definitions */
#ifndef typedef_SFc1_sl_quadrotorPositionGuidanceInstanceStruct
#define typedef_SFc1_sl_quadrotorPositionGuidanceInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c1_sfEvent;
  boolean_T c1_isStable;
  boolean_T c1_doneDoubleBufferReInit;
  uint8_T c1_is_active_c1_sl_quadrotorPositionGuidance;
  real_T c1_S[60];
  boolean_T c1_S_not_empty;
  real_T c1_firstentry;
  boolean_T c1_firstentry_not_empty;
  real_T (*c1_Xwp)[15];
  real_T (*c1_Ywp)[15];
  real_T (*c1_Zwp)[15];
  real_T (*c1_PSIwp)[15];
  real_T (*c1_XwpIn)[15];
  real_T (*c1_YwpIn)[15];
  real_T (*c1_ZwpIn)[15];
  real_T (*c1_PSIwpIn)[15];
} SFc1_sl_quadrotorPositionGuidanceInstanceStruct;

#endif                                 /*typedef_SFc1_sl_quadrotorPositionGuidanceInstanceStruct*/

/* Named Constants */

/* Variable Declarations */
extern struct SfDebugInstanceStruct *sfGlobalDebugInstanceStruct;

/* Variable Definitions */

/* Function Declarations */
extern const mxArray
  *sf_c1_sl_quadrotorPositionGuidance_get_eml_resolved_functions_info(void);

/* Function Definitions */
extern void sf_c1_sl_quadrotorPositionGuidance_get_check_sum(mxArray *plhs[]);
extern void c1_sl_quadrotorPositionGuidance_method_dispatcher(SimStruct *S,
  int_T method, void *data);

#endif
