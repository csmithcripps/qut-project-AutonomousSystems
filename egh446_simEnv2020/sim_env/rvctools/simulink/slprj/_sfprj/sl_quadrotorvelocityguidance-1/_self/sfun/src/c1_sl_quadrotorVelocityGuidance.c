/* Include files */

#include <stddef.h>
#include "blas.h"
#include "sl_quadrotorVelocityGuidance_sfun.h"
#include "c1_sl_quadrotorVelocityGuidance.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "sl_quadrotorVelocityGuidance_sfun_debug_macros.h"
#define _SF_MEX_LISTEN_FOR_CTRL_C(S)   sf_mex_listen_for_ctrl_c(sfGlobalDebugInstanceStruct,S);

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static real_T _sfTime_;
static const char * c1_debug_family_names[16] = { "Nwaypoints", "fid",
  "formatSpec", "sizeA", "nargin", "nargout", "XwpIn", "YwpIn", "ZwpIn",
  "PSIwpIn", "Xwp", "Ywp", "Zwp", "PSIwp", "S", "firstentry" };

/* Function Declarations */
static void initialize_c1_sl_quadrotorVelocityGuidance
  (SFc1_sl_quadrotorVelocityGuidanceInstanceStruct *chartInstance);
static void initialize_params_c1_sl_quadrotorVelocityGuidance
  (SFc1_sl_quadrotorVelocityGuidanceInstanceStruct *chartInstance);
static void enable_c1_sl_quadrotorVelocityGuidance
  (SFc1_sl_quadrotorVelocityGuidanceInstanceStruct *chartInstance);
static void disable_c1_sl_quadrotorVelocityGuidance
  (SFc1_sl_quadrotorVelocityGuidanceInstanceStruct *chartInstance);
static void c1_update_debugger_state_c1_sl_quadrotorVelocityGuidance
  (SFc1_sl_quadrotorVelocityGuidanceInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c1_sl_quadrotorVelocityGuidance
  (SFc1_sl_quadrotorVelocityGuidanceInstanceStruct *chartInstance);
static void set_sim_state_c1_sl_quadrotorVelocityGuidance
  (SFc1_sl_quadrotorVelocityGuidanceInstanceStruct *chartInstance, const mxArray
   *c1_st);
static void finalize_c1_sl_quadrotorVelocityGuidance
  (SFc1_sl_quadrotorVelocityGuidanceInstanceStruct *chartInstance);
static void sf_gateway_c1_sl_quadrotorVelocityGuidance
  (SFc1_sl_quadrotorVelocityGuidanceInstanceStruct *chartInstance);
static void mdl_start_c1_sl_quadrotorVelocityGuidance
  (SFc1_sl_quadrotorVelocityGuidanceInstanceStruct *chartInstance);
static void initSimStructsc1_sl_quadrotorVelocityGuidance
  (SFc1_sl_quadrotorVelocityGuidanceInstanceStruct *chartInstance);
static void init_script_number_translation(uint32_T c1_machineNumber, uint32_T
  c1_chartNumber, uint32_T c1_instanceNumber);
static const mxArray *c1_sf_marshallOut(void *chartInstanceVoid, void *c1_inData);
static real_T c1_emlrt_marshallIn
  (SFc1_sl_quadrotorVelocityGuidanceInstanceStruct *chartInstance, const mxArray
   *c1_b_firstentry, const char_T *c1_identifier);
static real_T c1_b_emlrt_marshallIn
  (SFc1_sl_quadrotorVelocityGuidanceInstanceStruct *chartInstance, const mxArray
   *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void c1_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static const mxArray *c1_b_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static void c1_c_emlrt_marshallIn
  (SFc1_sl_quadrotorVelocityGuidanceInstanceStruct *chartInstance, const mxArray
   *c1_b_S, const char_T *c1_identifier, real_T c1_y[60]);
static void c1_d_emlrt_marshallIn
  (SFc1_sl_quadrotorVelocityGuidanceInstanceStruct *chartInstance, const mxArray
   *c1_u, const emlrtMsgIdentifier *c1_parentId, real_T c1_y[60]);
static void c1_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static const mxArray *c1_c_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static void c1_e_emlrt_marshallIn
  (SFc1_sl_quadrotorVelocityGuidanceInstanceStruct *chartInstance, const mxArray
   *c1_b_PSIwp, const char_T *c1_identifier, real_T c1_y[15]);
static void c1_f_emlrt_marshallIn
  (SFc1_sl_quadrotorVelocityGuidanceInstanceStruct *chartInstance, const mxArray
   *c1_u, const emlrtMsgIdentifier *c1_parentId, real_T c1_y[15]);
static void c1_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static const mxArray *c1_d_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static real_T c1_g_emlrt_marshallIn
  (SFc1_sl_quadrotorVelocityGuidanceInstanceStruct *chartInstance, const mxArray
   *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void c1_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static const mxArray *c1_e_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static void c1_h_emlrt_marshallIn
  (SFc1_sl_quadrotorVelocityGuidanceInstanceStruct *chartInstance, const mxArray
   *c1_u, const emlrtMsgIdentifier *c1_parentId, real_T c1_y[2]);
static void c1_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static const mxArray *c1_f_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static void c1_i_emlrt_marshallIn
  (SFc1_sl_quadrotorVelocityGuidanceInstanceStruct *chartInstance, const mxArray
   *c1_u, const emlrtMsgIdentifier *c1_parentId, char_T c1_y[11]);
static void c1_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static const mxArray *c1_g_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static void c1_info_helper(const mxArray **c1_info);
static const mxArray *c1_emlrt_marshallOut(const char * c1_u);
static const mxArray *c1_b_emlrt_marshallOut(const uint32_T c1_u);
static void c1_j_emlrt_marshallIn
  (SFc1_sl_quadrotorVelocityGuidanceInstanceStruct *chartInstance, const mxArray
   *c1_fscanf, const char_T *c1_identifier, real_T c1_y[60]);
static void c1_k_emlrt_marshallIn
  (SFc1_sl_quadrotorVelocityGuidanceInstanceStruct *chartInstance, const mxArray
   *c1_u, const emlrtMsgIdentifier *c1_parentId, real_T c1_y[60]);
static const mxArray *c1_h_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static int32_T c1_l_emlrt_marshallIn
  (SFc1_sl_quadrotorVelocityGuidanceInstanceStruct *chartInstance, const mxArray
   *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void c1_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static uint8_T c1_m_emlrt_marshallIn
  (SFc1_sl_quadrotorVelocityGuidanceInstanceStruct *chartInstance, const mxArray
   *c1_b_is_active_c1_sl_quadrotorVelocityGuidance, const char_T *c1_identifier);
static uint8_T c1_n_emlrt_marshallIn
  (SFc1_sl_quadrotorVelocityGuidanceInstanceStruct *chartInstance, const mxArray
   *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void init_dsm_address_info
  (SFc1_sl_quadrotorVelocityGuidanceInstanceStruct *chartInstance);
static void init_simulink_io_address
  (SFc1_sl_quadrotorVelocityGuidanceInstanceStruct *chartInstance);

/* Function Definitions */
static void initialize_c1_sl_quadrotorVelocityGuidance
  (SFc1_sl_quadrotorVelocityGuidanceInstanceStruct *chartInstance)
{
  chartInstance->c1_sfEvent = CALL_EVENT;
  _sfTime_ = sf_get_time(chartInstance->S);
  chartInstance->c1_S_not_empty = false;
  chartInstance->c1_firstentry_not_empty = false;
  chartInstance->c1_is_active_c1_sl_quadrotorVelocityGuidance = 0U;
}

static void initialize_params_c1_sl_quadrotorVelocityGuidance
  (SFc1_sl_quadrotorVelocityGuidanceInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void enable_c1_sl_quadrotorVelocityGuidance
  (SFc1_sl_quadrotorVelocityGuidanceInstanceStruct *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void disable_c1_sl_quadrotorVelocityGuidance
  (SFc1_sl_quadrotorVelocityGuidanceInstanceStruct *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void c1_update_debugger_state_c1_sl_quadrotorVelocityGuidance
  (SFc1_sl_quadrotorVelocityGuidanceInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static const mxArray *get_sim_state_c1_sl_quadrotorVelocityGuidance
  (SFc1_sl_quadrotorVelocityGuidanceInstanceStruct *chartInstance)
{
  const mxArray *c1_st;
  const mxArray *c1_y = NULL;
  int32_T c1_i0;
  real_T c1_u[15];
  const mxArray *c1_b_y = NULL;
  int32_T c1_i1;
  real_T c1_b_u[15];
  const mxArray *c1_c_y = NULL;
  int32_T c1_i2;
  real_T c1_c_u[15];
  const mxArray *c1_d_y = NULL;
  int32_T c1_i3;
  real_T c1_d_u[15];
  const mxArray *c1_e_y = NULL;
  int32_T c1_i4;
  real_T c1_e_u[60];
  const mxArray *c1_f_y = NULL;
  real_T c1_hoistedGlobal;
  real_T c1_f_u;
  const mxArray *c1_g_y = NULL;
  uint8_T c1_b_hoistedGlobal;
  uint8_T c1_g_u;
  const mxArray *c1_h_y = NULL;
  c1_st = NULL;
  c1_st = NULL;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_createcellmatrix(7, 1), false);
  for (c1_i0 = 0; c1_i0 < 15; c1_i0++) {
    c1_u[c1_i0] = (*chartInstance->c1_PSIwp)[c1_i0];
  }

  c1_b_y = NULL;
  sf_mex_assign(&c1_b_y, sf_mex_create("y", c1_u, 0, 0U, 1U, 0U, 2, 1, 15),
                false);
  sf_mex_setcell(c1_y, 0, c1_b_y);
  for (c1_i1 = 0; c1_i1 < 15; c1_i1++) {
    c1_b_u[c1_i1] = (*chartInstance->c1_Xwp)[c1_i1];
  }

  c1_c_y = NULL;
  sf_mex_assign(&c1_c_y, sf_mex_create("y", c1_b_u, 0, 0U, 1U, 0U, 2, 1, 15),
                false);
  sf_mex_setcell(c1_y, 1, c1_c_y);
  for (c1_i2 = 0; c1_i2 < 15; c1_i2++) {
    c1_c_u[c1_i2] = (*chartInstance->c1_Ywp)[c1_i2];
  }

  c1_d_y = NULL;
  sf_mex_assign(&c1_d_y, sf_mex_create("y", c1_c_u, 0, 0U, 1U, 0U, 2, 1, 15),
                false);
  sf_mex_setcell(c1_y, 2, c1_d_y);
  for (c1_i3 = 0; c1_i3 < 15; c1_i3++) {
    c1_d_u[c1_i3] = (*chartInstance->c1_Zwp)[c1_i3];
  }

  c1_e_y = NULL;
  sf_mex_assign(&c1_e_y, sf_mex_create("y", c1_d_u, 0, 0U, 1U, 0U, 2, 1, 15),
                false);
  sf_mex_setcell(c1_y, 3, c1_e_y);
  for (c1_i4 = 0; c1_i4 < 60; c1_i4++) {
    c1_e_u[c1_i4] = chartInstance->c1_S[c1_i4];
  }

  c1_f_y = NULL;
  if (!chartInstance->c1_S_not_empty) {
    sf_mex_assign(&c1_f_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c1_f_y, sf_mex_create("y", c1_e_u, 0, 0U, 1U, 0U, 2, 4, 15),
                  false);
  }

  sf_mex_setcell(c1_y, 4, c1_f_y);
  c1_hoistedGlobal = chartInstance->c1_firstentry;
  c1_f_u = c1_hoistedGlobal;
  c1_g_y = NULL;
  if (!chartInstance->c1_firstentry_not_empty) {
    sf_mex_assign(&c1_g_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c1_g_y, sf_mex_create("y", &c1_f_u, 0, 0U, 0U, 0U, 0), false);
  }

  sf_mex_setcell(c1_y, 5, c1_g_y);
  c1_b_hoistedGlobal =
    chartInstance->c1_is_active_c1_sl_quadrotorVelocityGuidance;
  c1_g_u = c1_b_hoistedGlobal;
  c1_h_y = NULL;
  sf_mex_assign(&c1_h_y, sf_mex_create("y", &c1_g_u, 3, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c1_y, 6, c1_h_y);
  sf_mex_assign(&c1_st, c1_y, false);
  return c1_st;
}

static void set_sim_state_c1_sl_quadrotorVelocityGuidance
  (SFc1_sl_quadrotorVelocityGuidanceInstanceStruct *chartInstance, const mxArray
   *c1_st)
{
  const mxArray *c1_u;
  real_T c1_dv0[15];
  int32_T c1_i5;
  real_T c1_dv1[15];
  int32_T c1_i6;
  real_T c1_dv2[15];
  int32_T c1_i7;
  real_T c1_dv3[15];
  int32_T c1_i8;
  real_T c1_dv4[60];
  int32_T c1_i9;
  chartInstance->c1_doneDoubleBufferReInit = true;
  c1_u = sf_mex_dup(c1_st);
  c1_e_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c1_u, 0)),
                        "PSIwp", c1_dv0);
  for (c1_i5 = 0; c1_i5 < 15; c1_i5++) {
    (*chartInstance->c1_PSIwp)[c1_i5] = c1_dv0[c1_i5];
  }

  c1_e_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c1_u, 1)),
                        "Xwp", c1_dv1);
  for (c1_i6 = 0; c1_i6 < 15; c1_i6++) {
    (*chartInstance->c1_Xwp)[c1_i6] = c1_dv1[c1_i6];
  }

  c1_e_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c1_u, 2)),
                        "Ywp", c1_dv2);
  for (c1_i7 = 0; c1_i7 < 15; c1_i7++) {
    (*chartInstance->c1_Ywp)[c1_i7] = c1_dv2[c1_i7];
  }

  c1_e_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c1_u, 3)),
                        "Zwp", c1_dv3);
  for (c1_i8 = 0; c1_i8 < 15; c1_i8++) {
    (*chartInstance->c1_Zwp)[c1_i8] = c1_dv3[c1_i8];
  }

  c1_c_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c1_u, 4)), "S",
                        c1_dv4);
  for (c1_i9 = 0; c1_i9 < 60; c1_i9++) {
    chartInstance->c1_S[c1_i9] = c1_dv4[c1_i9];
  }

  chartInstance->c1_firstentry = c1_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c1_u, 5)), "firstentry");
  chartInstance->c1_is_active_c1_sl_quadrotorVelocityGuidance =
    c1_m_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c1_u, 6)),
    "is_active_c1_sl_quadrotorVelocityGuidance");
  sf_mex_destroy(&c1_u);
  c1_update_debugger_state_c1_sl_quadrotorVelocityGuidance(chartInstance);
  sf_mex_destroy(&c1_st);
}

static void finalize_c1_sl_quadrotorVelocityGuidance
  (SFc1_sl_quadrotorVelocityGuidanceInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void sf_gateway_c1_sl_quadrotorVelocityGuidance
  (SFc1_sl_quadrotorVelocityGuidanceInstanceStruct *chartInstance)
{
  static real_T c1_dv5[2] = { 4.0, 0.0 };

  int32_T c1_i10;
  real_T c1_b_XwpIn[15];
  int32_T c1_i11;
  real_T c1_b_YwpIn[15];
  int32_T c1_i12;
  real_T c1_b_ZwpIn[15];
  int32_T c1_i13;
  real_T c1_b_PSIwpIn[15];
  uint32_T c1_debug_family_var_map[16];
  real_T c1_Nwaypoints;
  const mxArray *c1_fid = NULL;
  char_T c1_formatSpec[11];
  real_T c1_sizeA[2];
  real_T c1_nargin = 4.0;
  real_T c1_nargout = 4.0;
  real_T c1_b_Xwp[15];
  real_T c1_b_Ywp[15];
  real_T c1_b_Zwp[15];
  real_T c1_b_PSIwp[15];
  int32_T c1_i14;
  int32_T c1_i15;
  static char_T c1_cv0[14] = { '\\', 'W', 'a', 'y', 'p', 'o', 'i', 'n', 't', 's',
    '.', 't', 'x', 't' };

  char_T c1_u[14];
  const mxArray *c1_y = NULL;
  char_T c1_b_u;
  const mxArray *c1_b_y = NULL;
  int32_T c1_i16;
  static char_T c1_cv1[11] = { '%', 'd', ' ', '%', 'd', ' ', '%', 'd', ' ', '%',
    'd' };

  int32_T c1_i17;
  int32_T c1_i18;
  char_T c1_c_u[11];
  const mxArray *c1_c_y = NULL;
  int32_T c1_i19;
  real_T c1_d_u[2];
  const mxArray *c1_d_y = NULL;
  real_T c1_dv6[60];
  int32_T c1_i20;
  int32_T c1_i21;
  int32_T c1_i22;
  int32_T c1_i23;
  int32_T c1_i24;
  int32_T c1_i25;
  int32_T c1_i26;
  int32_T c1_i27;
  int32_T c1_i28;
  int32_T c1_i29;
  int32_T c1_i30;
  int32_T c1_i31;
  int32_T c1_i32;
  int32_T c1_i33;
  int32_T c1_i34;
  int32_T c1_i35;
  int32_T c1_i36;
  int32_T c1_i37;
  int32_T c1_i38;
  int32_T c1_i39;
  int32_T c1_i40;
  int32_T c1_i41;
  int32_T c1_i42;
  int32_T c1_i43;
  int32_T c1_i44;
  c1_dv5[1U] = rtInf;
  _SFD_SYMBOL_SCOPE_PUSH(0U, 0U);
  _sfTime_ = sf_get_time(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 0U, chartInstance->c1_sfEvent);
  chartInstance->c1_sfEvent = CALL_EVENT;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 0U, chartInstance->c1_sfEvent);
  for (c1_i10 = 0; c1_i10 < 15; c1_i10++) {
    c1_b_XwpIn[c1_i10] = (*chartInstance->c1_XwpIn)[c1_i10];
  }

  for (c1_i11 = 0; c1_i11 < 15; c1_i11++) {
    c1_b_YwpIn[c1_i11] = (*chartInstance->c1_YwpIn)[c1_i11];
  }

  for (c1_i12 = 0; c1_i12 < 15; c1_i12++) {
    c1_b_ZwpIn[c1_i12] = (*chartInstance->c1_ZwpIn)[c1_i12];
  }

  for (c1_i13 = 0; c1_i13 < 15; c1_i13++) {
    c1_b_PSIwpIn[c1_i13] = (*chartInstance->c1_PSIwpIn)[c1_i13];
  }

  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 16U, 16U, c1_debug_family_names,
    c1_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_Nwaypoints, 0U, c1_d_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_fid, 1U, c1_g_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c1_formatSpec, 2U, c1_f_sf_marshallOut,
    c1_f_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c1_sizeA, 3U, c1_e_sf_marshallOut,
    c1_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_nargin, 4U, c1_d_sf_marshallOut,
    c1_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_nargout, 5U, c1_d_sf_marshallOut,
    c1_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(c1_b_XwpIn, 6U, c1_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c1_b_YwpIn, 7U, c1_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c1_b_ZwpIn, 8U, c1_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c1_b_PSIwpIn, 9U, c1_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c1_b_Xwp, 10U, c1_c_sf_marshallOut,
    c1_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c1_b_Ywp, 11U, c1_c_sf_marshallOut,
    c1_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c1_b_Zwp, 12U, c1_c_sf_marshallOut,
    c1_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c1_b_PSIwp, 13U, c1_c_sf_marshallOut,
    c1_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(chartInstance->c1_S, 14U,
    c1_b_sf_marshallOut, c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&chartInstance->c1_firstentry, 15U,
    c1_sf_marshallOut, c1_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 4);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 5);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 6);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 9);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 10);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 17);
  if (CV_EML_IF(0, 1, 0, !chartInstance->c1_firstentry_not_empty)) {
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 18);
    chartInstance->c1_firstentry = 0.0;
    chartInstance->c1_firstentry_not_empty = true;
  }

  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 22);
  c1_Nwaypoints = 15.0;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 23);
  for (c1_i14 = 0; c1_i14 < 60; c1_i14++) {
    chartInstance->c1_S[c1_i14] = 0.0;
  }

  chartInstance->c1_S_not_empty = true;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 27);
  if (CV_EML_IF(0, 1, 1, CV_RELATIONAL_EVAL(4U, 0U, 0,
        chartInstance->c1_firstentry, 0.0, -1, 0U, chartInstance->c1_firstentry ==
        0.0))) {
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 28);
    for (c1_i15 = 0; c1_i15 < 14; c1_i15++) {
      c1_u[c1_i15] = c1_cv0[c1_i15];
    }

    c1_y = NULL;
    sf_mex_assign(&c1_y, sf_mex_create("y", c1_u, 10, 0U, 1U, 0U, 2, 1, 14),
                  false);
    c1_b_u = 'r';
    c1_b_y = NULL;
    sf_mex_assign(&c1_b_y, sf_mex_create("y", &c1_b_u, 10, 0U, 0U, 0U, 0), false);
    sf_mex_assign(&c1_fid, sf_mex_call_debug(sfGlobalDebugInstanceStruct,
      "fopen", 1U, 2U, 14, sf_mex_call_debug(sfGlobalDebugInstanceStruct,
      "strcat", 1U, 2U, 14, sf_mex_call_debug(sfGlobalDebugInstanceStruct, "pwd",
      1U, 0U), 14, c1_y), 14, c1_b_y), false);
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 29);
    for (c1_i16 = 0; c1_i16 < 11; c1_i16++) {
      c1_formatSpec[c1_i16] = c1_cv1[c1_i16];
    }

    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 30);
    for (c1_i17 = 0; c1_i17 < 2; c1_i17++) {
      c1_sizeA[c1_i17] = c1_dv5[c1_i17];
    }

    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 31);
    for (c1_i18 = 0; c1_i18 < 11; c1_i18++) {
      c1_c_u[c1_i18] = c1_formatSpec[c1_i18];
    }

    c1_c_y = NULL;
    sf_mex_assign(&c1_c_y, sf_mex_create("y", c1_c_u, 10, 0U, 1U, 0U, 2, 1, 11),
                  false);
    for (c1_i19 = 0; c1_i19 < 2; c1_i19++) {
      c1_d_u[c1_i19] = c1_sizeA[c1_i19];
    }

    c1_d_y = NULL;
    sf_mex_assign(&c1_d_y, sf_mex_create("y", c1_d_u, 0, 0U, 1U, 0U, 2, 1, 2),
                  false);
    c1_j_emlrt_marshallIn(chartInstance, sf_mex_call_debug
                          (sfGlobalDebugInstanceStruct, "fscanf", 1U, 3U, 14,
      sf_mex_dup(c1_fid), 14, c1_c_y, 14, c1_d_y), "fscanf", c1_dv6);
    for (c1_i20 = 0; c1_i20 < 60; c1_i20++) {
      chartInstance->c1_S[c1_i20] = c1_dv6[c1_i20];
    }

    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 32);
    c1_i21 = 0;
    for (c1_i22 = 0; c1_i22 < 15; c1_i22++) {
      c1_b_Xwp[c1_i22] = chartInstance->c1_S[c1_i21];
      c1_i21 += 4;
    }

    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 33);
    c1_i23 = 0;
    for (c1_i24 = 0; c1_i24 < 15; c1_i24++) {
      c1_b_Ywp[c1_i24] = chartInstance->c1_S[c1_i23 + 1];
      c1_i23 += 4;
    }

    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 34);
    c1_i25 = 0;
    for (c1_i26 = 0; c1_i26 < 15; c1_i26++) {
      c1_b_Zwp[c1_i26] = chartInstance->c1_S[c1_i25 + 2];
      c1_i25 += 4;
    }

    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 35);
    c1_i27 = 0;
    for (c1_i28 = 0; c1_i28 < 15; c1_i28++) {
      c1_b_PSIwp[c1_i28] = chartInstance->c1_S[c1_i27 + 3];
      c1_i27 += 4;
    }

    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 36);
    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "fclose", 0U, 1U, 14,
                      sf_mex_dup(c1_fid));
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 37);
    chartInstance->c1_firstentry = 1.0;
  } else {
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 53);
    for (c1_i29 = 0; c1_i29 < 15; c1_i29++) {
      c1_b_Xwp[c1_i29] = c1_b_XwpIn[c1_i29];
    }

    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 54);
    for (c1_i30 = 0; c1_i30 < 15; c1_i30++) {
      c1_b_Ywp[c1_i30] = c1_b_YwpIn[c1_i30];
    }

    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 55);
    for (c1_i31 = 0; c1_i31 < 15; c1_i31++) {
      c1_b_Zwp[c1_i31] = c1_b_ZwpIn[c1_i31];
    }

    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 56);
    for (c1_i32 = 0; c1_i32 < 15; c1_i32++) {
      c1_b_PSIwp[c1_i32] = c1_b_PSIwpIn[c1_i32];
    }
  }

  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, -56);
  _SFD_SYMBOL_SCOPE_POP();
  sf_mex_destroy(&c1_fid);
  for (c1_i33 = 0; c1_i33 < 15; c1_i33++) {
    (*chartInstance->c1_Xwp)[c1_i33] = c1_b_Xwp[c1_i33];
  }

  for (c1_i34 = 0; c1_i34 < 15; c1_i34++) {
    (*chartInstance->c1_Ywp)[c1_i34] = c1_b_Ywp[c1_i34];
  }

  for (c1_i35 = 0; c1_i35 < 15; c1_i35++) {
    (*chartInstance->c1_Zwp)[c1_i35] = c1_b_Zwp[c1_i35];
  }

  for (c1_i36 = 0; c1_i36 < 15; c1_i36++) {
    (*chartInstance->c1_PSIwp)[c1_i36] = c1_b_PSIwp[c1_i36];
  }

  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 0U, chartInstance->c1_sfEvent);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_sl_quadrotorVelocityGuidanceMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
  for (c1_i37 = 0; c1_i37 < 15; c1_i37++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c1_Xwp)[c1_i37], 0U);
  }

  for (c1_i38 = 0; c1_i38 < 15; c1_i38++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c1_Ywp)[c1_i38], 1U);
  }

  for (c1_i39 = 0; c1_i39 < 15; c1_i39++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c1_Zwp)[c1_i39], 2U);
  }

  for (c1_i40 = 0; c1_i40 < 15; c1_i40++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c1_PSIwp)[c1_i40], 3U);
  }

  for (c1_i41 = 0; c1_i41 < 15; c1_i41++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c1_XwpIn)[c1_i41], 4U);
  }

  for (c1_i42 = 0; c1_i42 < 15; c1_i42++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c1_YwpIn)[c1_i42], 5U);
  }

  for (c1_i43 = 0; c1_i43 < 15; c1_i43++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c1_ZwpIn)[c1_i43], 6U);
  }

  for (c1_i44 = 0; c1_i44 < 15; c1_i44++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c1_PSIwpIn)[c1_i44], 7U);
  }
}

static void mdl_start_c1_sl_quadrotorVelocityGuidance
  (SFc1_sl_quadrotorVelocityGuidanceInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void initSimStructsc1_sl_quadrotorVelocityGuidance
  (SFc1_sl_quadrotorVelocityGuidanceInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void init_script_number_translation(uint32_T c1_machineNumber, uint32_T
  c1_chartNumber, uint32_T c1_instanceNumber)
{
  (void)c1_machineNumber;
  (void)c1_chartNumber;
  (void)c1_instanceNumber;
}

static const mxArray *c1_sf_marshallOut(void *chartInstanceVoid, void *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  real_T c1_u;
  const mxArray *c1_y = NULL;
  SFc1_sl_quadrotorVelocityGuidanceInstanceStruct *chartInstance;
  chartInstance = (SFc1_sl_quadrotorVelocityGuidanceInstanceStruct *)
    chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_u = *(real_T *)c1_inData;
  c1_y = NULL;
  if (!chartInstance->c1_firstentry_not_empty) {
    sf_mex_assign(&c1_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), false);
  } else {
    sf_mex_assign(&c1_y, sf_mex_create("y", &c1_u, 0, 0U, 0U, 0U, 0), false);
  }

  sf_mex_assign(&c1_mxArrayOutData, c1_y, false);
  return c1_mxArrayOutData;
}

static real_T c1_emlrt_marshallIn
  (SFc1_sl_quadrotorVelocityGuidanceInstanceStruct *chartInstance, const mxArray
   *c1_b_firstentry, const char_T *c1_identifier)
{
  real_T c1_y;
  emlrtMsgIdentifier c1_thisId;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_firstentry),
    &c1_thisId);
  sf_mex_destroy(&c1_b_firstentry);
  return c1_y;
}

static real_T c1_b_emlrt_marshallIn
  (SFc1_sl_quadrotorVelocityGuidanceInstanceStruct *chartInstance, const mxArray
   *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  real_T c1_y;
  real_T c1_d0;
  if (mxIsEmpty(c1_u)) {
    chartInstance->c1_firstentry_not_empty = false;
  } else {
    chartInstance->c1_firstentry_not_empty = true;
    sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_d0, 1, 0, 0U, 0, 0U, 0);
    c1_y = c1_d0;
  }

  sf_mex_destroy(&c1_u);
  return c1_y;
}

static void c1_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_b_firstentry;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  real_T c1_y;
  SFc1_sl_quadrotorVelocityGuidanceInstanceStruct *chartInstance;
  chartInstance = (SFc1_sl_quadrotorVelocityGuidanceInstanceStruct *)
    chartInstanceVoid;
  c1_b_firstentry = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_firstentry),
    &c1_thisId);
  sf_mex_destroy(&c1_b_firstentry);
  *(real_T *)c1_outData = c1_y;
  sf_mex_destroy(&c1_mxArrayInData);
}

static const mxArray *c1_b_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  int32_T c1_i45;
  int32_T c1_i46;
  int32_T c1_i47;
  real_T c1_b_inData[60];
  int32_T c1_i48;
  int32_T c1_i49;
  int32_T c1_i50;
  real_T c1_u[60];
  const mxArray *c1_y = NULL;
  SFc1_sl_quadrotorVelocityGuidanceInstanceStruct *chartInstance;
  chartInstance = (SFc1_sl_quadrotorVelocityGuidanceInstanceStruct *)
    chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_i45 = 0;
  for (c1_i46 = 0; c1_i46 < 15; c1_i46++) {
    for (c1_i47 = 0; c1_i47 < 4; c1_i47++) {
      c1_b_inData[c1_i47 + c1_i45] = (*(real_T (*)[60])c1_inData)[c1_i47 +
        c1_i45];
    }

    c1_i45 += 4;
  }

  c1_i48 = 0;
  for (c1_i49 = 0; c1_i49 < 15; c1_i49++) {
    for (c1_i50 = 0; c1_i50 < 4; c1_i50++) {
      c1_u[c1_i50 + c1_i48] = c1_b_inData[c1_i50 + c1_i48];
    }

    c1_i48 += 4;
  }

  c1_y = NULL;
  if (!chartInstance->c1_S_not_empty) {
    sf_mex_assign(&c1_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), false);
  } else {
    sf_mex_assign(&c1_y, sf_mex_create("y", c1_u, 0, 0U, 1U, 0U, 2, 4, 15),
                  false);
  }

  sf_mex_assign(&c1_mxArrayOutData, c1_y, false);
  return c1_mxArrayOutData;
}

static void c1_c_emlrt_marshallIn
  (SFc1_sl_quadrotorVelocityGuidanceInstanceStruct *chartInstance, const mxArray
   *c1_b_S, const char_T *c1_identifier, real_T c1_y[60])
{
  emlrtMsgIdentifier c1_thisId;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_S), &c1_thisId, c1_y);
  sf_mex_destroy(&c1_b_S);
}

static void c1_d_emlrt_marshallIn
  (SFc1_sl_quadrotorVelocityGuidanceInstanceStruct *chartInstance, const mxArray
   *c1_u, const emlrtMsgIdentifier *c1_parentId, real_T c1_y[60])
{
  real_T c1_dv7[60];
  int32_T c1_i51;
  if (mxIsEmpty(c1_u)) {
    chartInstance->c1_S_not_empty = false;
  } else {
    chartInstance->c1_S_not_empty = true;
    sf_mex_import(c1_parentId, sf_mex_dup(c1_u), c1_dv7, 1, 0, 0U, 1, 0U, 2, 4,
                  15);
    for (c1_i51 = 0; c1_i51 < 60; c1_i51++) {
      c1_y[c1_i51] = c1_dv7[c1_i51];
    }
  }

  sf_mex_destroy(&c1_u);
}

static void c1_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_b_S;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  real_T c1_y[60];
  int32_T c1_i52;
  int32_T c1_i53;
  int32_T c1_i54;
  SFc1_sl_quadrotorVelocityGuidanceInstanceStruct *chartInstance;
  chartInstance = (SFc1_sl_quadrotorVelocityGuidanceInstanceStruct *)
    chartInstanceVoid;
  c1_b_S = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_S), &c1_thisId, c1_y);
  sf_mex_destroy(&c1_b_S);
  c1_i52 = 0;
  for (c1_i53 = 0; c1_i53 < 15; c1_i53++) {
    for (c1_i54 = 0; c1_i54 < 4; c1_i54++) {
      (*(real_T (*)[60])c1_outData)[c1_i54 + c1_i52] = c1_y[c1_i54 + c1_i52];
    }

    c1_i52 += 4;
  }

  sf_mex_destroy(&c1_mxArrayInData);
}

static const mxArray *c1_c_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  int32_T c1_i55;
  real_T c1_b_inData[15];
  int32_T c1_i56;
  real_T c1_u[15];
  const mxArray *c1_y = NULL;
  SFc1_sl_quadrotorVelocityGuidanceInstanceStruct *chartInstance;
  chartInstance = (SFc1_sl_quadrotorVelocityGuidanceInstanceStruct *)
    chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  for (c1_i55 = 0; c1_i55 < 15; c1_i55++) {
    c1_b_inData[c1_i55] = (*(real_T (*)[15])c1_inData)[c1_i55];
  }

  for (c1_i56 = 0; c1_i56 < 15; c1_i56++) {
    c1_u[c1_i56] = c1_b_inData[c1_i56];
  }

  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", c1_u, 0, 0U, 1U, 0U, 2, 1, 15), false);
  sf_mex_assign(&c1_mxArrayOutData, c1_y, false);
  return c1_mxArrayOutData;
}

static void c1_e_emlrt_marshallIn
  (SFc1_sl_quadrotorVelocityGuidanceInstanceStruct *chartInstance, const mxArray
   *c1_b_PSIwp, const char_T *c1_identifier, real_T c1_y[15])
{
  emlrtMsgIdentifier c1_thisId;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_PSIwp), &c1_thisId, c1_y);
  sf_mex_destroy(&c1_b_PSIwp);
}

static void c1_f_emlrt_marshallIn
  (SFc1_sl_quadrotorVelocityGuidanceInstanceStruct *chartInstance, const mxArray
   *c1_u, const emlrtMsgIdentifier *c1_parentId, real_T c1_y[15])
{
  real_T c1_dv8[15];
  int32_T c1_i57;
  (void)chartInstance;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), c1_dv8, 1, 0, 0U, 1, 0U, 2, 1, 15);
  for (c1_i57 = 0; c1_i57 < 15; c1_i57++) {
    c1_y[c1_i57] = c1_dv8[c1_i57];
  }

  sf_mex_destroy(&c1_u);
}

static void c1_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_b_PSIwp;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  real_T c1_y[15];
  int32_T c1_i58;
  SFc1_sl_quadrotorVelocityGuidanceInstanceStruct *chartInstance;
  chartInstance = (SFc1_sl_quadrotorVelocityGuidanceInstanceStruct *)
    chartInstanceVoid;
  c1_b_PSIwp = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_PSIwp), &c1_thisId, c1_y);
  sf_mex_destroy(&c1_b_PSIwp);
  for (c1_i58 = 0; c1_i58 < 15; c1_i58++) {
    (*(real_T (*)[15])c1_outData)[c1_i58] = c1_y[c1_i58];
  }

  sf_mex_destroy(&c1_mxArrayInData);
}

static const mxArray *c1_d_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  real_T c1_u;
  const mxArray *c1_y = NULL;
  SFc1_sl_quadrotorVelocityGuidanceInstanceStruct *chartInstance;
  chartInstance = (SFc1_sl_quadrotorVelocityGuidanceInstanceStruct *)
    chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_u = *(real_T *)c1_inData;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", &c1_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c1_mxArrayOutData, c1_y, false);
  return c1_mxArrayOutData;
}

static real_T c1_g_emlrt_marshallIn
  (SFc1_sl_quadrotorVelocityGuidanceInstanceStruct *chartInstance, const mxArray
   *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  real_T c1_y;
  real_T c1_d1;
  (void)chartInstance;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_d1, 1, 0, 0U, 0, 0U, 0);
  c1_y = c1_d1;
  sf_mex_destroy(&c1_u);
  return c1_y;
}

static void c1_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_nargout;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  real_T c1_y;
  SFc1_sl_quadrotorVelocityGuidanceInstanceStruct *chartInstance;
  chartInstance = (SFc1_sl_quadrotorVelocityGuidanceInstanceStruct *)
    chartInstanceVoid;
  c1_nargout = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_g_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_nargout), &c1_thisId);
  sf_mex_destroy(&c1_nargout);
  *(real_T *)c1_outData = c1_y;
  sf_mex_destroy(&c1_mxArrayInData);
}

static const mxArray *c1_e_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  int32_T c1_i59;
  real_T c1_b_inData[2];
  int32_T c1_i60;
  real_T c1_u[2];
  const mxArray *c1_y = NULL;
  SFc1_sl_quadrotorVelocityGuidanceInstanceStruct *chartInstance;
  chartInstance = (SFc1_sl_quadrotorVelocityGuidanceInstanceStruct *)
    chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  for (c1_i59 = 0; c1_i59 < 2; c1_i59++) {
    c1_b_inData[c1_i59] = (*(real_T (*)[2])c1_inData)[c1_i59];
  }

  for (c1_i60 = 0; c1_i60 < 2; c1_i60++) {
    c1_u[c1_i60] = c1_b_inData[c1_i60];
  }

  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", c1_u, 0, 0U, 1U, 0U, 2, 1, 2), false);
  sf_mex_assign(&c1_mxArrayOutData, c1_y, false);
  return c1_mxArrayOutData;
}

static void c1_h_emlrt_marshallIn
  (SFc1_sl_quadrotorVelocityGuidanceInstanceStruct *chartInstance, const mxArray
   *c1_u, const emlrtMsgIdentifier *c1_parentId, real_T c1_y[2])
{
  real_T c1_dv9[2];
  int32_T c1_i61;
  (void)chartInstance;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), c1_dv9, 1, 0, 0U, 1, 0U, 2, 1, 2);
  for (c1_i61 = 0; c1_i61 < 2; c1_i61++) {
    c1_y[c1_i61] = c1_dv9[c1_i61];
  }

  sf_mex_destroy(&c1_u);
}

static void c1_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_sizeA;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  real_T c1_y[2];
  int32_T c1_i62;
  SFc1_sl_quadrotorVelocityGuidanceInstanceStruct *chartInstance;
  chartInstance = (SFc1_sl_quadrotorVelocityGuidanceInstanceStruct *)
    chartInstanceVoid;
  c1_sizeA = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_h_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_sizeA), &c1_thisId, c1_y);
  sf_mex_destroy(&c1_sizeA);
  for (c1_i62 = 0; c1_i62 < 2; c1_i62++) {
    (*(real_T (*)[2])c1_outData)[c1_i62] = c1_y[c1_i62];
  }

  sf_mex_destroy(&c1_mxArrayInData);
}

static const mxArray *c1_f_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  int32_T c1_i63;
  char_T c1_b_inData[11];
  int32_T c1_i64;
  char_T c1_u[11];
  const mxArray *c1_y = NULL;
  SFc1_sl_quadrotorVelocityGuidanceInstanceStruct *chartInstance;
  chartInstance = (SFc1_sl_quadrotorVelocityGuidanceInstanceStruct *)
    chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  for (c1_i63 = 0; c1_i63 < 11; c1_i63++) {
    c1_b_inData[c1_i63] = (*(char_T (*)[11])c1_inData)[c1_i63];
  }

  for (c1_i64 = 0; c1_i64 < 11; c1_i64++) {
    c1_u[c1_i64] = c1_b_inData[c1_i64];
  }

  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", c1_u, 10, 0U, 1U, 0U, 2, 1, 11), false);
  sf_mex_assign(&c1_mxArrayOutData, c1_y, false);
  return c1_mxArrayOutData;
}

static void c1_i_emlrt_marshallIn
  (SFc1_sl_quadrotorVelocityGuidanceInstanceStruct *chartInstance, const mxArray
   *c1_u, const emlrtMsgIdentifier *c1_parentId, char_T c1_y[11])
{
  char_T c1_cv2[11];
  int32_T c1_i65;
  (void)chartInstance;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), c1_cv2, 1, 10, 0U, 1, 0U, 2, 1,
                11);
  for (c1_i65 = 0; c1_i65 < 11; c1_i65++) {
    c1_y[c1_i65] = c1_cv2[c1_i65];
  }

  sf_mex_destroy(&c1_u);
}

static void c1_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_formatSpec;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  char_T c1_y[11];
  int32_T c1_i66;
  SFc1_sl_quadrotorVelocityGuidanceInstanceStruct *chartInstance;
  chartInstance = (SFc1_sl_quadrotorVelocityGuidanceInstanceStruct *)
    chartInstanceVoid;
  c1_formatSpec = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_i_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_formatSpec), &c1_thisId,
                        c1_y);
  sf_mex_destroy(&c1_formatSpec);
  for (c1_i66 = 0; c1_i66 < 11; c1_i66++) {
    (*(char_T (*)[11])c1_outData)[c1_i66] = c1_y[c1_i66];
  }

  sf_mex_destroy(&c1_mxArrayInData);
}

static const mxArray *c1_g_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  const mxArray *c1_u;
  const mxArray *c1_y = NULL;
  SFc1_sl_quadrotorVelocityGuidanceInstanceStruct *chartInstance;
  chartInstance = (SFc1_sl_quadrotorVelocityGuidanceInstanceStruct *)
    chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_u = sf_mex_dup(*(const mxArray **)c1_inData);
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_duplicatearraysafe(&c1_u), false);
  sf_mex_destroy(&c1_u);
  sf_mex_assign(&c1_mxArrayOutData, c1_y, false);
  return c1_mxArrayOutData;
}

const mxArray
  *sf_c1_sl_quadrotorVelocityGuidance_get_eml_resolved_functions_info(void)
{
  const mxArray *c1_nameCaptureInfo = NULL;
  c1_nameCaptureInfo = NULL;
  sf_mex_assign(&c1_nameCaptureInfo, sf_mex_createstruct("structure", 2, 2, 1),
                false);
  c1_info_helper(&c1_nameCaptureInfo);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c1_nameCaptureInfo);
  return c1_nameCaptureInfo;
}

static void c1_info_helper(const mxArray **c1_info)
{
  const mxArray *c1_rhs0 = NULL;
  const mxArray *c1_lhs0 = NULL;
  const mxArray *c1_rhs1 = NULL;
  const mxArray *c1_lhs1 = NULL;
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(""), "context", "context", 0);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("fopen"), "name", "name", 0);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 0);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/iofun/fopen.m"), "resolved",
                  "resolved", 0);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1381821500U), "fileTimeLo",
                  "fileTimeLo", 0);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 0);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 0);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 0);
  sf_mex_assign(&c1_rhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs0), "rhs", "rhs", 0);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs0), "lhs", "lhs", 0);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(""), "context", "context", 1);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("fclose"), "name", "name", 1);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("mxArray"), "dominantType",
                  "dominantType", 1);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/iofun/fclose.m"), "resolved",
                  "resolved", 1);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1381821500U), "fileTimeLo",
                  "fileTimeLo", 1);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 1);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 1);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 1);
  sf_mex_assign(&c1_rhs1, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs1, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs1), "rhs", "rhs", 1);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs1), "lhs", "lhs", 1);
  sf_mex_destroy(&c1_rhs0);
  sf_mex_destroy(&c1_lhs0);
  sf_mex_destroy(&c1_rhs1);
  sf_mex_destroy(&c1_lhs1);
}

static const mxArray *c1_emlrt_marshallOut(const char * c1_u)
{
  const mxArray *c1_y = NULL;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", c1_u, 15, 0U, 0U, 0U, 2, 1, strlen
    (c1_u)), false);
  return c1_y;
}

static const mxArray *c1_b_emlrt_marshallOut(const uint32_T c1_u)
{
  const mxArray *c1_y = NULL;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", &c1_u, 7, 0U, 0U, 0U, 0), false);
  return c1_y;
}

static void c1_j_emlrt_marshallIn
  (SFc1_sl_quadrotorVelocityGuidanceInstanceStruct *chartInstance, const mxArray
   *c1_fscanf, const char_T *c1_identifier, real_T c1_y[60])
{
  emlrtMsgIdentifier c1_thisId;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_k_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_fscanf), &c1_thisId, c1_y);
  sf_mex_destroy(&c1_fscanf);
}

static void c1_k_emlrt_marshallIn
  (SFc1_sl_quadrotorVelocityGuidanceInstanceStruct *chartInstance, const mxArray
   *c1_u, const emlrtMsgIdentifier *c1_parentId, real_T c1_y[60])
{
  real_T c1_dv10[60];
  int32_T c1_i67;
  (void)chartInstance;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), c1_dv10, 1, 0, 0U, 1, 0U, 2, 4,
                15);
  for (c1_i67 = 0; c1_i67 < 60; c1_i67++) {
    c1_y[c1_i67] = c1_dv10[c1_i67];
  }

  sf_mex_destroy(&c1_u);
}

static const mxArray *c1_h_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  int32_T c1_u;
  const mxArray *c1_y = NULL;
  SFc1_sl_quadrotorVelocityGuidanceInstanceStruct *chartInstance;
  chartInstance = (SFc1_sl_quadrotorVelocityGuidanceInstanceStruct *)
    chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_u = *(int32_T *)c1_inData;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", &c1_u, 6, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c1_mxArrayOutData, c1_y, false);
  return c1_mxArrayOutData;
}

static int32_T c1_l_emlrt_marshallIn
  (SFc1_sl_quadrotorVelocityGuidanceInstanceStruct *chartInstance, const mxArray
   *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  int32_T c1_y;
  int32_T c1_i68;
  (void)chartInstance;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_i68, 1, 6, 0U, 0, 0U, 0);
  c1_y = c1_i68;
  sf_mex_destroy(&c1_u);
  return c1_y;
}

static void c1_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_b_sfEvent;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  int32_T c1_y;
  SFc1_sl_quadrotorVelocityGuidanceInstanceStruct *chartInstance;
  chartInstance = (SFc1_sl_quadrotorVelocityGuidanceInstanceStruct *)
    chartInstanceVoid;
  c1_b_sfEvent = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_l_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_sfEvent),
    &c1_thisId);
  sf_mex_destroy(&c1_b_sfEvent);
  *(int32_T *)c1_outData = c1_y;
  sf_mex_destroy(&c1_mxArrayInData);
}

static uint8_T c1_m_emlrt_marshallIn
  (SFc1_sl_quadrotorVelocityGuidanceInstanceStruct *chartInstance, const mxArray
   *c1_b_is_active_c1_sl_quadrotorVelocityGuidance, const char_T *c1_identifier)
{
  uint8_T c1_y;
  emlrtMsgIdentifier c1_thisId;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_n_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c1_b_is_active_c1_sl_quadrotorVelocityGuidance), &c1_thisId);
  sf_mex_destroy(&c1_b_is_active_c1_sl_quadrotorVelocityGuidance);
  return c1_y;
}

static uint8_T c1_n_emlrt_marshallIn
  (SFc1_sl_quadrotorVelocityGuidanceInstanceStruct *chartInstance, const mxArray
   *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  uint8_T c1_y;
  uint8_T c1_u0;
  (void)chartInstance;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_u0, 1, 3, 0U, 0, 0U, 0);
  c1_y = c1_u0;
  sf_mex_destroy(&c1_u);
  return c1_y;
}

static void init_dsm_address_info
  (SFc1_sl_quadrotorVelocityGuidanceInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void init_simulink_io_address
  (SFc1_sl_quadrotorVelocityGuidanceInstanceStruct *chartInstance)
{
  chartInstance->c1_Xwp = (real_T (*)[15])ssGetOutputPortSignal_wrapper
    (chartInstance->S, 1);
  chartInstance->c1_Ywp = (real_T (*)[15])ssGetOutputPortSignal_wrapper
    (chartInstance->S, 2);
  chartInstance->c1_Zwp = (real_T (*)[15])ssGetOutputPortSignal_wrapper
    (chartInstance->S, 3);
  chartInstance->c1_PSIwp = (real_T (*)[15])ssGetOutputPortSignal_wrapper
    (chartInstance->S, 4);
  chartInstance->c1_XwpIn = (real_T (*)[15])ssGetInputPortSignal_wrapper
    (chartInstance->S, 0);
  chartInstance->c1_YwpIn = (real_T (*)[15])ssGetInputPortSignal_wrapper
    (chartInstance->S, 1);
  chartInstance->c1_ZwpIn = (real_T (*)[15])ssGetInputPortSignal_wrapper
    (chartInstance->S, 2);
  chartInstance->c1_PSIwpIn = (real_T (*)[15])ssGetInputPortSignal_wrapper
    (chartInstance->S, 3);
}

/* SFunction Glue Code */
#ifdef utFree
#undef utFree
#endif

#ifdef utMalloc
#undef utMalloc
#endif

#ifdef __cplusplus

extern "C" void *utMalloc(size_t size);
extern "C" void utFree(void*);

#else

extern void *utMalloc(size_t size);
extern void utFree(void*);

#endif

void sf_c1_sl_quadrotorVelocityGuidance_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(1361548171U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(1650415422U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(2005929577U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(3529574316U);
}

mxArray* sf_c1_sl_quadrotorVelocityGuidance_get_post_codegen_info(void);
mxArray *sf_c1_sl_quadrotorVelocityGuidance_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals", "postCodegenInfo" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1, 1, sizeof
    (autoinheritanceFields)/sizeof(autoinheritanceFields[0]),
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("oGZEL6SpfHAROZ8NqMiIq");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,4,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(15);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(15);
      mxSetField(mxData,1,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,1,"type",mxType);
    }

    mxSetField(mxData,1,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(15);
      mxSetField(mxData,2,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,2,"type",mxType);
    }

    mxSetField(mxData,2,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(15);
      mxSetField(mxData,3,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,3,"type",mxType);
    }

    mxSetField(mxData,3,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"inputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"parameters",mxCreateDoubleMatrix(0,0,
                mxREAL));
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,4,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(15);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(15);
      mxSetField(mxData,1,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,1,"type",mxType);
    }

    mxSetField(mxData,1,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(15);
      mxSetField(mxData,2,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,2,"type",mxType);
    }

    mxSetField(mxData,2,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(15);
      mxSetField(mxData,3,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,3,"type",mxType);
    }

    mxSetField(mxData,3,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  {
    mxArray* mxPostCodegenInfo =
      sf_c1_sl_quadrotorVelocityGuidance_get_post_codegen_info();
    mxSetField(mxAutoinheritanceInfo,0,"postCodegenInfo",mxPostCodegenInfo);
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c1_sl_quadrotorVelocityGuidance_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

mxArray *sf_c1_sl_quadrotorVelocityGuidance_jit_fallback_info(void)
{
  const char *infoFields[] = { "fallbackType", "fallbackReason",
    "incompatibleSymbol", };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 3, infoFields);
  mxArray *fallbackReason = mxCreateString("feature_off");
  mxArray *incompatibleSymbol = mxCreateString("");
  mxArray *fallbackType = mxCreateString("early");
  mxSetField(mxInfo, 0, infoFields[0], fallbackType);
  mxSetField(mxInfo, 0, infoFields[1], fallbackReason);
  mxSetField(mxInfo, 0, infoFields[2], incompatibleSymbol);
  return mxInfo;
}

mxArray *sf_c1_sl_quadrotorVelocityGuidance_updateBuildInfo_args_info(void)
{
  mxArray *mxBIArgs = mxCreateCellMatrix(1,0);
  return mxBIArgs;
}

mxArray* sf_c1_sl_quadrotorVelocityGuidance_get_post_codegen_info(void)
{
  const char* fieldNames[] = { "exportedFunctionsUsedByThisChart",
    "exportedFunctionsChecksum" };

  mwSize dims[2] = { 1, 1 };

  mxArray* mxPostCodegenInfo = mxCreateStructArray(2, dims, sizeof(fieldNames)/
    sizeof(fieldNames[0]), fieldNames);

  {
    mxArray* mxExportedFunctionsChecksum = mxCreateString("");
    mwSize exp_dims[2] = { 0, 1 };

    mxArray* mxExportedFunctionsUsedByThisChart = mxCreateCellArray(2, exp_dims);
    mxSetField(mxPostCodegenInfo, 0, "exportedFunctionsUsedByThisChart",
               mxExportedFunctionsUsedByThisChart);
    mxSetField(mxPostCodegenInfo, 0, "exportedFunctionsChecksum",
               mxExportedFunctionsChecksum);
  }

  return mxPostCodegenInfo;
}

static const mxArray *sf_get_sim_state_info_c1_sl_quadrotorVelocityGuidance(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x7'type','srcId','name','auxInfo'{{M[1],M[8],T\"PSIwp\",},{M[1],M[5],T\"Xwp\",},{M[1],M[6],T\"Ywp\",},{M[1],M[7],T\"Zwp\",},{M[4],M[0],T\"S\",S'l','i','p'{{M1x2[252 253],M[0],}}},{M[4],M[0],T\"firstentry\",S'l','i','p'{{M1x2[266 276],M[0],}}},{M[8],M[0],T\"is_active_c1_sl_quadrotorVelocityGuidance\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 7, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c1_sl_quadrotorVelocityGuidance_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc1_sl_quadrotorVelocityGuidanceInstanceStruct *chartInstance;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
    chartInstance = (SFc1_sl_quadrotorVelocityGuidanceInstanceStruct *)
      chartInfo->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _sl_quadrotorVelocityGuidanceMachineNumber_,
           1,
           1,
           1,
           0,
           8,
           0,
           0,
           0,
           0,
           0,
           &(chartInstance->chartNumber),
           &(chartInstance->instanceNumber),
           (void *)S);

        /* Each instance must initialize its own list of scripts */
        init_script_number_translation
          (_sl_quadrotorVelocityGuidanceMachineNumber_,
           chartInstance->chartNumber,chartInstance->instanceNumber);
        if (chartAlreadyPresent==0) {
          /* this is the first instance */
          sf_debug_set_chart_disable_implicit_casting
            (sfGlobalDebugInstanceStruct,
             _sl_quadrotorVelocityGuidanceMachineNumber_,
             chartInstance->chartNumber,1);
          sf_debug_set_chart_event_thresholds(sfGlobalDebugInstanceStruct,
            _sl_quadrotorVelocityGuidanceMachineNumber_,
            chartInstance->chartNumber,
            0,
            0,
            0);
          _SFD_SET_DATA_PROPS(0,2,0,1,"Xwp");
          _SFD_SET_DATA_PROPS(1,2,0,1,"Ywp");
          _SFD_SET_DATA_PROPS(2,2,0,1,"Zwp");
          _SFD_SET_DATA_PROPS(3,2,0,1,"PSIwp");
          _SFD_SET_DATA_PROPS(4,1,1,0,"XwpIn");
          _SFD_SET_DATA_PROPS(5,1,1,0,"YwpIn");
          _SFD_SET_DATA_PROPS(6,1,1,0,"ZwpIn");
          _SFD_SET_DATA_PROPS(7,1,1,0,"PSIwpIn");
          _SFD_STATE_INFO(0,0,2);
          _SFD_CH_SUBSTATE_COUNT(0);
          _SFD_CH_SUBSTATE_DECOMP(0);
        }

        _SFD_CV_INIT_CHART(0,0,0,0);

        {
          _SFD_CV_INIT_STATE(0,0,0,0,0,0,NULL,NULL);
        }

        _SFD_CV_INIT_TRANS(0,0,NULL,NULL,0,NULL);

        /* Initialization of MATLAB Function Model Coverage */
        _SFD_CV_INIT_EML(0,1,1,2,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,855);
        _SFD_CV_INIT_EML_IF(0,1,0,342,364,-1,388);
        _SFD_CV_INIT_EML_IF(0,1,1,510,528,771,854);
        _SFD_CV_INIT_EML_RELATIONAL(0,1,0,513,528,-1,0);

        {
          unsigned int dimVector[2];
          dimVector[0]= 1;
          dimVector[1]= 15;
          _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c1_c_sf_marshallOut,(MexInFcnForType)
            c1_c_sf_marshallIn);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 1;
          dimVector[1]= 15;
          _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c1_c_sf_marshallOut,(MexInFcnForType)
            c1_c_sf_marshallIn);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 1;
          dimVector[1]= 15;
          _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c1_c_sf_marshallOut,(MexInFcnForType)
            c1_c_sf_marshallIn);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 1;
          dimVector[1]= 15;
          _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c1_c_sf_marshallOut,(MexInFcnForType)
            c1_c_sf_marshallIn);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 1;
          dimVector[1]= 15;
          _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c1_c_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 1;
          dimVector[1]= 15;
          _SFD_SET_DATA_COMPILED_PROPS(5,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c1_c_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 1;
          dimVector[1]= 15;
          _SFD_SET_DATA_COMPILED_PROPS(6,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c1_c_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 1;
          dimVector[1]= 15;
          _SFD_SET_DATA_COMPILED_PROPS(7,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c1_c_sf_marshallOut,(MexInFcnForType)NULL);
        }

        _SFD_SET_DATA_VALUE_PTR(0U, *chartInstance->c1_Xwp);
        _SFD_SET_DATA_VALUE_PTR(1U, *chartInstance->c1_Ywp);
        _SFD_SET_DATA_VALUE_PTR(2U, *chartInstance->c1_Zwp);
        _SFD_SET_DATA_VALUE_PTR(3U, *chartInstance->c1_PSIwp);
        _SFD_SET_DATA_VALUE_PTR(4U, *chartInstance->c1_XwpIn);
        _SFD_SET_DATA_VALUE_PTR(5U, *chartInstance->c1_YwpIn);
        _SFD_SET_DATA_VALUE_PTR(6U, *chartInstance->c1_ZwpIn);
        _SFD_SET_DATA_VALUE_PTR(7U, *chartInstance->c1_PSIwpIn);
      }
    } else {
      sf_debug_reset_current_state_configuration(sfGlobalDebugInstanceStruct,
        _sl_quadrotorVelocityGuidanceMachineNumber_,chartInstance->chartNumber,
        chartInstance->instanceNumber);
    }
  }
}

static const char* sf_get_instance_specialization(void)
{
  return "uAUOkyd7x48D5r9S6hUUi";
}

static void sf_opaque_initialize_c1_sl_quadrotorVelocityGuidance(void
  *chartInstanceVar)
{
  chart_debug_initialization(((SFc1_sl_quadrotorVelocityGuidanceInstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c1_sl_quadrotorVelocityGuidance
    ((SFc1_sl_quadrotorVelocityGuidanceInstanceStruct*) chartInstanceVar);
  initialize_c1_sl_quadrotorVelocityGuidance
    ((SFc1_sl_quadrotorVelocityGuidanceInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_enable_c1_sl_quadrotorVelocityGuidance(void
  *chartInstanceVar)
{
  enable_c1_sl_quadrotorVelocityGuidance
    ((SFc1_sl_quadrotorVelocityGuidanceInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_disable_c1_sl_quadrotorVelocityGuidance(void
  *chartInstanceVar)
{
  disable_c1_sl_quadrotorVelocityGuidance
    ((SFc1_sl_quadrotorVelocityGuidanceInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_gateway_c1_sl_quadrotorVelocityGuidance(void
  *chartInstanceVar)
{
  sf_gateway_c1_sl_quadrotorVelocityGuidance
    ((SFc1_sl_quadrotorVelocityGuidanceInstanceStruct*) chartInstanceVar);
}

static const mxArray* sf_opaque_get_sim_state_c1_sl_quadrotorVelocityGuidance
  (SimStruct* S)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  return get_sim_state_c1_sl_quadrotorVelocityGuidance
    ((SFc1_sl_quadrotorVelocityGuidanceInstanceStruct*)chartInfo->chartInstance);/* raw sim ctx */
}

static void sf_opaque_set_sim_state_c1_sl_quadrotorVelocityGuidance(SimStruct* S,
  const mxArray *st)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  set_sim_state_c1_sl_quadrotorVelocityGuidance
    ((SFc1_sl_quadrotorVelocityGuidanceInstanceStruct*)chartInfo->chartInstance,
     st);
}

static void sf_opaque_terminate_c1_sl_quadrotorVelocityGuidance(void
  *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc1_sl_quadrotorVelocityGuidanceInstanceStruct*)
                    chartInstanceVar)->S;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_sl_quadrotorVelocityGuidance_optimization_info();
    }

    finalize_c1_sl_quadrotorVelocityGuidance
      ((SFc1_sl_quadrotorVelocityGuidanceInstanceStruct*) chartInstanceVar);
    utFree(chartInstanceVar);
    if (crtInfo != NULL) {
      utFree(crtInfo);
    }

    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc1_sl_quadrotorVelocityGuidance
    ((SFc1_sl_quadrotorVelocityGuidanceInstanceStruct*) chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c1_sl_quadrotorVelocityGuidance(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
    initialize_params_c1_sl_quadrotorVelocityGuidance
      ((SFc1_sl_quadrotorVelocityGuidanceInstanceStruct*)
       (chartInfo->chartInstance));
  }
}

static void mdlSetWorkWidths_c1_sl_quadrotorVelocityGuidance(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_sl_quadrotorVelocityGuidance_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(sf_get_instance_specialization(),infoStruct,1);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(sf_get_instance_specialization(),
                infoStruct,1,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop
      (sf_get_instance_specialization(),infoStruct,1,
       "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(sf_get_instance_specialization(),infoStruct,1);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 3, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,1,4);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,1,4);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=4; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 4; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,1);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(1301623741U));
  ssSetChecksum1(S,(2110887469U));
  ssSetChecksum2(S,(927222075U));
  ssSetChecksum3(S,(3931190673U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c1_sl_quadrotorVelocityGuidance(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c1_sl_quadrotorVelocityGuidance(SimStruct *S)
{
  SFc1_sl_quadrotorVelocityGuidanceInstanceStruct *chartInstance;
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)utMalloc(sizeof
    (ChartRunTimeInfo));
  chartInstance = (SFc1_sl_quadrotorVelocityGuidanceInstanceStruct *)utMalloc
    (sizeof(SFc1_sl_quadrotorVelocityGuidanceInstanceStruct));
  memset(chartInstance, 0, sizeof
         (SFc1_sl_quadrotorVelocityGuidanceInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c1_sl_quadrotorVelocityGuidance;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c1_sl_quadrotorVelocityGuidance;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c1_sl_quadrotorVelocityGuidance;
  chartInstance->chartInfo.enableChart =
    sf_opaque_enable_c1_sl_quadrotorVelocityGuidance;
  chartInstance->chartInfo.disableChart =
    sf_opaque_disable_c1_sl_quadrotorVelocityGuidance;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c1_sl_quadrotorVelocityGuidance;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c1_sl_quadrotorVelocityGuidance;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c1_sl_quadrotorVelocityGuidance;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c1_sl_quadrotorVelocityGuidance;
  chartInstance->chartInfo.mdlStart = mdlStart_c1_sl_quadrotorVelocityGuidance;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c1_sl_quadrotorVelocityGuidance;
  chartInstance->chartInfo.extModeExec = NULL;
  chartInstance->chartInfo.restoreLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.restoreBeforeLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.storeCurrentConfiguration = NULL;
  chartInstance->chartInfo.callAtomicSubchartUserFcn = NULL;
  chartInstance->chartInfo.callAtomicSubchartAutoFcn = NULL;
  chartInstance->chartInfo.debugInstance = sfGlobalDebugInstanceStruct;
  chartInstance->S = S;
  crtInfo->checksum = SF_RUNTIME_INFO_CHECKSUM;
  crtInfo->instanceInfo = (&(chartInstance->chartInfo));
  crtInfo->isJITEnabled = false;
  crtInfo->compiledInfo = NULL;
  ssSetUserData(S,(void *)(crtInfo));  /* register the chart instance with simstruct */
  init_dsm_address_info(chartInstance);
  init_simulink_io_address(chartInstance);
  if (!sim_mode_is_rtw_gen(S)) {
  }

  sf_opaque_init_subchart_simstructs(chartInstance->chartInfo.chartInstance);
  chart_debug_initialization(S,1);
}

void c1_sl_quadrotorVelocityGuidance_method_dispatcher(SimStruct *S, int_T
  method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c1_sl_quadrotorVelocityGuidance(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c1_sl_quadrotorVelocityGuidance(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c1_sl_quadrotorVelocityGuidance(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c1_sl_quadrotorVelocityGuidance_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
