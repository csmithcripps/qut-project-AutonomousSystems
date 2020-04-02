/* Include files */

#include <stddef.h>
#include "blas.h"
#include "sl_quadrotorAttitudeGuidance_sfun.h"
#include "c5_sl_quadrotorAttitudeGuidance.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "sl_quadrotorAttitudeGuidance_sfun_debug_macros.h"
#define _SF_MEX_LISTEN_FOR_CTRL_C(S)   sf_mex_listen_for_ctrl_c(sfGlobalDebugInstanceStruct,S);

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static real_T _sfTime_;
static const char * c5_debug_family_names[6] = { "nargin", "nargout", "x_wp",
  "y_wp", "z_wp", "X" };

/* Function Declarations */
static void initialize_c5_sl_quadrotorAttitudeGuidance
  (SFc5_sl_quadrotorAttitudeGuidanceInstanceStruct *chartInstance);
static void initialize_params_c5_sl_quadrotorAttitudeGuidance
  (SFc5_sl_quadrotorAttitudeGuidanceInstanceStruct *chartInstance);
static void enable_c5_sl_quadrotorAttitudeGuidance
  (SFc5_sl_quadrotorAttitudeGuidanceInstanceStruct *chartInstance);
static void disable_c5_sl_quadrotorAttitudeGuidance
  (SFc5_sl_quadrotorAttitudeGuidanceInstanceStruct *chartInstance);
static void c5_update_debugger_state_c5_sl_quadrotorAttitudeGuidance
  (SFc5_sl_quadrotorAttitudeGuidanceInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c5_sl_quadrotorAttitudeGuidance
  (SFc5_sl_quadrotorAttitudeGuidanceInstanceStruct *chartInstance);
static void set_sim_state_c5_sl_quadrotorAttitudeGuidance
  (SFc5_sl_quadrotorAttitudeGuidanceInstanceStruct *chartInstance, const mxArray
   *c5_st);
static void finalize_c5_sl_quadrotorAttitudeGuidance
  (SFc5_sl_quadrotorAttitudeGuidanceInstanceStruct *chartInstance);
static void sf_gateway_c5_sl_quadrotorAttitudeGuidance
  (SFc5_sl_quadrotorAttitudeGuidanceInstanceStruct *chartInstance);
static void mdl_start_c5_sl_quadrotorAttitudeGuidance
  (SFc5_sl_quadrotorAttitudeGuidanceInstanceStruct *chartInstance);
static void initSimStructsc5_sl_quadrotorAttitudeGuidance
  (SFc5_sl_quadrotorAttitudeGuidanceInstanceStruct *chartInstance);
static void init_script_number_translation(uint32_T c5_machineNumber, uint32_T
  c5_chartNumber, uint32_T c5_instanceNumber);
static const mxArray *c5_sf_marshallOut(void *chartInstanceVoid, void *c5_inData);
static const mxArray *c5_b_sf_marshallOut(void *chartInstanceVoid, void
  *c5_inData);
static real_T c5_emlrt_marshallIn
  (SFc5_sl_quadrotorAttitudeGuidanceInstanceStruct *chartInstance, const mxArray
   *c5_u, const emlrtMsgIdentifier *c5_parentId);
static void c5_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData);
static void c5_info_helper(const mxArray **c5_info);
static const mxArray *c5_emlrt_marshallOut(const char * c5_u);
static const mxArray *c5_b_emlrt_marshallOut(const uint32_T c5_u);
static const mxArray *c5_c_sf_marshallOut(void *chartInstanceVoid, void
  *c5_inData);
static int32_T c5_b_emlrt_marshallIn
  (SFc5_sl_quadrotorAttitudeGuidanceInstanceStruct *chartInstance, const mxArray
   *c5_u, const emlrtMsgIdentifier *c5_parentId);
static void c5_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData);
static uint8_T c5_c_emlrt_marshallIn
  (SFc5_sl_quadrotorAttitudeGuidanceInstanceStruct *chartInstance, const mxArray
   *c5_b_is_active_c5_sl_quadrotorAttitudeGuidance, const char_T *c5_identifier);
static uint8_T c5_d_emlrt_marshallIn
  (SFc5_sl_quadrotorAttitudeGuidanceInstanceStruct *chartInstance, const mxArray
   *c5_u, const emlrtMsgIdentifier *c5_parentId);
static void init_dsm_address_info
  (SFc5_sl_quadrotorAttitudeGuidanceInstanceStruct *chartInstance);
static void init_simulink_io_address
  (SFc5_sl_quadrotorAttitudeGuidanceInstanceStruct *chartInstance);

/* Function Definitions */
static void initialize_c5_sl_quadrotorAttitudeGuidance
  (SFc5_sl_quadrotorAttitudeGuidanceInstanceStruct *chartInstance)
{
  chartInstance->c5_sfEvent = CALL_EVENT;
  _sfTime_ = sf_get_time(chartInstance->S);
  chartInstance->c5_is_active_c5_sl_quadrotorAttitudeGuidance = 0U;
}

static void initialize_params_c5_sl_quadrotorAttitudeGuidance
  (SFc5_sl_quadrotorAttitudeGuidanceInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void enable_c5_sl_quadrotorAttitudeGuidance
  (SFc5_sl_quadrotorAttitudeGuidanceInstanceStruct *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void disable_c5_sl_quadrotorAttitudeGuidance
  (SFc5_sl_quadrotorAttitudeGuidanceInstanceStruct *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void c5_update_debugger_state_c5_sl_quadrotorAttitudeGuidance
  (SFc5_sl_quadrotorAttitudeGuidanceInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static const mxArray *get_sim_state_c5_sl_quadrotorAttitudeGuidance
  (SFc5_sl_quadrotorAttitudeGuidanceInstanceStruct *chartInstance)
{
  const mxArray *c5_st;
  const mxArray *c5_y = NULL;
  uint8_T c5_hoistedGlobal;
  uint8_T c5_u;
  const mxArray *c5_b_y = NULL;
  c5_st = NULL;
  c5_st = NULL;
  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_createcellmatrix(1, 1), false);
  c5_hoistedGlobal = chartInstance->c5_is_active_c5_sl_quadrotorAttitudeGuidance;
  c5_u = c5_hoistedGlobal;
  c5_b_y = NULL;
  sf_mex_assign(&c5_b_y, sf_mex_create("y", &c5_u, 3, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c5_y, 0, c5_b_y);
  sf_mex_assign(&c5_st, c5_y, false);
  return c5_st;
}

static void set_sim_state_c5_sl_quadrotorAttitudeGuidance
  (SFc5_sl_quadrotorAttitudeGuidanceInstanceStruct *chartInstance, const mxArray
   *c5_st)
{
  const mxArray *c5_u;
  chartInstance->c5_doneDoubleBufferReInit = true;
  c5_u = sf_mex_dup(c5_st);
  chartInstance->c5_is_active_c5_sl_quadrotorAttitudeGuidance =
    c5_c_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c5_u, 0)),
    "is_active_c5_sl_quadrotorAttitudeGuidance");
  sf_mex_destroy(&c5_u);
  c5_update_debugger_state_c5_sl_quadrotorAttitudeGuidance(chartInstance);
  sf_mex_destroy(&c5_st);
}

static void finalize_c5_sl_quadrotorAttitudeGuidance
  (SFc5_sl_quadrotorAttitudeGuidanceInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void sf_gateway_c5_sl_quadrotorAttitudeGuidance
  (SFc5_sl_quadrotorAttitudeGuidanceInstanceStruct *chartInstance)
{
  int32_T c5_i0;
  real_T c5_hoistedGlobal;
  real_T c5_b_hoistedGlobal;
  real_T c5_c_hoistedGlobal;
  real_T c5_b_x_wp;
  real_T c5_b_y_wp;
  real_T c5_b_z_wp;
  int32_T c5_i1;
  real_T c5_b_X[12];
  uint32_T c5_debug_family_var_map[6];
  real_T c5_nargin = 4.0;
  real_T c5_nargout = 0.0;
  real_T c5_u;
  const mxArray *c5_y = NULL;
  real_T c5_b_u;
  const mxArray *c5_b_y = NULL;
  real_T c5_c_u;
  const mxArray *c5_c_y = NULL;
  int32_T c5_i2;
  static char_T c5_cv0[2] = { 'r', '*' };

  char_T c5_d_u[2];
  const mxArray *c5_d_y = NULL;
  real_T c5_e_u;
  const mxArray *c5_e_y = NULL;
  real_T c5_f_u;
  const mxArray *c5_f_y = NULL;
  real_T c5_g_u;
  const mxArray *c5_g_y = NULL;
  int32_T c5_i3;
  static char_T c5_cv1[2] = { 'r', '+' };

  char_T c5_h_u[2];
  const mxArray *c5_h_y = NULL;
  real_T c5_i_u;
  const mxArray *c5_i_y = NULL;
  real_T c5_j_u;
  const mxArray *c5_j_y = NULL;
  real_T c5_k_u;
  const mxArray *c5_k_y = NULL;
  int32_T c5_i4;
  static char_T c5_cv2[2] = { 'g', '*' };

  char_T c5_l_u[2];
  const mxArray *c5_l_y = NULL;
  real_T c5_m_u[2];
  const mxArray *c5_m_y = NULL;
  real_T c5_n_u[2];
  const mxArray *c5_n_y = NULL;
  real_T c5_o_u[2];
  const mxArray *c5_o_y = NULL;
  int32_T c5_i5;
  static char_T c5_cv3[3] = { 'b', '-', '-' };

  char_T c5_p_u[3];
  const mxArray *c5_p_y = NULL;
  int32_T c5_i6;
  static char_T c5_cv4[4] = { 'Y', 'D', 'i', 'r' };

  char_T c5_q_u[4];
  const mxArray *c5_q_y = NULL;
  int32_T c5_i7;
  static char_T c5_cv5[7] = { 'r', 'e', 'v', 'e', 'r', 's', 'e' };

  char_T c5_r_u[7];
  const mxArray *c5_r_y = NULL;
  _SFD_SYMBOL_SCOPE_PUSH(0U, 0U);
  _sfTime_ = sf_get_time(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 4U, chartInstance->c5_sfEvent);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c5_x_wp, 0U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c5_y_wp, 1U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c5_z_wp, 2U);
  for (c5_i0 = 0; c5_i0 < 12; c5_i0++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c5_X)[c5_i0], 3U);
  }

  chartInstance->c5_sfEvent = CALL_EVENT;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 4U, chartInstance->c5_sfEvent);
  c5_hoistedGlobal = *chartInstance->c5_x_wp;
  c5_b_hoistedGlobal = *chartInstance->c5_y_wp;
  c5_c_hoistedGlobal = *chartInstance->c5_z_wp;
  c5_b_x_wp = c5_hoistedGlobal;
  c5_b_y_wp = c5_b_hoistedGlobal;
  c5_b_z_wp = c5_c_hoistedGlobal;
  for (c5_i1 = 0; c5_i1 < 12; c5_i1++) {
    c5_b_X[c5_i1] = (*chartInstance->c5_X)[c5_i1];
  }

  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 6U, 6U, c5_debug_family_names,
    c5_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_nargin, 0U, c5_b_sf_marshallOut,
    c5_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_nargout, 1U, c5_b_sf_marshallOut,
    c5_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c5_b_x_wp, 2U, c5_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c5_b_y_wp, 3U, c5_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c5_b_z_wp, 4U, c5_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c5_b_X, 5U, c5_sf_marshallOut);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 5);
  c5_u = c5_b_x_wp;
  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_create("y", &c5_u, 0, 0U, 0U, 0U, 0), false);
  c5_b_u = -c5_b_y_wp;
  c5_b_y = NULL;
  sf_mex_assign(&c5_b_y, sf_mex_create("y", &c5_b_u, 0, 0U, 0U, 0U, 0), false);
  c5_c_u = c5_b_z_wp;
  c5_c_y = NULL;
  sf_mex_assign(&c5_c_y, sf_mex_create("y", &c5_c_u, 0, 0U, 0U, 0U, 0), false);
  for (c5_i2 = 0; c5_i2 < 2; c5_i2++) {
    c5_d_u[c5_i2] = c5_cv0[c5_i2];
  }

  c5_d_y = NULL;
  sf_mex_assign(&c5_d_y, sf_mex_create("y", c5_d_u, 10, 0U, 1U, 0U, 2, 1, 2),
                false);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "plot3", 0U, 4U, 14, c5_y, 14,
                    c5_b_y, 14, c5_c_y, 14, c5_d_y);
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 6);
  c5_e_u = c5_b_x_wp;
  c5_e_y = NULL;
  sf_mex_assign(&c5_e_y, sf_mex_create("y", &c5_e_u, 0, 0U, 0U, 0U, 0), false);
  c5_f_u = -c5_b_y_wp;
  c5_f_y = NULL;
  sf_mex_assign(&c5_f_y, sf_mex_create("y", &c5_f_u, 0, 0U, 0U, 0U, 0), false);
  c5_g_u = 0.0;
  c5_g_y = NULL;
  sf_mex_assign(&c5_g_y, sf_mex_create("y", &c5_g_u, 0, 0U, 0U, 0U, 0), false);
  for (c5_i3 = 0; c5_i3 < 2; c5_i3++) {
    c5_h_u[c5_i3] = c5_cv1[c5_i3];
  }

  c5_h_y = NULL;
  sf_mex_assign(&c5_h_y, sf_mex_create("y", c5_h_u, 10, 0U, 1U, 0U, 2, 1, 2),
                false);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "plot3", 0U, 4U, 14, c5_e_y, 14,
                    c5_f_y, 14, c5_g_y, 14, c5_h_y);
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 8);
  c5_i_u = c5_b_X[0];
  c5_i_y = NULL;
  sf_mex_assign(&c5_i_y, sf_mex_create("y", &c5_i_u, 0, 0U, 0U, 0U, 0), false);
  c5_j_u = -c5_b_X[1];
  c5_j_y = NULL;
  sf_mex_assign(&c5_j_y, sf_mex_create("y", &c5_j_u, 0, 0U, 0U, 0U, 0), false);
  c5_k_u = -c5_b_X[2];
  c5_k_y = NULL;
  sf_mex_assign(&c5_k_y, sf_mex_create("y", &c5_k_u, 0, 0U, 0U, 0U, 0), false);
  for (c5_i4 = 0; c5_i4 < 2; c5_i4++) {
    c5_l_u[c5_i4] = c5_cv2[c5_i4];
  }

  c5_l_y = NULL;
  sf_mex_assign(&c5_l_y, sf_mex_create("y", c5_l_u, 10, 0U, 1U, 0U, 2, 1, 2),
                false);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "plot3", 0U, 4U, 14, c5_i_y, 14,
                    c5_j_y, 14, c5_k_y, 14, c5_l_y);
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 10);
  c5_m_u[0] = c5_b_x_wp;
  c5_m_u[1] = c5_b_X[0];
  c5_m_y = NULL;
  sf_mex_assign(&c5_m_y, sf_mex_create("y", c5_m_u, 0, 0U, 1U, 0U, 2, 1, 2),
                false);
  c5_n_u[0] = -c5_b_y_wp;
  c5_n_u[1] = -c5_b_X[1];
  c5_n_y = NULL;
  sf_mex_assign(&c5_n_y, sf_mex_create("y", c5_n_u, 0, 0U, 1U, 0U, 2, 1, 2),
                false);
  c5_o_u[0] = c5_b_z_wp;
  c5_o_u[1] = -c5_b_X[2];
  c5_o_y = NULL;
  sf_mex_assign(&c5_o_y, sf_mex_create("y", c5_o_u, 0, 0U, 1U, 0U, 2, 1, 2),
                false);
  for (c5_i5 = 0; c5_i5 < 3; c5_i5++) {
    c5_p_u[c5_i5] = c5_cv3[c5_i5];
  }

  c5_p_y = NULL;
  sf_mex_assign(&c5_p_y, sf_mex_create("y", c5_p_u, 10, 0U, 1U, 0U, 2, 1, 3),
                false);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "plot3", 0U, 4U, 14, c5_m_y, 14,
                    c5_n_y, 14, c5_o_y, 14, c5_p_y);
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 12);
  for (c5_i6 = 0; c5_i6 < 4; c5_i6++) {
    c5_q_u[c5_i6] = c5_cv4[c5_i6];
  }

  c5_q_y = NULL;
  sf_mex_assign(&c5_q_y, sf_mex_create("y", c5_q_u, 10, 0U, 1U, 0U, 2, 1, 4),
                false);
  for (c5_i7 = 0; c5_i7 < 7; c5_i7++) {
    c5_r_u[c5_i7] = c5_cv5[c5_i7];
  }

  c5_r_y = NULL;
  sf_mex_assign(&c5_r_y, sf_mex_create("y", c5_r_u, 10, 0U, 1U, 0U, 2, 1, 7),
                false);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "set", 0U, 3U, 14,
                    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "gca", 1U, 0U),
                    14, c5_q_y, 14, c5_r_y);
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, -12);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 4U, chartInstance->c5_sfEvent);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_sl_quadrotorAttitudeGuidanceMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
}

static void mdl_start_c5_sl_quadrotorAttitudeGuidance
  (SFc5_sl_quadrotorAttitudeGuidanceInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void initSimStructsc5_sl_quadrotorAttitudeGuidance
  (SFc5_sl_quadrotorAttitudeGuidanceInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void init_script_number_translation(uint32_T c5_machineNumber, uint32_T
  c5_chartNumber, uint32_T c5_instanceNumber)
{
  (void)c5_machineNumber;
  (void)c5_chartNumber;
  (void)c5_instanceNumber;
}

static const mxArray *c5_sf_marshallOut(void *chartInstanceVoid, void *c5_inData)
{
  const mxArray *c5_mxArrayOutData = NULL;
  int32_T c5_i8;
  real_T c5_b_inData[12];
  int32_T c5_i9;
  real_T c5_u[12];
  const mxArray *c5_y = NULL;
  SFc5_sl_quadrotorAttitudeGuidanceInstanceStruct *chartInstance;
  chartInstance = (SFc5_sl_quadrotorAttitudeGuidanceInstanceStruct *)
    chartInstanceVoid;
  c5_mxArrayOutData = NULL;
  for (c5_i8 = 0; c5_i8 < 12; c5_i8++) {
    c5_b_inData[c5_i8] = (*(real_T (*)[12])c5_inData)[c5_i8];
  }

  for (c5_i9 = 0; c5_i9 < 12; c5_i9++) {
    c5_u[c5_i9] = c5_b_inData[c5_i9];
  }

  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_create("y", c5_u, 0, 0U, 1U, 0U, 1, 12), false);
  sf_mex_assign(&c5_mxArrayOutData, c5_y, false);
  return c5_mxArrayOutData;
}

static const mxArray *c5_b_sf_marshallOut(void *chartInstanceVoid, void
  *c5_inData)
{
  const mxArray *c5_mxArrayOutData = NULL;
  real_T c5_u;
  const mxArray *c5_y = NULL;
  SFc5_sl_quadrotorAttitudeGuidanceInstanceStruct *chartInstance;
  chartInstance = (SFc5_sl_quadrotorAttitudeGuidanceInstanceStruct *)
    chartInstanceVoid;
  c5_mxArrayOutData = NULL;
  c5_u = *(real_T *)c5_inData;
  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_create("y", &c5_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c5_mxArrayOutData, c5_y, false);
  return c5_mxArrayOutData;
}

static real_T c5_emlrt_marshallIn
  (SFc5_sl_quadrotorAttitudeGuidanceInstanceStruct *chartInstance, const mxArray
   *c5_u, const emlrtMsgIdentifier *c5_parentId)
{
  real_T c5_y;
  real_T c5_d0;
  (void)chartInstance;
  sf_mex_import(c5_parentId, sf_mex_dup(c5_u), &c5_d0, 1, 0, 0U, 0, 0U, 0);
  c5_y = c5_d0;
  sf_mex_destroy(&c5_u);
  return c5_y;
}

static void c5_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData)
{
  const mxArray *c5_nargout;
  const char_T *c5_identifier;
  emlrtMsgIdentifier c5_thisId;
  real_T c5_y;
  SFc5_sl_quadrotorAttitudeGuidanceInstanceStruct *chartInstance;
  chartInstance = (SFc5_sl_quadrotorAttitudeGuidanceInstanceStruct *)
    chartInstanceVoid;
  c5_nargout = sf_mex_dup(c5_mxArrayInData);
  c5_identifier = c5_varName;
  c5_thisId.fIdentifier = c5_identifier;
  c5_thisId.fParent = NULL;
  c5_y = c5_emlrt_marshallIn(chartInstance, sf_mex_dup(c5_nargout), &c5_thisId);
  sf_mex_destroy(&c5_nargout);
  *(real_T *)c5_outData = c5_y;
  sf_mex_destroy(&c5_mxArrayInData);
}

const mxArray
  *sf_c5_sl_quadrotorAttitudeGuidance_get_eml_resolved_functions_info(void)
{
  const mxArray *c5_nameCaptureInfo = NULL;
  c5_nameCaptureInfo = NULL;
  sf_mex_assign(&c5_nameCaptureInfo, sf_mex_createstruct("structure", 2, 3, 1),
                false);
  c5_info_helper(&c5_nameCaptureInfo);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c5_nameCaptureInfo);
  return c5_nameCaptureInfo;
}

static void c5_info_helper(const mxArray **c5_info)
{
  const mxArray *c5_rhs0 = NULL;
  const mxArray *c5_lhs0 = NULL;
  const mxArray *c5_rhs1 = NULL;
  const mxArray *c5_lhs1 = NULL;
  const mxArray *c5_rhs2 = NULL;
  const mxArray *c5_lhs2 = NULL;
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "context", "context", 0);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("plot3"), "name", "name", 0);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 0);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXMB]$matlabroot$/toolbox/matlab/graph3d/plot3"), "resolved", "resolved",
                  0);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(MAX_uint32_T), "fileTimeLo",
                  "fileTimeLo", 0);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(MAX_uint32_T), "fileTimeHi",
                  "fileTimeHi", 0);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(MAX_uint32_T), "mFileTimeLo",
                  "mFileTimeLo", 0);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(MAX_uint32_T), "mFileTimeHi",
                  "mFileTimeHi", 0);
  sf_mex_assign(&c5_rhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs0), "rhs", "rhs", 0);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs0), "lhs", "lhs", 0);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "context", "context", 1);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("gca"), "name", "name", 1);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 1);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXMB]$matlabroot$/toolbox/matlab/graphics/gca"), "resolved", "resolved", 1);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(MAX_uint32_T), "fileTimeLo",
                  "fileTimeLo", 1);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(MAX_uint32_T), "fileTimeHi",
                  "fileTimeHi", 1);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(MAX_uint32_T), "mFileTimeLo",
                  "mFileTimeLo", 1);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(MAX_uint32_T), "mFileTimeHi",
                  "mFileTimeHi", 1);
  sf_mex_assign(&c5_rhs1, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs1, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs1), "rhs", "rhs", 1);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs1), "lhs", "lhs", 1);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "context", "context", 2);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("set"), "name", "name", 2);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 2);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/scomp/set.m"), "resolved", "resolved", 2);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1378267190U), "fileTimeLo",
                  "fileTimeLo", 2);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 2);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 2);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 2);
  sf_mex_assign(&c5_rhs2, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs2, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs2), "rhs", "rhs", 2);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs2), "lhs", "lhs", 2);
  sf_mex_destroy(&c5_rhs0);
  sf_mex_destroy(&c5_lhs0);
  sf_mex_destroy(&c5_rhs1);
  sf_mex_destroy(&c5_lhs1);
  sf_mex_destroy(&c5_rhs2);
  sf_mex_destroy(&c5_lhs2);
}

static const mxArray *c5_emlrt_marshallOut(const char * c5_u)
{
  const mxArray *c5_y = NULL;
  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_create("y", c5_u, 15, 0U, 0U, 0U, 2, 1, strlen
    (c5_u)), false);
  return c5_y;
}

static const mxArray *c5_b_emlrt_marshallOut(const uint32_T c5_u)
{
  const mxArray *c5_y = NULL;
  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_create("y", &c5_u, 7, 0U, 0U, 0U, 0), false);
  return c5_y;
}

static const mxArray *c5_c_sf_marshallOut(void *chartInstanceVoid, void
  *c5_inData)
{
  const mxArray *c5_mxArrayOutData = NULL;
  int32_T c5_u;
  const mxArray *c5_y = NULL;
  SFc5_sl_quadrotorAttitudeGuidanceInstanceStruct *chartInstance;
  chartInstance = (SFc5_sl_quadrotorAttitudeGuidanceInstanceStruct *)
    chartInstanceVoid;
  c5_mxArrayOutData = NULL;
  c5_u = *(int32_T *)c5_inData;
  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_create("y", &c5_u, 6, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c5_mxArrayOutData, c5_y, false);
  return c5_mxArrayOutData;
}

static int32_T c5_b_emlrt_marshallIn
  (SFc5_sl_quadrotorAttitudeGuidanceInstanceStruct *chartInstance, const mxArray
   *c5_u, const emlrtMsgIdentifier *c5_parentId)
{
  int32_T c5_y;
  int32_T c5_i10;
  (void)chartInstance;
  sf_mex_import(c5_parentId, sf_mex_dup(c5_u), &c5_i10, 1, 6, 0U, 0, 0U, 0);
  c5_y = c5_i10;
  sf_mex_destroy(&c5_u);
  return c5_y;
}

static void c5_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData)
{
  const mxArray *c5_b_sfEvent;
  const char_T *c5_identifier;
  emlrtMsgIdentifier c5_thisId;
  int32_T c5_y;
  SFc5_sl_quadrotorAttitudeGuidanceInstanceStruct *chartInstance;
  chartInstance = (SFc5_sl_quadrotorAttitudeGuidanceInstanceStruct *)
    chartInstanceVoid;
  c5_b_sfEvent = sf_mex_dup(c5_mxArrayInData);
  c5_identifier = c5_varName;
  c5_thisId.fIdentifier = c5_identifier;
  c5_thisId.fParent = NULL;
  c5_y = c5_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c5_b_sfEvent),
    &c5_thisId);
  sf_mex_destroy(&c5_b_sfEvent);
  *(int32_T *)c5_outData = c5_y;
  sf_mex_destroy(&c5_mxArrayInData);
}

static uint8_T c5_c_emlrt_marshallIn
  (SFc5_sl_quadrotorAttitudeGuidanceInstanceStruct *chartInstance, const mxArray
   *c5_b_is_active_c5_sl_quadrotorAttitudeGuidance, const char_T *c5_identifier)
{
  uint8_T c5_y;
  emlrtMsgIdentifier c5_thisId;
  c5_thisId.fIdentifier = c5_identifier;
  c5_thisId.fParent = NULL;
  c5_y = c5_d_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c5_b_is_active_c5_sl_quadrotorAttitudeGuidance), &c5_thisId);
  sf_mex_destroy(&c5_b_is_active_c5_sl_quadrotorAttitudeGuidance);
  return c5_y;
}

static uint8_T c5_d_emlrt_marshallIn
  (SFc5_sl_quadrotorAttitudeGuidanceInstanceStruct *chartInstance, const mxArray
   *c5_u, const emlrtMsgIdentifier *c5_parentId)
{
  uint8_T c5_y;
  uint8_T c5_u0;
  (void)chartInstance;
  sf_mex_import(c5_parentId, sf_mex_dup(c5_u), &c5_u0, 1, 3, 0U, 0, 0U, 0);
  c5_y = c5_u0;
  sf_mex_destroy(&c5_u);
  return c5_y;
}

static void init_dsm_address_info
  (SFc5_sl_quadrotorAttitudeGuidanceInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void init_simulink_io_address
  (SFc5_sl_quadrotorAttitudeGuidanceInstanceStruct *chartInstance)
{
  chartInstance->c5_x_wp = (real_T *)ssGetInputPortSignal_wrapper
    (chartInstance->S, 0);
  chartInstance->c5_y_wp = (real_T *)ssGetInputPortSignal_wrapper
    (chartInstance->S, 1);
  chartInstance->c5_z_wp = (real_T *)ssGetInputPortSignal_wrapper
    (chartInstance->S, 2);
  chartInstance->c5_X = (real_T (*)[12])ssGetInputPortSignal_wrapper
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

void sf_c5_sl_quadrotorAttitudeGuidance_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(462132121U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(2035467090U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(1664712787U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(47178207U);
}

mxArray* sf_c5_sl_quadrotorAttitudeGuidance_get_post_codegen_info(void);
mxArray *sf_c5_sl_quadrotorAttitudeGuidance_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals", "postCodegenInfo" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1, 1, sizeof
    (autoinheritanceFields)/sizeof(autoinheritanceFields[0]),
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("qgiDKtzcHr76cdGAHTS3TF");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,4,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
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
      pr[1] = (double)(1);
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
      pr[1] = (double)(1);
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
      pr[0] = (double)(12);
      pr[1] = (double)(1);
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
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  {
    mxArray* mxPostCodegenInfo =
      sf_c5_sl_quadrotorAttitudeGuidance_get_post_codegen_info();
    mxSetField(mxAutoinheritanceInfo,0,"postCodegenInfo",mxPostCodegenInfo);
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c5_sl_quadrotorAttitudeGuidance_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

mxArray *sf_c5_sl_quadrotorAttitudeGuidance_jit_fallback_info(void)
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

mxArray *sf_c5_sl_quadrotorAttitudeGuidance_updateBuildInfo_args_info(void)
{
  mxArray *mxBIArgs = mxCreateCellMatrix(1,0);
  return mxBIArgs;
}

mxArray* sf_c5_sl_quadrotorAttitudeGuidance_get_post_codegen_info(void)
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

static const mxArray *sf_get_sim_state_info_c5_sl_quadrotorAttitudeGuidance(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S'type','srcId','name','auxInfo'{{M[8],M[0],T\"is_active_c5_sl_quadrotorAttitudeGuidance\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 1, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c5_sl_quadrotorAttitudeGuidance_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc5_sl_quadrotorAttitudeGuidanceInstanceStruct *chartInstance;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
    chartInstance = (SFc5_sl_quadrotorAttitudeGuidanceInstanceStruct *)
      chartInfo->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _sl_quadrotorAttitudeGuidanceMachineNumber_,
           5,
           1,
           1,
           0,
           4,
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
          (_sl_quadrotorAttitudeGuidanceMachineNumber_,
           chartInstance->chartNumber,chartInstance->instanceNumber);
        if (chartAlreadyPresent==0) {
          /* this is the first instance */
          sf_debug_set_chart_disable_implicit_casting
            (sfGlobalDebugInstanceStruct,
             _sl_quadrotorAttitudeGuidanceMachineNumber_,
             chartInstance->chartNumber,1);
          sf_debug_set_chart_event_thresholds(sfGlobalDebugInstanceStruct,
            _sl_quadrotorAttitudeGuidanceMachineNumber_,
            chartInstance->chartNumber,
            0,
            0,
            0);
          _SFD_SET_DATA_PROPS(0,1,1,0,"x_wp");
          _SFD_SET_DATA_PROPS(1,1,1,0,"y_wp");
          _SFD_SET_DATA_PROPS(2,1,1,0,"z_wp");
          _SFD_SET_DATA_PROPS(3,1,1,0,"X");
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
        _SFD_CV_INIT_EML(0,1,1,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,325);
        _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c5_b_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c5_b_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c5_b_sf_marshallOut,(MexInFcnForType)NULL);

        {
          unsigned int dimVector[1];
          dimVector[0]= 12;
          _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c5_sf_marshallOut,(MexInFcnForType)NULL);
        }

        _SFD_SET_DATA_VALUE_PTR(0U, chartInstance->c5_x_wp);
        _SFD_SET_DATA_VALUE_PTR(1U, chartInstance->c5_y_wp);
        _SFD_SET_DATA_VALUE_PTR(2U, chartInstance->c5_z_wp);
        _SFD_SET_DATA_VALUE_PTR(3U, *chartInstance->c5_X);
      }
    } else {
      sf_debug_reset_current_state_configuration(sfGlobalDebugInstanceStruct,
        _sl_quadrotorAttitudeGuidanceMachineNumber_,chartInstance->chartNumber,
        chartInstance->instanceNumber);
    }
  }
}

static const char* sf_get_instance_specialization(void)
{
  return "LXhmdIMdEY7E9x8eNMf2tF";
}

static void sf_opaque_initialize_c5_sl_quadrotorAttitudeGuidance(void
  *chartInstanceVar)
{
  chart_debug_initialization(((SFc5_sl_quadrotorAttitudeGuidanceInstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c5_sl_quadrotorAttitudeGuidance
    ((SFc5_sl_quadrotorAttitudeGuidanceInstanceStruct*) chartInstanceVar);
  initialize_c5_sl_quadrotorAttitudeGuidance
    ((SFc5_sl_quadrotorAttitudeGuidanceInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_enable_c5_sl_quadrotorAttitudeGuidance(void
  *chartInstanceVar)
{
  enable_c5_sl_quadrotorAttitudeGuidance
    ((SFc5_sl_quadrotorAttitudeGuidanceInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_disable_c5_sl_quadrotorAttitudeGuidance(void
  *chartInstanceVar)
{
  disable_c5_sl_quadrotorAttitudeGuidance
    ((SFc5_sl_quadrotorAttitudeGuidanceInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_gateway_c5_sl_quadrotorAttitudeGuidance(void
  *chartInstanceVar)
{
  sf_gateway_c5_sl_quadrotorAttitudeGuidance
    ((SFc5_sl_quadrotorAttitudeGuidanceInstanceStruct*) chartInstanceVar);
}

static const mxArray* sf_opaque_get_sim_state_c5_sl_quadrotorAttitudeGuidance
  (SimStruct* S)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  return get_sim_state_c5_sl_quadrotorAttitudeGuidance
    ((SFc5_sl_quadrotorAttitudeGuidanceInstanceStruct*)chartInfo->chartInstance);/* raw sim ctx */
}

static void sf_opaque_set_sim_state_c5_sl_quadrotorAttitudeGuidance(SimStruct* S,
  const mxArray *st)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  set_sim_state_c5_sl_quadrotorAttitudeGuidance
    ((SFc5_sl_quadrotorAttitudeGuidanceInstanceStruct*)chartInfo->chartInstance,
     st);
}

static void sf_opaque_terminate_c5_sl_quadrotorAttitudeGuidance(void
  *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc5_sl_quadrotorAttitudeGuidanceInstanceStruct*)
                    chartInstanceVar)->S;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_sl_quadrotorAttitudeGuidance_optimization_info();
    }

    finalize_c5_sl_quadrotorAttitudeGuidance
      ((SFc5_sl_quadrotorAttitudeGuidanceInstanceStruct*) chartInstanceVar);
    utFree(chartInstanceVar);
    if (crtInfo != NULL) {
      utFree(crtInfo);
    }

    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc5_sl_quadrotorAttitudeGuidance
    ((SFc5_sl_quadrotorAttitudeGuidanceInstanceStruct*) chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c5_sl_quadrotorAttitudeGuidance(SimStruct *S)
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
    initialize_params_c5_sl_quadrotorAttitudeGuidance
      ((SFc5_sl_quadrotorAttitudeGuidanceInstanceStruct*)
       (chartInfo->chartInstance));
  }
}

static void mdlSetWorkWidths_c5_sl_quadrotorAttitudeGuidance(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_sl_quadrotorAttitudeGuidance_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(sf_get_instance_specialization(),infoStruct,5);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(sf_get_instance_specialization(),
                infoStruct,5,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop
      (sf_get_instance_specialization(),infoStruct,5,
       "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(sf_get_instance_specialization(),infoStruct,5);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 3, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,5,4);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=0; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 4; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,5);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(2941514325U));
  ssSetChecksum1(S,(4265137290U));
  ssSetChecksum2(S,(458138646U));
  ssSetChecksum3(S,(3774519083U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c5_sl_quadrotorAttitudeGuidance(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c5_sl_quadrotorAttitudeGuidance(SimStruct *S)
{
  SFc5_sl_quadrotorAttitudeGuidanceInstanceStruct *chartInstance;
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)utMalloc(sizeof
    (ChartRunTimeInfo));
  chartInstance = (SFc5_sl_quadrotorAttitudeGuidanceInstanceStruct *)utMalloc
    (sizeof(SFc5_sl_quadrotorAttitudeGuidanceInstanceStruct));
  memset(chartInstance, 0, sizeof
         (SFc5_sl_quadrotorAttitudeGuidanceInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c5_sl_quadrotorAttitudeGuidance;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c5_sl_quadrotorAttitudeGuidance;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c5_sl_quadrotorAttitudeGuidance;
  chartInstance->chartInfo.enableChart =
    sf_opaque_enable_c5_sl_quadrotorAttitudeGuidance;
  chartInstance->chartInfo.disableChart =
    sf_opaque_disable_c5_sl_quadrotorAttitudeGuidance;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c5_sl_quadrotorAttitudeGuidance;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c5_sl_quadrotorAttitudeGuidance;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c5_sl_quadrotorAttitudeGuidance;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c5_sl_quadrotorAttitudeGuidance;
  chartInstance->chartInfo.mdlStart = mdlStart_c5_sl_quadrotorAttitudeGuidance;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c5_sl_quadrotorAttitudeGuidance;
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

void c5_sl_quadrotorAttitudeGuidance_method_dispatcher(SimStruct *S, int_T
  method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c5_sl_quadrotorAttitudeGuidance(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c5_sl_quadrotorAttitudeGuidance(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c5_sl_quadrotorAttitudeGuidance(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c5_sl_quadrotorAttitudeGuidance_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
