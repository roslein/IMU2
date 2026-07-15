/*
 * Real-world IMU Auto-Generated Calibration Parameters (calib_params.h)
 * 본 헤더파일은 툴체인을 통해 통합 생성되었습니다.
 */

#ifndef _CALIB_PARAMS_H_
#define _CALIB_PARAMS_H_

// 1. 가속도계 12-Parameter 보정용 대수 정합 변수
const float ACC_W[3][3] = {
  {   0.00100860f,  -0.00000156f,  -0.00000322f },
  {  -0.00000018f,   0.00099797f,  -0.00001686f },
  {  -0.00000492f,   0.00001066f,   0.00099817f }
};

const float ACC_B[3] = {
   13.06194854f,  12.69094028f,   9.38315482f
};

// 2. 자력계 6-param (Geometry) 보정용 대수 정합 변수
const float MAG_W[3][3] = {
  {   0.00000005f,   0.00000000f,   0.00000000f },
  {   0.00000000f,   0.00000485f,   0.00000000f },
  {   0.00000000f,   0.00000000f,   0.00000443f }
};

const float MAG_B[3] = {
  19217101.45943978f, 184.36022289f, -2689.93244376f
};

// 3. 자이로스코프 Global Bias 및 Scale Factor 보정용 대수 정합 변수
const float GYRO_W[3][3] = {
  {   1.00000000f,   0.00000000f,   0.00000000f },
  {   0.00000000f,   1.00000000f,   0.00000000f },
  {   0.00000000f,   0.00000000f,   1.00000000f }
};

const float GYRO_B[3] = {
    0.05331174f,  -0.01454020f,  -0.09081462f
};

#endif // _CALIB_PARAMS_H_
