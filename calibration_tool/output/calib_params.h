/*
 * Real-world IMU Auto-Generated Calibration Parameters (calib_params.h)
 * 본 헤더파일은 툴체인을 통해 통합 생성되었습니다.
 */

#ifndef _CALIB_PARAMS_H_
#define _CALIB_PARAMS_H_

// 1. 가속도계 12-Parameter 보정용 대수 정합 변수
const float ACC_W[3][3] = {
  {  -0.00100926f,   0.00000871f,   0.00000279f },
  {   0.00000065f,  -0.00099312f,   0.00001769f },
  {  -0.00000086f,  -0.00001280f,  -0.00099754f }
};

const float ACC_B[3] = {
   11.90815577f,  16.87290551f,   6.21252449f
};

// 2. 자력계 9-Parameter 타원체 피팅(Soft/Hard Iron) 보정용 대수 정합 변수
const float MAG_W[3][3] = {
  {   0.00008733f,  -0.00000298f,  -0.00000126f },
  {  -0.00000298f,   0.00007746f,   0.00000367f },
  {  -0.00000126f,   0.00000367f,   0.00009943f }
};

const float MAG_B[3] = {
  2055.76507794f, 1404.51495888f, -1914.47248925f
};

// 3. 자이로스코프 20-Positions Global Bias 및 Scale Factor 보정용 대수 정합 변수
const float GYRO_W[3][3] = {
  {   1.00000000f,   0.00000000f,   0.00000000f },
  {   0.00000000f,   1.00000000f,   0.00000000f },
  {   0.00000000f,   0.00000000f,   1.00000000f }
};

const float GYRO_B[3] = {
   -0.02547283f,   0.11272309f,  -0.07280521f
};

#endif // _CALIB_PARAMS_H_
