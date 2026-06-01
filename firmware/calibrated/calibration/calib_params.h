/*
 * Real-world IMU Auto-Generated Calibration Parameters (calib_params.h)
 * 생성일: 2026-06-01
 * 본 헤더파일을 복사하여 firmware/calibration/calib_params.h 경로에 이식하십시오.
 */

#ifndef _CALIB_PARAMS_H_
#define _CALIB_PARAMS_H_

// 1. 가속도계 12-Parameter 보정용 대수 정합 변수
const float ACC_W[3][3] = {
  {  -0.00100869f,   0.00000240f,   0.00000351f },
  {   0.00000399f,  -0.00099181f,   0.00001127f },
  {  -0.00000032f,  -0.00000506f,  -0.00099961f }
};

const float ACC_B[3] = {
   14.21928475f,  14.35343564f,  11.03892283f
};

// 2. 자력계 9-Parameter 타원체 피팅(Soft/Hard Iron) 보정용 대수 정합 변수
const float MAG_W[3][3] = {
  {   0.00014250f,  -0.00003038f,  -0.00002869f },
  {  -0.00003038f,   0.00010793f,  -0.00001983f },
  {  -0.00002869f,  -0.00001983f,   0.00011612f }
};

const float MAG_B[3] = {
  1423.91414046f, 355.74115862f, -1661.91979602f
};

#endif // _CALIB_PARAMS_H_
