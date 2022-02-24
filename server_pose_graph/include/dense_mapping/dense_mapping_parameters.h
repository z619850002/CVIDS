#pragma once

#include <cstring>

#define DOWNSAMPLE 0
#define BENCHMARK 0
#define OFFLINE 0

extern int nStereoNumber;


#if DOWNSAMPLE
const int WIDTH = 640 / 2;
const int ALIGN_WIDTH = 384;
const int HEIGHT = 480 / 2;
const float FOCAL = (466.42 + 466.07) / 4;
const int DEP_CNT = 128 / 2;
#else
const int WIDTH = 640;
const int ALIGN_WIDTH = 640;
const int HEIGHT = 480;
// const float FOCAL = (466.42 + 466.07) / 2;
// const float FOCAL =  481.2f/640.0*752.0;
const float FOCAL = (461.6 + 460.3)/2;
const int DEP_CNT = 128;
#endif



const int MARGIN = 2;
const int PATCH_SIZE = (2 * MARGIN + 1) * (2 * MARGIN + 1);

// BASE_LINE * FOCAL = DEP_CNT * NEAR_DEP
const float BASE_LINE = 0.11;
const float DEP_SAMPLE = 1.0f / (BASE_LINE * FOCAL);

const int BLOCK_SIZE_X = 16;
const int BLOCK_SIZE_Y = 8;

extern float pi1;
extern float pi2;
extern float tau_so;
extern float sgm_q1;
extern float sgm_q2;
extern int sgm_iter;
extern float var_scale;
extern int var_width;

const float DEP_INF = 1000.0f;
const float PIXEL_INF = 1000.0f;


extern float nSparseRatio;
