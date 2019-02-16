#ifndef _PLANEFIT_H__
#define _PLANEFIT_H__
#include "main.h"
void fitPlane(const cv::Mat &points, cv::Mat& plane);
void dataAdjustToMat( cv:: Mat & x, cv:: Mat &y, cv:: Mat &z,cv:: Mat &result );
void calDistanceToGround(cv:: Mat & distImage);
#endif