#ifndef _MAIN_H
#define _MAIN_H

#include<sys/time.h>
//opencv
#include <opencv2/opencv.hpp>
//c
#include <time.h> 
#include <stdio.h>
#include <stdlib.h>
#include <math.h> 

//C++
#include <iostream>
#include <fstream>
#include <string.h>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <vector>
#include <stack>
#include <algorithm>
//openni
#include "OpenNI.h"


#include "planeFit.h"

using namespace std;
using namespace cv;
using namespace openni;

#define   IMAGE_X  320
#define   IMAGE_Y  240

#define TanX 0.41421          //0.413  //0.425
#define TanY 0.59       //0.599911  //0.553  0.57


extern Mat plane;//用于存储地面方程的系数
//存储实际坐标
extern Mat g_x_img;
extern Mat g_y_img;
extern Mat g_z_img;


#endif
