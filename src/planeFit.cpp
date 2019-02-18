#include "planeFit.h"

/*
 * 
 * 
 *  最小二乘拟合平面，平面方程：Ax+By+Cz=D
 *  A = plane.at<float>(0,0)
 *  B = plane.at<float>(1,0)
 *  C = plane.at<float>(2,0)
 *  D = plane.at<float>(3,0)
 * 
 * */
void fitPlane(const cv::Mat &points, cv::Mat& plane){
    
    double start = static_cast<double>(cvGetTickCount());
    int rows = points.rows;
    int cols = points.cols;

    cv::Mat centroid = cv::Mat::zeros(1,cols,CV_32FC1);
    for(int i=0;i<cols;i++)
    {
        for(int j=0;j<rows;j++)
        {
            centroid.at<float>(0,i) += points.at<float>(j,i);
        }
        centroid.at<float>(0,i)/=rows;
        cout<<"第"<<i<<"列"<<centroid.at<float>(0,i)<<"   ";
    }
    cout<<endl;

    cv::Mat points2 = cv::Mat::ones(rows,cols,CV_32FC1);
    for(int i=0;i<rows;i++){
        for(int j=0;j<cols;j++){
            points2.at<float>(i,j) = points.at<float>(i,j) - centroid.at<float>(0,j) ;
        }
    }
    cv::Mat A,W,U,V;
    cv::gemm(points2,points,1,NULL,0,A,CV_GEMM_A_T);
    SVD::compute(A,W,U,V);

    Mat tmpplane = cv::Mat::zeros(4,1,CV_32FC1);
	 
    for (int c = 0; c<cols; c++){
        tmpplane.at<float>(c,0) = V.at<float>(cols-1,c);
        tmpplane.at<float>(cols,0) += tmpplane.at<float>(c,0)*centroid.at<float>(0,c);
		
    }
	tmpplane.copyTo(plane);
	
	double time = ((double)cvGetTickCount() - start) / cvGetTickFrequency();
	cout << "拟合所用时间为:" << time/1000 << "ms" << endl;

}


//将距离数据转到1个Mat中  行 数据数  列3
void dataAdjustToMat( Mat & Ximg, Mat &Yimg, Mat &Zimg,Mat &result )
{
	double start = static_cast<double>(cvGetTickCount());
	int pointNum=0;
	int rows=Ximg.rows;
	int cols=Ximg.cols;
	for(int y=0;y<rows;y++)
	{
		float *xData=Ximg.ptr<float>(y);
		float *yData=Yimg.ptr<float>(y);
		float *zData=Zimg.ptr<float>(y);
		
		for(int x=0;x<cols;x++)
		{
			result.at<float>(y*cols+x,0)=xData[x];
			result.at<float>(y*cols+x,1)=yData[x];
			result.at<float>(y*cols+x,2)=zData[x];
		}
	}
	double time = ((double)cvGetTickCount() - start) / cvGetTickFrequency();
	cout << "调整格式所用时间为:" << time/1000 << "ms" << endl;
}
/*
 * AX+BY+CZ-D=0
 * d=|Ax0+By0+CZ0-D|/sqrt(A*A+B*B+C*C);
 * 	=|Ax0+By0+CZ0-D|/n
 * 其中 n平方为1
 * */
void calDistanceToGround(Mat & distImage)
{
	double start = static_cast<double>(cvGetTickCount());
	float A=plane.at<float>(0,0);
	float B=plane.at<float>(1,0);
	float C=plane.at<float>(2,0);
	float D=plane.at<float>(3,0);
	//float sumCofSquare=plane.at<float>(0,0)*plane.at<float>(0,0)+plane.at<float>(1,0)*plane.at<float>(1,0)+plane.at<float>(2,0)*plane.at<float>(2,0);
	//float sumCofSquare=A*A+B*B+C*C;//已经约束为1了
	//cout<<sumCofSquare<<endl;
	for(int y=0;y<IMAGE_Y;y++)
	{
		float *pdsit = distImage.ptr<float>(y);
		float *xData = g_x_img.ptr<float>(y);
		float *yData = g_y_img.ptr<float>(y);
		float *zData = g_z_img.ptr<float>(y);
		for(int x=0;x<IMAGE_X;x++)
		{
			pdsit[x]=abs((A*xData[x]+B*yData[x]+C*zData[x]-D));
			//(A*xData[x]+B*yData[x]+C*zData[x]-D);
		}
	}
	double time = ((double)cvGetTickCount() - start) / cvGetTickFrequency();
	cout << "计算距离时间为:" << time/1000 << "ms" << endl;

}


void calSegDistMat(Mat &mask,float minHeight,float maxHeight)
{
	
	
}
