#include "main.h"
//#include <librealsense2/rs.hpp>

//Global Variable
Mat g_src_img = Mat::zeros(IMAGE_Y, IMAGE_X, CV_16UC3);
Mat g_dep_img = Mat::zeros(IMAGE_Y, IMAGE_X, CV_16UC1);//存储的是x值


//存储实际坐标
Mat g_x_img=Mat::zeros(IMAGE_Y,IMAGE_X,CV_32FC1);
Mat g_y_img=Mat::zeros(IMAGE_Y,IMAGE_X,CV_32FC1);
Mat g_z_img=Mat::zeros(IMAGE_Y,IMAGE_X,CV_32FC1);

//存储到地面的距离
Mat g_dist_image=Mat::zeros(IMAGE_Y,IMAGE_X,CV_32FC1);

//ROI_image坐标数据用于拟合平面
Mat imageROI[3];

//鼠标句柄
bool drawingBox=false;
Rect box(-1,-1,0,0);
Mat plane=Mat::zeros(4,1,CV_32FC1);
void on_mouse(int event,int x,int y,int flags,void *ustc)
{
    Mat image = *(cv::Mat*) ustc;
    Mat temp;


    if(event==CV_EVENT_RBUTTONDOWN)
    {
        cout<<x<<' '<<y<<endl;
        cout<<"X "<<g_x_img.at<float>(y,x)<<endl;
        cout<<"Y "<<g_y_img.at<float>(y,x)<<endl;
        cout<<"Z "<<g_z_img.at<float>(y,x)<<endl;
        cout<<"dist "<<g_dist_image.at<float>(y,x)<<endl;
    }
    else if(event==CV_EVENT_LBUTTONDOWN)
    {
        drawingBox=true;
        box = Rect( x, y, 0, 0 );
    }
    else if(event==CV_EVENT_MOUSEMOVE)//左键按下   鼠标移动 在图像上画矩形
    {
        if(drawingBox)
        {
            box.width=x-box.x;
            box.height=y-box.y;
        }


    }
    else if(event == CV_EVENT_LBUTTONUP)
    {
        drawingBox = false;
        if(box.width!=0&&box.height!=0)
        {
            g_src_img.copyTo(temp);
            if( box.width<0 )
            {
                box.x+=box.width;
                box.width *=-1;
            }
            if( box.height<0 )
            {
                box.y+=box.height;
                box.height*=-1;
            }

            imageROI[0]=g_x_img(box);
            imageROI[1]=g_y_img(box);
            imageROI[2]=g_z_img(box);


            for(int i=0;i<3;i++)
            {
                ofstream output(to_string(i)+".txt");//没有此文件则重新创建，在项目中
                float coordinate;
                for (int r = 0; r < imageROI[i].rows; r++)
                {
                    for (int c = 0; c < imageROI[i].cols; c++)
                    {
                        coordinate = (float)imageROI[i].at<float>(r, c);
                        output <<coordinate <<"  ";
                    }
                    output <<endl;
                }
                output.close();
            }

            //拟合平面
            Mat fitPoints(imageROI[0].rows*imageROI[0].cols,3,CV_32FC1);
            dataAdjustToMat(imageROI[0],imageROI[1],imageROI[2],fitPoints);
            fitPlane(fitPoints,plane);

            //计算距离()
            calDistanceToGround(g_dist_image);

            // 最小二乘拟合平面，平面方程：Ax+By+Cz=D
            cout<<" A: "<<plane.at<float>(0,0)<< " B: "<< plane.at<float>(1,0)<<" C:  "<<plane.at<float>(2,0)<<" D: "<<plane.at<float>(3,0)<<endl;

            rectangle(temp,Point(box.x,box.y),Point(box.x+box.width,box.y+box.height),Scalar(0,255,0),1,8,0);

            imshow("img",temp);
        }


    }
    else if(event==CV_EVENT_MBUTTONDOWN)
    {
        cout<<"X "<<g_x_img.at<float>(y,x)<<endl;
        cout<<"Y "<<g_y_img.at<float>(y,x)<<endl;
        cout<<"Z "<<g_z_img.at<float>(y,x)<<endl;
        cout<<"AX+BY+CZ-D:  "<<plane.at<float>(0,0)*g_x_img.at<float>(y,x)+plane.at<float>(1,0)*g_y_img.at<float>(y,x)+plane.at<float>(2,0)*g_z_img.at<float>(y,x)-plane.at<float>(3,0)<<endl;
        //        cout<<"raw_depth"<< <<endl;
    }
}


/*inline float getXCameraCoordinate(float z,float x)
{
#ifdef Mirror_Symmetry
    return z*TanX/(IMAGE_X/2)*((IMAGE_X/2)-x);
#else
    return z*TanX/(IMAGE_X/2)*((IMAGE_X/2)-x);
#endif
}
inline float getYCameraCoordinate(float z,int y)
{
#ifdef Mirror_Symmetry
    return z*TanY/(IMAGE_Y/2)*((IMAGE_Y/2)-y);
#else
    return z*TanX/(IMAGE_Y/2)*((IMAGE_Y/2)-y);
#endif
}*/
void getMapData()
{
    double start = static_cast<double>(cvGetTickCount());

    //将深度信息存储到图片里
    //cvtColor(g_dep_img,g_y_img,CV_32F);
    g_dep_img.convertTo(g_z_img,CV_32F);

    for(int y=0;y<IMAGE_Y;y++)
    {

        float *xData=g_x_img.ptr<float>(y);
        float *yData=g_y_img.ptr<float>(y);
        float *zData=g_z_img.ptr<float>(y);

        for(int x=0;x<IMAGE_X;x++)
        {
            xData[x]=zData[x]*TanX/(IMAGE_X/2)*((IMAGE_X/2)-x);
            yData[x]=zData[x]*TanY/(IMAGE_Y/2)*((IMAGE_Y/2)-y);

            //xData[x]=getXCameraCoordinate(zData[x],x);
            //yData[x]=getYCameraCoordinate(zData[x],y);
        }

    }


    double time = ((double)cvGetTickCount() - start) / cvGetTickFrequency();
    //cout << "所用时间为:" << time/1000 << "ms" << endl;

}

const int cdepthwidth = 320;
const int cdepthheight = 240;
const int ccolorwidth = 320;
const int ccolorheight = 240;

const int frame_num = 30;

int main(int argc, char **argv)
{


    // Initialize OpenNI Environment
    OpenNI::initialize();
    // 声明并打开Device设备，我用的是Kinect。
    Device devAnyDevice;
    devAnyDevice.open(ANY_DEVICE );
    // 创建深度数据流
    VideoStream streamDepth;
    streamDepth.create( devAnyDevice, SENSOR_DEPTH );
    // 创建彩色图像数据流
    VideoStream streamColor;
    streamColor.create( devAnyDevice, SENSOR_COLOR );
    // 设置深度图像视频模式
    VideoMode mModeDepth;
    // 分辨率大小
    mModeDepth.setResolution( cdepthwidth, cdepthheight );
    // 每秒30帧
    mModeDepth.setFps( frame_num );
    // 像素格式
    mModeDepth.setPixelFormat( PIXEL_FORMAT_DEPTH_1_MM );

    streamDepth.setVideoMode( mModeDepth);

    // 同样的设置彩色图像视频模式
    VideoMode mModeColor;
    mModeColor.setResolution( ccolorwidth, ccolorheight );
    mModeColor.setFps( frame_num );
    mModeColor.setPixelFormat( PIXEL_FORMAT_RGB888 );


    streamColor.setVideoMode( mModeColor);
    auto camSetting =streamColor.getCameraSettings();
    camSetting->setAutoWhiteBalanceEnabled(true);
    camSetting->setAutoExposureEnabled(true);
    //camSetting->setExposure(70)!=openni::STATUS_OK;
    // 图像模式注册
    if( devAnyDevice.isImageRegistrationModeSupported(
                IMAGE_REGISTRATION_DEPTH_TO_COLOR ) )
    {
        devAnyDevice.setImageRegistrationMode( IMAGE_REGISTRATION_DEPTH_TO_COLOR );
    }

    // 打开深度和图像数据流
    streamDepth.start();
    streamColor.start();

    // 循环读取数据流信息并保存在VideoFrameRef中
    VideoFrameRef  frameDepth;
    VideoFrameRef  frameColor;


    Mat cImageBGR;
    Mat mScaledDepth;

    // 获得最大深度值
    int iMaxDepth = streamDepth.getMaxPixelValue();
//    rs2::pipeline pipe;

    //Create a configuration for configuring the pipeline with a non default profile
//    rs2::config cfg;

    //Add desired streams to configuration
//    cfg.enable_stream(RS2_STREAM_INFRARED, IMAGE_X, IMAGE_Y, RS2_FORMAT_Y8, 30);
//    cfg.enable_stream(RS2_STREAM_DEPTH, IMAGE_X, IMAGE_Y, RS2_FORMAT_Z16, 30);
//    cfg.enable_stream(RS2_STREAM_COLOR, IMAGE_X, IMAGE_Y, RS2_FORMAT_BGR8, 30);

//    pipe.start(cfg);

    namedWindow( "Depth Image" );
    namedWindow( "Color Image" );

    setMouseCallback("Color Image", on_mouse,(void *)&g_dep_img);
    //创建窗口


    char key ;
    // int count = 1;

    while(1)
    {
        streamDepth.readFrame( &frameDepth );
        streamColor.readFrame( &frameColor );

//        rs2::frameset frames;
//        for(int i = 0; i < 3; i++)
//        {
//            //Wait for all configured streams to produce a frame
//            frames = pipe.wait_for_frames();
//        }
//        rs2::frame depth_frame = frames.get_depth_frame();
//        rs2::frame color_frame = frames.get_color_frame();

        // 将深度数据转换成OpenCV格式
        const Mat mImageDepth( frameDepth.getHeight(), frameDepth.getWidth(), CV_16UC1, (void*)frameDepth.getData());
//        const Mat mImageDepth(Size(IMAGE_X, IMAGE_Y), CV_16UC1, (void*)depth_frame.get_data(), Mat::AUTO_STEP);

        g_dep_img = mImageDepth.clone();

        // 为了让深度图像显示的更加明显一些，将CV_16UC1 ==> CV_8U格式
        mImageDepth.convertTo( mScaledDepth, CV_8U, 255.0 / iMaxDepth );
        // 显示出深度图像
        imshow( "Depth Image", mScaledDepth );
        imshow( "Depth Image", mImageDepth );

        // 同样的将彩色图像数据转化成OpenCV格式
        const Mat mImageRGB(frameColor.getHeight(), frameColor.getWidth(), CV_8UC3, (void*)frameColor.getData());
        // 首先将RGB格式转换为BGR格式
        cvtColor( mImageRGB, cImageBGR, COLOR_RGB2BGR );
//        const Mat cImageBGR(Size(IMAGE_X, IMAGE_Y), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);

        g_src_img = cImageBGR.clone();

        getMapData();

        // 然后显示彩色图像
        imshow( "Color Image", cImageBGR );

        char c = waitKey(50);
        if (c == 'q') break;
    }
    streamDepth.stop();
    streamColor.stop();



}
