#pragma once
#include <Kinect.h>
#include <string>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/cudaimgproc.hpp"
#include "opencv2/core/cuda.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/cudafilters.hpp"
#include "opencv2/cudaimgproc.hpp"
#include "opencv2/cudaarithm.hpp"

#include <vector>
#include <iostream>
#include <array>
namespace Kinect
{
	using namespace cv;
	using namespace std;
	class Mykinect
	{
	public:
		IKinectSensor* pSensor;
		//Color
		Mat *Color_image;
		// Depth
		Mat *mDepthImg;
		Mat *mImg8bit;
	private:
		Mat ColorFrame;
		Mat DepthFrame;
		IDepthFrameSource* pDepththFrameSource;
		IDepthFrameReader* pDepthFrameReader;
		IColorFrameSource* pColorFrameSource;
		IColorFrameReader*  pColorFrameReader;
		IInfraredFrameSource* pIIRFrameSource;
		IInfraredFrameReader* pIIRFrameReader;
	public:
		Mykinect()
		{
			if (!GetDefaultKinectSensor(&this->pSensor) == S_OK)erro_return(1);
			//Open Kinect,Depth
			pSensor->Open();
			pSensor->get_DepthFrameSource(&pDepththFrameSource);
			pDepththFrameSource->OpenReader(&pDepthFrameReader);
			//Open RGB
			pSensor->get_ColorFrameSource(&pColorFrameSource);
			pColorFrameSource->OpenReader(&pColorFrameReader);
			//Open IIR
			pSensor->get_InfraredFrameSource(&pIIRFrameSource);
			pIIRFrameSource->OpenReader(&pIIRFrameReader);
			//Get Color image Size
			Color_image = new Mat(1080, 1920, CV_8UC4);
			mDepthImg = new Mat(424, 512, CV_16UC1);
			mImg8bit = new Mat(424,512, CV_8UC1);
		}
		~Mykinect()
		{
			pSensor->Release();
			pDepththFrameSource->Release();
			pDepthFrameReader->Release();
			pColorFrameSource->Release();
			pColorFrameReader->Release();
			pIIRFrameSource->Release();
			pIIRFrameReader->Release();
		}

		Mat *Depthframe()
		{
			//Get frame description
			/*int iWidth = 0;
			int iHeight = 0;
			IFrameDescription* pFrameDescription = nullptr;
			pDepththFrameSource->get_FrameDescription(&pFrameDescription);
			pFrameDescription->get_Width(&iWidth);
			pFrameDescription->get_Height(&iHeight);
			pFrameDescription->Release();
			pFrameDescription = nullptr;*/
			//get some dpeth only meta
			UINT16 uDepthMin = 0, uDepthMax = 0;
			pDepththFrameSource->get_DepthMinReliableDistance(&uDepthMin);
			pDepththFrameSource->get_DepthMaxReliableDistance(&uDepthMax);
		
			// get frame reader
			IDepthFrameReader* pFrameReader = nullptr;
			pDepththFrameSource->OpenReader(&pFrameReader);
			IDepthFrame* pFrame = nullptr;
			if (pFrameReader->AcquireLatestFrame(&pFrame) == S_OK)
			{
				//copy the depth map to image
				pFrame->CopyFrameDataToArray(424 * 512,reinterpret_cast<UINT16*>((*mDepthImg).data));
				//convert from 16bit to 8bit
				mDepthImg->convertTo(*mImg8bit, CV_8U, 255.0f / (uDepthMax));
				//release frame
				pFrame->Release();
			}
			//horizontal flip
			flip(*mImg8bit, *mImg8bit, 1);
			return mImg8bit;
		}
		Mat *Colorframe()
		{
			int height = 0, width = 0;  
			//Mat Color_temp(1080, 1920, CV_8UC4);
			//Get Color frame size
			IFrameDescription * myDescription = nullptr;
			pColorFrameSource->get_FrameDescription(&myDescription);
			myDescription->get_Height(&height);
			myDescription->get_Width(&width);
			myDescription->Release();
			//Mat *image = new Mat(height, width,CV_8UC4);
			IColorFrame* pFrame = nullptr;
			//Get Color frame
			HRESULT ss;
			while (ss != S_OK) {
				ss = pColorFrameReader->AcquireLatestFrame(&pFrame);
				waitKey(5);
			}
			//kinect color convert to mat
			pFrame->CopyConvertedFrameDataToArray(1920 * 1080 * 4, reinterpret_cast<BYTE*>((*Color_image).data), ColorImageFormat_Bgra);
			pFrame->Release();
			/*cv::Size sss(1280,720);
			cv::resize(Color_temp, Color_temp, sss);
			cv::imshow("ss", Color_temp);
			Color_image->copyTo(Color_temp);*/
			//horizontal flip
			//flip(*Color_image,*Color_image,1);
			return Color_image;
		}
	private:
		std::string erro_return(int erro_code)
		{
			switch (erro_code)
			{
				case 1:
					perror("Kinect is can not connect");
					waitKey(10000);
					exit(-1);
				break;
				case 2:
					perror("Can not get frame size");
					waitKey(10000);
					exit(-1);
				break;
			}
			return "0";
		}
	};
	class MyJoint
	{
	private:
		Mykinect *Kinect;
		IBodyFrameReader* pIBodyFrameReader;
		IBodyFrameSource* pIBodyFrameSource;
		ICoordinateMapper* pCoordinateMapper;
		//Count of People 
		INT32 Count;
		//Joint Data
		IBody** aBody;
		//Joint system Timer
		UINT64 Joint_Timer;
	public:
		double *Joint_info;
		MyJoint(Mykinect *InputKinect)
		{
			Kinect = InputKinect;
			Count = 0;
			Joint_info = new double[4];
			for (int i = 0; i < 4;i++)Joint_info[i] = 0;
			//To Calculation Joint speed and Displacement, Acceleration
			this->Joint_Timer_clear();
			//Init kinect Body obkect
			//HRESULT ss;
			pIBodyFrameSource = nullptr; 
			pIBodyFrameReader = nullptr;
			//Get Body Sensor Source
			(Kinect->pSensor)->get_BodyFrameSource(&pIBodyFrameSource);
			pIBodyFrameSource->get_BodyCount(&Count);
			//
			pCoordinateMapper = nullptr;
			(Kinect->pSensor)->get_CoordinateMapper(&pCoordinateMapper);
			//Get Count of People
			aBody = new IBody*[Count];
			for (int i = 0; i < Count; ++i)aBody[i] = nullptr;
			//Open Body Sensor
			pIBodyFrameSource->OpenReader(&pIBodyFrameReader);
			pIBodyFrameSource->Release();
			pIBodyFrameSource = nullptr;
		}
		HRESULT Joint_Timer_inc()
		{
			Joint_Timer++;
			return S_OK;
		}
		UINT64 Get_Joint_Timer() 
		{
			return Joint_Timer;
		}
		HRESULT Joint_Timer_clear()
		{
			Joint_Timer = 0;
			return S_OK;
		}
		double *Get_Joint(JointType InputJoint)
		{
			
			IBodyFrame* pFrame=nullptr;
			//Check Body Read
			if (pIBodyFrameReader->AcquireLatestFrame(&pFrame) == S_OK)
			{
				//Refreash Body Information
				if (pFrame->GetAndRefreshBodyData(Count, aBody) == S_OK)
				{
					for (int i = 0; i < Count; i++)
					{
						IBody* pBody = aBody[i];
						BOOLEAN bTracked = false;
						if ((pBody->get_IsTracked(&bTracked) == S_OK) && bTracked)
						{
							Joint aJoints[JointType::JointType_Count];
							pBody->GetJoints(JointType::JointType_Count, aJoints);
							const Joint& rJointPos = aJoints[InputJoint];
							if (rJointPos.TrackingState != TrackingState_NotTracked)
							{
								//Convert to color map
								ColorSpacePoint Joint_info_temp;
								pCoordinateMapper->MapCameraPointToColorSpace(rJointPos.Position, &Joint_info_temp);
								Joint_info[0] = i;
								Joint_info[1] = Joint_info_temp.X;
								Joint_info[2] = Joint_info_temp.Y;
								//Joint_info[2] = rJointPos.Position.Z;
								Joint_info[3] = Joint_Timer;
							}
						}
					}

				}
				pFrame->Release();
			}
			return Joint_info;
		}
	};
	class gesture
	{
		MyJoint* pJoint;
		gesture(MyJoint* InputJoint)
		{
			this->pJoint = InputJoint;
		}
		double Joint_Speed(double* prv_Jointinfo, double* Input_Jointinfo)
		{
			double time_diff= Input_Jointinfo[2] - prv_Jointinfo[2];
			//Distance diff
			double distance=sqrt(pow((Input_Jointinfo[1] - prv_Jointinfo[1]),2)+
			pow((Input_Jointinfo[0] - prv_Jointinfo[0]), 2));
			return distance / time_diff;
		}
		double Joint_distance(double* prv_Jointinfo, double* Input_Jointinfo)
		{
			double distance = sqrt(pow((Input_Jointinfo[1] - prv_Jointinfo[1]), 2) +
				pow((Input_Jointinfo[0] - prv_Jointinfo[0]), 2));
			return distance;
		}
	};
	class Picture
	{
	private:
		int ID;
		string Path;
		//Class picture Include Mat object entity
		Mat image;
	public:
		cv::cuda::GpuMat *image_GPU;
		int position_x;
		int position_y;
		//Picure border
		int pic_boarder_x[2][2];
		int pic_boarder_y[2][2];
		//Picture Constructor 
		~Picture(){};
		Picture(Mat INputMat, int ID1)
		{
			this->ID = ID1;
			this->position_x = -1;
			this->position_y = -1;
			this->image = INputMat;
			this->image_GPU=new cv::cuda::GpuMat(image);
			this->Picture_Load();
		}
		Picture(Mat INputMat, int ID1, int position_x1, int position_y1)
		{
			this->ID = ID1;
			this->position_x = position_x1;
			this->position_y = position_y1;
			this->image = INputMat;
			this->image_GPU = new cv::cuda::GpuMat(image);
			this->Picture_Load();
		}
		//Create Picture information,and provide position
		Picture( string Path1, int ID1, int position_x1, int position_y1)
		{
			this->ID = ID1;
			this->Path = Path1;
			this->position_x = position_x1;
			this->position_y = position_y1;
			//File check
			FILE *f1;
			f1 = fopen((this->Path).c_str(), "r");
			if (!f1)
			{
				//File too Large
				std::cerr << E_NOTIMPL;
			}
			fclose(f1);
			//Init Picture
			Mat pic = imread(this->Path, CV_LOAD_IMAGE_COLOR);
			this->image = pic;
			this->Picture_Load();
		}
		//Load Picture
		HRESULT Picture_Load()
		{
			if (this->ID < 0)
			{
				return E_NOTIMPL;
			}
			//Calculate picture boarder
			pic_boarder_x[0][0] = this->position_x;
			pic_boarder_y[0][0] = this->position_y;

			pic_boarder_x[0][1] = this->position_x + this->image.cols;
			pic_boarder_y[0][1] = this->position_y;

			pic_boarder_x[1][0] = this->position_x;
			pic_boarder_y[1][0] = this->position_y + this->image.rows;

			pic_boarder_x[1][1] = this->position_x + this->image.cols;
			pic_boarder_y[1][1] = this->position_y + this->image.rows;

			//bondary check 
			return S_OK;
		}
		//Add Position information
		HRESULT Add_Position(int position_x1, int position_y1)
		{
			this->position_x = position_x1;
			this->position_y = position_y1;
			return S_OK;
		}
		HRESULT Change_Mat(Mat InputMat)
		{
			this->image = InputMat;
			return S_OK;
		}
		HRESULT Show_Picture(const string& window_name)
		{
			cv::imshow(window_name, image);
			return S_OK;
		}
		HRESULT Picture_resize(int x = 0,int y =0,float fx=1.5,float fy=1.5)
		{
			cv::Size desize(x,y);
			cv::resize(image, image, desize, fx,fy);
			return S_OK;
		}
		//Get Picture Positive
		int Get_Pos_X()
		{
			return position_x;
		}
		int Get_Pos_Y()
		{
			return position_y;
		}
		//Get image
		Mat Get_image()
		{
			return image;
		}
		//Get Picture boarder

		//Get ID
		int Get_ID()
		{
			return this->ID;
		}
	};
	class MyPictureBox
	{
	private:
		vector<Picture*> Picture_Box;
	public:
		MyPictureBox()
		{
			this->MyPictureBox_clear();
		}
		~MyPictureBox(){}
		vector<Picture*> *MyPictureBox_Get()
		{
			return &Picture_Box;
		}
		HRESULT MyPictureBox_clear()
		{
			this->Picture_Box.clear();
			return S_OK;
		}
		HRESULT MyPictureBox_put(Picture *Input_pic)
		{
			Picture_Box.push_back(Input_pic);
			return S_OK;
		}
		HRESULT MyPictureBox_Del(int index)
		{
			//if vector size is 1,than clear, else erase index
			if (Picture_Box.size())MyPictureBox_clear();
			else Picture_Box.erase(Picture_Box.begin()+index);
			return S_OK;
		}
		HRESULT MyPictureBox_Multiple_Put(Picture **Input_pic,int count)
		{
			for(int i=0;i<count;i++)Picture_Box.push_back(Input_pic[i]);
			return S_OK;
		}
		int MyPictureBox_get_size()
		{
			return Picture_Box.size();
		}
	};
	class view
	{
		MyPictureBox *PicBox;
		Picture* background;
	public:
		view(Picture* Inputbackground, MyPictureBox *InputPicBox)
		{
			this->background = Inputbackground;
			this->PicBox = InputPicBox;
		}
		HRESULT Image_puts()
		{
			vector <Picture*>::iterator it = (PicBox->MyPictureBox_Get())->begin();
			for (int i = 0; it != (PicBox->MyPictureBox_Get())->end(); it++, i++)
			{
				Mat back = (background->Get_image());
				//kinect's Color frame is 8UC4 , but Mat is 8UC3.Therefore convert it.
				cvtColor(back, back, CV_BGRA2BGR);
				Mat logo = (PicBox->MyPictureBox_Get())->at(i)->Get_image();
				if (!logo.data) { cerr << "Error" << endl; }
				//Mat Temp
				Mat ImageROT;
				//Mask
				Mat mask = logo;
				//Setting area
				ImageROT = back(Rect((PicBox->MyPictureBox_Get()->at(i))->position_x, (PicBox->MyPictureBox_Get()->at(i))->position_y, logo.cols, logo.rows));
				//Tweak color
				addWeighted(ImageROT, 0.5, logo, 0.5, 0, ImageROT);
				//Picture
				logo.copyTo(ImageROT, mask);
				//save return background
				background->Change_Mat(back);
			}
			return S_OK;
		}
		HRESULT Image_puts_GPU()
		{
			vector <Picture*>::iterator it = (PicBox->MyPictureBox_Get())->begin();
			for (int i = 0; it != (PicBox->MyPictureBox_Get())->end(); it++, i++)
			{
				Mat back_CPU = (background->Get_image());
				cv::cuda::GpuMat back_GPU(back_CPU);
				//kinect's Color frame is 8UC4 , but Mat is 8UC3.Therefore convert it.
				cv::cuda::cvtColor(back_GPU, back_GPU, CV_BGRA2BGR);
				Mat back(back_GPU);
				Mat logo = (PicBox->MyPictureBox_Get())->at(i)->Get_image();
				if (!logo.data) { cerr << "Error" << endl; }
				//Mat Temp
				Mat ImageROT;
				//Mask
				Mat mask = logo;
				//Setting area
				ImageROT = back(Rect((PicBox->MyPictureBox_Get()->at(i))->position_x, (PicBox->MyPictureBox_Get()->at(i))->position_y, logo.cols, logo.rows));
				//Tweak color
				addWeighted(ImageROT, 0.5, logo, 0.5, 0, ImageROT);
				//Picture
				logo.copyTo(ImageROT, mask);	
				//save return background
				background->Change_Mat(back);
			}
			return S_OK;
		}
		
		HRESULT Image_Move()
		{
			return S_OK;
		}
		HRESULT Plot_Line(int x1,int x2,int y1,int y2, const Scalar& color = (BACKGROUND_BLUE | BACKGROUND_INTENSITY), int thickness=12)
		{
			cv::Point P1(x1, y1);
			cv::Point P2(x2, y2);
			cv::line(background->Get_image(), P1, P2, color, thickness);
			return S_OK;
		}
		HRESULT Plot_Circle(int x, int y, int radius, const Scalar& color= (BACKGROUND_BLUE | BACKGROUND_INTENSITY), int thickness=10)
		{
			cv::Point P1(x, y);
			cv::circle(background->Get_image(), P1, radius, color, thickness);
			return S_OK;
		}
		HRESULT Plot_Text(int x, int y,const string& textint , double fontScale = 4,int fontFace= FONT_HERSHEY_COMPLEX ,const Scalar& color= (BACKGROUND_BLUE | BACKGROUND_INTENSITY), int thickness=10)
		{
			cv::Point P1(x, y);
			cv::putText(background->Get_image(), textint, P1,  fontFace,  fontScale,  color,  thickness);
			return S_OK;
		}
		HRESULT plot_Two_Joint(vector<cv::Point> track,int Line_weight)
		{
			vector <cv::Point>::iterator it = track.begin();
			for (int i = 0; (it + 1) != track.end(); it++, i++)
			{
				Plot_Line(track[i].x, track[i + 1].x, track[i].y, track[i + 1].y, (BACKGROUND_BLUE | BACKGROUND_INTENSITY), Line_weight);
			}
			return S_OK;
		}
		Picture* Get_background()
		{
			return background;
		}
	};
	class Trig_Item
	{
	public:
		cv::Point Trig_org;
		int Trig_reg[2];
		double Trig_Threshold;
		Trig_Item(int Trig_orgX,int Trig_orgY, int Trig_regX, int Trig_regY,double Trig_Threshold=0.015)
		{
			//Trig Origin
			this->Trig_org.x = Trig_orgX;
			this->Trig_org.y = Trig_orgY;
			//Trig Range
			this->Trig_reg[0] = Trig_regX;
			this->Trig_reg[1] = Trig_regY;
			//Trig Threshold
			this->Trig_Threshold = Trig_Threshold;
		}
	};
	class TrigBox
	{
	private:
		//Trig Count Max 20 Point at i7-3770 + 8GB RAM + GTX750TI + 1920*1080 Kinect V2 Color Frame
		array<Trig_Item*,20> Box;
	public:
		TrigBox()
		{
			for (const Trig_Item* Box_items : Box)Box_items = nullptr;
		}
		HRESULT Put_Items_Box(Trig_Item **Input_Trig,int Trig_Count)
		{
			for (int i = 0; i < size(Box);i++)this->Box[i] = Input_Trig[i];
			return S_OK;
		}
		array<Trig_Item*,20> Get_Box()
		{
			return Box;
		}
	};
	class Trig
	{
	private:
		TrigBox *Box1;
		Mykinect *Background;
		int Trig_ORGX;
		int Trig_ORGY;
		int Trig_RangeX;
		int Trig_RangeY;
		int Trig_One_Range;
		cv::Point locationl;
	public:
		Trig(TrigBox *InputBox,Mykinect *Input_Background, int Input_Trig_ORGX=0, 
			int Input_Trig_ORGY = 0,int Input_Trig_RangeX=1700, int Input_Trig_RangeY=700,int Input_Trig_One_Range=100)
		{
			this->Box1 = InputBox;
			this->Background = Input_Background;
			this->Trig_ORGX = Input_Trig_ORGX;
			this->Trig_ORGY = Input_Trig_ORGY;
			if (Input_Trig_RangeX < Input_Trig_One_Range)
			{
				cerr << "Trig Range must be large than Trig One Range"<< endl;
				exit(-1);
			}
			this->Trig_RangeX = Input_Trig_RangeX;
			this->Trig_RangeY = Input_Trig_RangeY;
			this->Trig_One_Range = Input_Trig_One_Range;
		}
		int Trig_Color_func(Mat Update_background,int orgX,int orgY,int regX,int regY)
		{
			//update webcam picture
			//Mat *Update_background = &(Backgrounds->Get_image());
			Mat imageROI1;
			Mat hsv;
			//各顏色的閥值
			Mat b; 
			//kinect V2 Convert to BGR
			cv::cvtColor(Update_background, Update_background, CV_BGRA2BGR);
			for (int i = 0; i < size(Box1->Get_Box()); i++)
			{
				imageROI1 = (Update_background)(Rect(orgX, orgY, regX, regY));
				Mat mask = Mat::zeros(imageROI1.rows, imageROI1.cols, CV_8U); //為了濾掉其他顏色
				cvtColor(imageROI1, hsv, CV_BGR2HSV);
				//Blue
				//inRange(hsv, Scalar(90, 100, 0), Scalar(130, 255, 255), b);
				inRange(hsv, Scalar(100, 43, 46), Scalar(124, 255, 255), b);
				//cv:waitKey(500);
				if ((float)sum(b)[0] / ((float)regX *(float)regY * 255.0) > 0.001)
				{
					return i;
				}
			}
			return -1;
		}
		cv::Point Trig_Color_func_GPU(Mat Update_background)
		{

			//locationl.x = 0;
			//locationl.y = 0;
			//update webcam picture
			cv::cuda::GpuMat imageROI1(Update_background);
			cv::cuda::GpuMat hsv;
			//各顏色的閥值
			Mat b;
			//kinect V2 Convert to BGR
			cv::cuda::cvtColor(imageROI1, imageROI1, CV_BGRA2BGR);
			//cv::cuda::GpuMat mask = cv::cuda::zeros(imageROI1.rows, imageROI1.cols, CV_8U); //為了濾掉其他顏色
			cv::cuda::cvtColor(imageROI1, hsv, CV_BGR2HSV);
			Mat hsv1(hsv);
			Mat hsv2 = hsv1(Rect(Trig_ORGX, Trig_ORGY, Trig_RangeX, Trig_RangeY));
			//Blue
			inRange(hsv2, Scalar(90, 100, 0), Scalar(130, 255, 255), b);
			//inRange(hsv2, Scalar(100, 43, 46), Scalar(124, 255, 255), b);
			//argmax(b)
			double minVal,maxVal = 0.0;
			cv::Point minLoc,maxLoc;
			minMaxLoc(b, &minVal,&maxVal, &minLoc,&maxLoc);
			imshow("Catch", b);
			//
			//cout << "X:" << maxLoc.x << endl;
			cout << "Y:" << maxLoc.y << endl;
		/*	if ((maxLoc.x + Trig_One_Range < Trig_RangeX) && (maxLoc.y + Trig_One_Range < Trig_RangeY))
			{
				Mat binary_roi = b(Rect(0, 0, maxLoc.x + Trig_One_Range, maxLoc.y + Trig_One_Range));
				if ((float)sum(binary_roi)[0] / ((float)Trig_One_Range *(float)Trig_One_Range * 255.0) > 0.02)
				{
					locationl.x = Trig_ORGX + maxLoc.x;
					locationl.y = Trig_ORGY + maxLoc.y;
				}
			}*/
			locationl.x = Trig_ORGX + maxLoc.x;
			locationl.y = Trig_ORGY + maxLoc.y;
			//cv:waitKey(500);
			return locationl;
		}
		void inRange_gpu(cv::Mat &src, cv::Scalar &lowerb, cv::Scalar &upperb,Mat &dst) 
		{
			const int m = 32;
			int numRows = src.rows, numCols = src.cols;
			if (numRows == 0 || numCols == 0) return;
			Mat h, s, v;
			Mat channel[3];
			
			//Split src's hsv
			cv::split(src, channel);
			//
			cv::cuda::GpuMat channel0_GPU(channel[0]);
			cv::cuda::GpuMat channel1_GPU(channel[1]);
			cv::cuda::GpuMat channel2_GPU(channel[2]);
			cv::cuda::GpuMat* channel_GPU_Temp = new cv::cuda::GpuMat[3];
			cv::cuda::GpuMat* channel_GPU_Temp2 = new cv::cuda::GpuMat[3];
			cv::cuda::GpuMat* channel_GPU_combin = new cv::cuda::GpuMat[3];
			// lower band
			cv::cuda::threshold(channel0_GPU, channel_GPU_Temp[0], lowerb[0],255, THRESH_BINARY_INV);
			cv::cuda::threshold(channel1_GPU, channel_GPU_Temp[1], lowerb[1],255, THRESH_BINARY_INV);
			cv::cuda::threshold(channel2_GPU, channel_GPU_Temp[2], lowerb[2],255, THRESH_BINARY_INV);
			// upper band
			cv::cuda::threshold(channel0_GPU, channel_GPU_Temp2[0], upperb[0], 255, THRESH_BINARY);
			cv::cuda::threshold(channel1_GPU, channel_GPU_Temp2[1], upperb[1], 255, THRESH_BINARY);
			cv::cuda::threshold(channel2_GPU, channel_GPU_Temp2[2], upperb[2], 255, THRESH_BINARY);
			//combine
			//channel_GPU_combin[0] = channel_GPU_Temp[0] + channel_GPU_Temp2[0];
		}

	};
}
