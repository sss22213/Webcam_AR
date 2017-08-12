#pragma once
#include "MyKinect.h"
#include "opencv2/imgproc/imgproc.hpp"
namespace Glasses {
	using namespace Kinect;
	using namespace System;
	using namespace System::ComponentModel;
	using namespace System::Collections;
	using namespace System::Windows::Forms;
	using namespace System::Data;
	using namespace System::Drawing;
	/// <summary>
	/// Summary for MyForm
	/// </summary>
	public ref class MyForm : public System::Windows::Forms::Form
	{
	public:
		//backgroundWorker1->RunWorkerAsync();
		
	private: System::Windows::Forms::Button^  button1;
	public: System::Windows::Forms::Button^  button2;
			
		MyForm(void)
		{
			InitializeComponent();
			//
			//TODO: Add the constructor code here
			//
		}

	protected:
		/// <summary>
		/// Clean up any resources being used.
		/// </summary>
		~MyForm()
		{
			if (components)
			{
				delete components;
			}
		}

	protected:


	private: System::Windows::Forms::Timer^  timer1;
	private: System::ComponentModel::BackgroundWorker^  backgroundWorker1;
	private: System::ComponentModel::IContainer^  components;

	private:
		/// <summary>
		/// Required designer variable.
		/// </summary>


#pragma region Windows Form Designer generated code
		/// <summary>
		/// Required method for Designer support - do not modify
		/// the contents of this method with the code editor.
		/// </summary>
		void InitializeComponent(void)
		{
			this->components = (gcnew System::ComponentModel::Container());
			this->timer1 = (gcnew System::Windows::Forms::Timer(this->components));
			this->backgroundWorker1 = (gcnew System::ComponentModel::BackgroundWorker());
			this->button1 = (gcnew System::Windows::Forms::Button());
			this->button2 = (gcnew System::Windows::Forms::Button());
			this->SuspendLayout();
			// 
			// timer1
			// 
			this->timer1->Tick += gcnew System::EventHandler(this, &MyForm::timer1_Tick);
			// 
			// backgroundWorker1
			// 
			this->backgroundWorker1->DoWork += gcnew System::ComponentModel::DoWorkEventHandler(this, &MyForm::backgroundWorker1_DoWork);
			// 
			// button1
			// 
			this->button1->Location = System::Drawing::Point(12, 174);
			this->button1->Name = L"button1";
			this->button1->Size = System::Drawing::Size(142, 109);
			this->button1->TabIndex = 0;
			this->button1->Text = L"button1";
			this->button1->UseVisualStyleBackColor = true;
			this->button1->Click += gcnew System::EventHandler(this, &MyForm::button1_Click);
			// 
			// button2
			// 
			this->button2->Location = System::Drawing::Point(335, 174);
			this->button2->Name = L"button2";
			this->button2->Size = System::Drawing::Size(142, 109);
			this->button2->TabIndex = 1;
			this->button2->Text = L"button2";
			this->button2->UseVisualStyleBackColor = true;
			this->button2->Click += gcnew System::EventHandler(this, &MyForm::button2_Click);
			// 
			// MyForm
			// 
			this->AutoScaleDimensions = System::Drawing::SizeF(6, 12);
			this->AutoScaleMode = System::Windows::Forms::AutoScaleMode::Font;
			this->ClientSize = System::Drawing::Size(525, 495);
			this->Controls->Add(this->button2);
			this->Controls->Add(this->button1);
			this->Name = L"MyForm";
			this->Text = L"MyForm";
			this->Load += gcnew System::EventHandler(this, &MyForm::MyForm_Load);
			this->ResumeLayout(false);

		}
#pragma endregion
	private: System::Void MyForm_Load(System::Object^  sender, System::EventArgs^  e) {
		//OPenCV CPU ISA Optimized
		setUseOptimized(true);
		//Init System Time(10 ms)
		timer1->Interval = 300;
		timer1->Start();

#define debug 0
#if debug
		MyJoint *JS;
		Mykinect *K1;
		K1 = new Mykinect;
		JS = new MyJoint(K1);
		MyJoint *prv = new MyJoint(K1);
		//init track
		vector<cv::Point> track;
		track.clear();
		
		double *temp = new double[4];
		Picture *Color_background = new Picture(*K1->Colorframe(),0);
		Picture *P2 = new Picture("F:\\Webcam_AR\\image\\glassesss.png", 2, 0, 0);
		Picture *P3 = new Picture("F:\\Webcam_AR\\image\\12.jpg", 3, 500, 500);
		P2->Picture_resize(0, 0, 1.3, 1.3);
		MyPictureBox *Box1 = new MyPictureBox;
		Box1->MyPictureBox_put(P2);;
		//Box1->MyPictureBox_put(P3);
		//Box1->MyPictureBox_put(P1);
		view *View1 = new view(Color_background,Box1);
		P2->Add_Position((View1->Get_background()->Get_image().cols) / 2 - P2->Get_image().cols / 2, (View1->Get_background()->Get_image()).rows / 2 - P2->Get_image().rows / 2);
		/*Mat back = Color_background->Get_image();
		cv::cvtColor(back, back, CV_BGRA2BGR);
		Mat logo = P2->Get_image();
		cout << back.type() << endl<<logo.type();

		Mat ImageROT = back(Rect(0, 0, logo.cols, logo.rows));
		
		//Copy retrun Picture
		logo.copyTo(ImageROT,logo);
		addWeighted(ImageROT, 0.5, logo, 0.5, 0, ImageROT);
		imshow("ss", back);*/
		
		prv = JS;
		int k = 0;
		while (1)
		{
			Color_background->Change_Mat(*K1->Colorframe());
			View1->Image_puts();
			//cv::imshow("TEST", Color_background->Get_image());
			//'cv::imshow("TT", P1->Get_image());
			//cv::imshow("TEST2", *K1->Depthframe());
			//(JS->Update_Joint());
			temp=JS->Get_Joint(JointType::JointType_HandRight);
			cv::Point pp1(JS->Joint_info[1], JS->Joint_info[2]);
			track.push_back(pp1);
			View1->plot_Two_Joint(track);
			printf("The People: %d, X IN %4.1f,Y IN %4.1f\n", temp[0],temp[1], temp[2]);
			//View1->plot_Two_Joint(JS, JointType::JointType_HandRight);
			
			//prv = JS;
			//View1->Plot_Line(20, 20, 500, 500, BACKGROUND_BLUE, 1);
			//View1->Plot_Line(0, 50, 100, 100);
			//View1->Plot_Line(80, 100, 100, 500);
			//View1->Plot_Circle( 1920-temp[1], temp[2], 20, BACKGROUND_BLUE | BACKGROUND_INTENSITY, CV_FILLED);
			View1->Plot_Text(temp[1], temp[2], "Bohung",2);
			View1->Get_background()->Show_Picture("name");
			cv::waitKey(30);
		}
#endif
	}
	private: System::Void textBox1_TextChanged(System::Object^  sender, System::EventArgs^  e) {
	}
	private: System::Void timer1_Tick(System::Object^  sender, System::EventArgs^  e) {
		if (backgroundWorker1->IsBusy != 0)backgroundWorker1->RunWorkerAsync();
		//JS->Joint_Timer_inc();
		//button1->PerformClick();
	}
private: System::Void backgroundWorker1_DoWork(System::Object^  sender, System::ComponentModel::DoWorkEventArgs^  e) {
	
}
private: System::Void button2_Click(System::Object^  sender, System::EventArgs^  e) {
	

}
private: System::Void button1_Click(System::Object^  sender, System::EventArgs^  e) {
	
		Mykinect Sensor;
		MyJoint Joints(&Sensor);
		Picture backgrounds(*Sensor.Colorframe(), 0);
		MyPictureBox Box1;
		view View1(&backgrounds, &Box1);
		vector<cv::Point> track;
		backgrounds.Change_Mat(*Sensor.Colorframe());
		//Glasses
		Picture Glasses("F:\\Webcam_AR\\image\\glassesss.png", 0, 0, 0);
		Glasses.Add_Position((View1.Get_background()->Get_image().cols) / 2 - Glasses.Get_image().cols / 2 -100, 0);
		Box1.MyPictureBox_put(&Glasses);
		Glasses.Picture_resize(0, 0, 1.4, 1.4);
		while (1)
		{
			double *temp = Joints.Get_Joint(JointType::JointType_HandRight);
			cv::Point pp1(Joints.Joint_info[1], Joints.Joint_info[2]);
			track.push_back(pp1);
			View1.plot_Two_Joint(track,100);
			printf("The People: %d, X IN %4.1f,Y IN %4.1f\n", temp[0], temp[1], temp[2]);
			View1.Image_puts();
			cv::imshow("TT", backgrounds.Get_image());
			//Need to after the imshow
			backgrounds.Change_Mat(*Sensor.Colorframe());
			cv::waitKey(30);
		}
}
};
}
