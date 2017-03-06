// This source code is licensed under the MIT license. Please see the License in License.txt.
// "This is preliminary software and/or hardware and APIs are preliminary and subject to change."
//

#include "stdafx.h"
//#include <Windows.h>
#include <Kinect.h>
#include <opencv2/opencv.hpp>
//#include <cstdlib>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
//#include <conio.h>
//#include "WaveFile.h"
#include "ComPtr.h"
#include <time.h>
#pragma comment(lib, "Ws2_32.lib")//UDP
#include <Kinect.Face.h>
//#include <winsock2.h>//UDP
//#include <Ws2tcpip.h>//UDP

#define _USE_MATH_DEFINES
#include <math.h>


/* FOR FACE RECOGNITION */
// Quote from Kinect for Windows SDK v2.0 Developer Preview - Samples/Native/FaceBasics-D2D, and Partial Modification
// ExtractFaceRotationInDegrees is: Copyright (c) Microsoft Corporation. All rights reserved.
inline void ExtractFaceRotationInDegrees( const Vector4* pQuaternion, int* pPitch, int* pYaw, int* pRoll )
{
	double x = pQuaternion->x;
	double y = pQuaternion->y;
	double z = pQuaternion->z;
	double w = pQuaternion->w;

	// convert face rotation quaternion to Euler angles in degrees
	*pPitch = static_cast<int>( std::atan2( 2 * ( y * z + w * x ), w * w - x * x - y * y + z * z ) / M_PI * 180.0f );
	*pYaw = static_cast<int>( std::asin( 2 * ( w * y - x * z ) ) / M_PI * 180.0f );
	*pRoll = static_cast<int>( std::atan2( 2 * ( x * y + w * z ), w * w + x * x - y * y - z * z ) / M_PI * 180.0f );
}


/* FOR UDP CONNECTION */

int wind;
int flag_soc = 0;

	bool InitSockets()
	{

		WSADATA WSAData;
		// Winsockの初期化、バージョンは最新の2.2を指定している
		WSAStartup(MAKEWORD(2, 2), &WSAData);

		return true;
	}

	SOCKET ConnectTo(const char* ip, int port, SOCKADDR_IN & sin)
	{
		SOCKET s;

		// ソケットの作成
		if ((s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) {
			std::cout << "Can't create a socket." << std::endl;
			return SOCKET_ERROR;
		}

		memset((char *)&sin, 0, sizeof(sin));
		sin.sin_family = AF_INET;
		//sin.sin_addr.s_addr = inet_addr("127.0.0.1");  //send UDP to Unity
		sin.sin_addr.s_addr = inet_addr("131.113.137.92"); //send UDP to Arduino
		sin.sin_port = htons(port);


		SOCKADDR_IN sin2;
		memset((char *)&sin2, 0, sizeof(sin2));
		sin2.sin_family = AF_INET;
		//sin2.sin_addr.s_addr = inet_addr("127.0.0.1");  //send UDP to Unity
		sin2.sin_addr.s_addr = inet_addr("131.113.137.92"); //send UDP to Arduino
		sin2.sin_port = htons(8888);
		bind(s, (struct sockaddr*)&sin2, sizeof(sin2));
		return s;
	}


	bool SendTo(char* buffer, SOCKET s, struct sockaddr_in & sin)
	{
		if (sendto(s, buffer, (int)strlen(buffer), 0,
			(SOCKADDR *)&sin, sizeof(sin)) == -1) {
			std::cout << "Can't sendto from the socket." << std::endl;
			return false;
		}
		return true;
	}

	SOCKADDR_IN g_sin;
	SOCKET s;

	int send_UDP(float wi, int id)
	{
		
		if (flag_soc == 0) {
			//SOCKADDR_IN sin;
			//SOCKET s;
			InitSockets();
			//s = ConnectTo("127.0.0.1", 7777, g_sin);  //send UDP to Unity
			s = ConnectTo("131.113.137.92", 8888, g_sin); //send UDP to Arduino
			flag_soc = 1;
			if (s == SOCKET_ERROR)
			{
				std::cout << "Failed to connect\n";
				return 0;
			}
		}

			char buff[30];
			sprintf_s(buff,"%d, %f",id,wi);// id 1:volume 2:speaking_rate 3:leanY
			SendTo(buff, s, g_sin);

		return 0;
	}


template<class Interface>
inline void SafeRelease( Interface *& pInterfaceToRelease )
{
	if( pInterfaceToRelease != NULL ){
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}


/* MAIN FUNCTION */

int _tmain( int argc, _TCHAR* argv[] )
{

	cv::setUseOptimized( true );

	/*TO NAME FILENAME WITH CURRENT DATE*/
	char time_buff[50] = "";
	time_t now = time(NULL);
	struct tm pnow;
	localtime_s(&pnow, &now);
	sprintf_s(time_buff, 50, "%04d%02d%02d_%02d%02d%02d.txt", pnow.tm_year + 1900, pnow.tm_mon + 1, pnow.tm_mday, pnow.tm_hour, pnow.tm_min, pnow.tm_sec);
	std::cout << time_buff << std::endl;


	//std::ofstream outputfile("test.txt");
	std::ofstream outputfile(time_buff);
	std::vector<float> audioBuffer;
	UINT subFrameLengthInBytes;


	// Sensor
	IKinectSensor* pSensor;
	HRESULT hResult = S_OK;
	hResult = GetDefaultKinectSensor( &pSensor );

	if( FAILED( hResult ) ){
		std::cerr << "Error : GetDefaultKinectSensor" << std::endl;
		return -1;
	}

	hResult = pSensor->Open();

	if( FAILED( hResult ) ){
		std::cerr << "Error : IKinectSensor::Open()" << std::endl;
		return -1;
	}

	// Source
	IDepthFrameSource* pDepthSource;
	hResult = pSensor->get_DepthFrameSource( &pDepthSource );

	IColorFrameSource* pColorSource;
	hResult = pSensor -> get_ColorFrameSource(&pColorSource);

	IBodyIndexFrameSource* pBodyIndexSource;
	hResult = pSensor -> get_BodyIndexFrameSource (&pBodyIndexSource);

	IBodyFrameSource* pBodySource;
	hResult = pSensor -> get_BodyFrameSource (&pBodySource);

	IAudioSource* pAudioSource;
	hResult = pSensor->get_AudioSource( &pAudioSource );

	if( FAILED( hResult ) ){
		std::cerr << "Error : IKinectSensor::get_DepthFrameSource()" << std::endl;
		return -1;
	}

	// Reader
	IDepthFrameReader* pDepthReader;
	hResult = pDepthSource->OpenReader( &pDepthReader );

	IColorFrameReader* pColorReader;
	hResult = pColorSource->OpenReader(&pColorReader);

	IBodyIndexFrameReader* pBodyIndexReader;
	hResult = pBodyIndexSource->OpenReader(&pBodyIndexReader);

	IBodyFrameReader* pBodyReader;
	hResult = pBodySource->OpenReader( &pBodyReader );

	IAudioBeamFrameReader* pAudioReader;
	hResult = pAudioSource->OpenReader( &pAudioReader );
	hResult = pAudioSource->get_SubFrameLengthInBytes( &subFrameLengthInBytes );

	if( FAILED( hResult ) ){
		std::cerr << "Error : IDepthFrameSource::OpenReader()" << std::endl;
		return -1;
	}

	// Description
	IFrameDescription* pDescription;
	IFrameDescription* pDescription_color;
	IFrameDescription* pDescription_bodyindex;
	hResult = pDepthSource->get_FrameDescription( &pDescription );
	hResult = pColorSource->get_FrameDescription(&pDescription_color);
	hResult = pBodyIndexSource->get_FrameDescription(&pDescription_bodyindex);
	if( FAILED( hResult ) ){
		std::cerr << "Error : IDepthFrameSource::get_FrameDescription()" << std::endl;
		return -1;
	}

	int width = 0;
	int height = 0;
	int volume_flag = 0;
	pDescription->get_Width( &width ); // 512
	pDescription->get_Height( &height ); // 424
	unsigned int bufferSize = width * height * sizeof( unsigned short );
	UINT count = 0;
	int facetrack_check = 0;

	int count_a = 0, count_b = 0, total_speak_count = 0;
	int total_lean_count = 0;
	float total_energy_count = 0;

	// Range ( Range of Depth is 500-8000[mm], Range of Detection is 500-4500[mm] ) 
	unsigned short min = 0;
	unsigned short max = 0;
	pDepthSource->get_DepthMinReliableDistance( &min ); // 500
	pDepthSource->get_DepthMaxReliableDistance( &max ); // 4500
	std::cout << "Range : " << min << " - " << max << std::endl;

	cv::Mat bufferMat( height, width, CV_16UC1 );
	cv::Mat depthMat( height, width, CV_8UC1 );
	//cv::namedWindow( "Depth" );

	unsigned int bufferSize_color = 1920 * 1080 * 4 * sizeof( unsigned char);
	cv::Mat bufferMat_color (1080, 1920, CV_8UC4);
	cv::Mat colorMat (540, 960, CV_8UC4);
	//cv::namedWindow("Color");

	cv::Mat bodyIndexMat(height, width, CV_8UC3);
	cv::namedWindow("BodyIndex");

	cv::Mat bodyMat (540, 960, CV_8UC4);
	//cv::namedWindow("Body");

	cv::Mat judge_img = cv::Mat::zeros(500, 500, CV_8UC3);
	cv::namedWindow("Speaker_Judge");

	cv::Mat rate_img = cv::Mat::zeros(500, 500, CV_8UC3);
	cv::namedWindow("Speaking_Rate");

	cv::Mat handcoord_img = cv::Mat::zeros(540, 960, CV_8UC3);
	cv::namedWindow("Hand Coordination");

	cv::Mat bufferMat_face( 1080, 1920, CV_8UC4 );
	cv::Mat faceMat( 540, 960 / 2, CV_8UC4 );
	cv::namedWindow( "Face" );

	std::cout << width << ", " << height << std::endl;

	// Color Table
	cv::Vec3b color[BODY_COUNT];
	color[0] = cv::Vec3b( 255,   0,   0 );//BLUE
	color[1] = cv::Vec3b(   0, 255,   0 );//GREEN
	color[2] = cv::Vec3b(   0,   0, 255 );//RED
	color[3] = cv::Vec3b( 255, 255,   0 );//CYAN
	color[4] = cv::Vec3b( 255,   0, 255 );//MAGENTA
	color[5] = cv::Vec3b(   0, 255, 255 );//YELLOW

	cv::Scalar color_opencv[BODY_COUNT];
	color_opencv[0] = cv::Scalar( 255,   0,   0 );
	color_opencv[1] = cv::Scalar(   0, 255,   0 );
	color_opencv[2] = cv::Scalar(   0,   0, 255 );
	color_opencv[3] = cv::Scalar( 255, 255,   0 );
	color_opencv[4] = cv::Scalar( 255,   0, 255 );
	color_opencv[5] = cv::Scalar(   0, 255, 255 );

	ICoordinateMapper* pCoordinateMapper;
	hResult = pSensor -> get_CoordinateMapper (&pCoordinateMapper);
	if(FAILED(hResult)){
		std::cerr << "Error: IkinectSensor::get_CoordinateMapper()" << std::endl;
		return -1;
	}

	//BUFFER FOR RECORDING
	audioBuffer.resize( subFrameLengthInBytes / sizeof( float ) );
	//audioFile.Open( "kinect_audio_a.wav" );
	//audioFile2.Open( "kinect_audio_b.wav" );
	float renormalized_energy = 0;

	/****** FOR FACE RECOGNITION ******/

	IFaceFrameSource* pFaceSource[BODY_COUNT];
	DWORD features = FaceFrameFeatures::FaceFrameFeatures_BoundingBoxInColorSpace
		| FaceFrameFeatures::FaceFrameFeatures_PointsInColorSpace
		| FaceFrameFeatures::FaceFrameFeatures_RotationOrientation
		| FaceFrameFeatures::FaceFrameFeatures_Happy
		| FaceFrameFeatures::FaceFrameFeatures_RightEyeClosed
		| FaceFrameFeatures::FaceFrameFeatures_LeftEyeClosed
		| FaceFrameFeatures::FaceFrameFeatures_MouthOpen
		| FaceFrameFeatures::FaceFrameFeatures_MouthMoved
		| FaceFrameFeatures::FaceFrameFeatures_LookingAway
		| FaceFrameFeatures::FaceFrameFeatures_Glasses
		| FaceFrameFeatures::FaceFrameFeatures_FaceEngagement;
	IFaceFrameReader* pFaceReader[BODY_COUNT];
	for( int count = 0; count < BODY_COUNT; count++ ){
		// Source
		hResult = CreateFaceFrameSource( pSensor, 0, features, &pFaceSource[count] );
		if( FAILED( hResult ) ){
			std::cerr << "Error : CreateFaceFrameSource" << std::endl;
			return -1;
		}

		// Reader
		hResult = pFaceSource[count]->OpenReader( &pFaceReader[count] );
		if( FAILED( hResult ) ){
			std::cerr << "Error : IFaceFrameSource::OpenReader()" << std::endl;
			return -1;
		}
	}

	// Face Property Table
	std::string property[FaceProperty::FaceProperty_Count];
	property[0] = "Happy";
	property[1] = "Engaged";
	property[2] = "WearingGlasses";
	property[3] = "LeftEyeClosed";
	property[4] = "RightEyeClosed";
	property[5] = "MouthOpen";
	property[6] = "MouthMoved";
	property[7] = "LookingAway";


	/* MAIN LOOP */

	while( 1 ){
		//while (GetMessage(&msg, NULL, 0, 0) > 0){
		//TranslateMessage(&msg);
		//DispatchMessage(&msg);



		// Frame
		IDepthFrame* pDepthFrame = nullptr;
		IColorFrame* pColorFrame = nullptr;
		IBodyIndexFrame* pBodyIndexFrame = nullptr;
		IBodyFrame* pBodyFrame = nullptr;
		ComPtr<IAudioBeamFrameList> pAudioFrameList;

		/****** COLOR FRAME ******/

		hResult = pColorReader->AcquireLatestFrame(&pColorFrame);

		if(SUCCEEDED(hResult)){
			hResult = pColorFrame->CopyConvertedFrameDataToArray(bufferSize_color, reinterpret_cast<BYTE*>(bufferMat_color.data), ColorImageFormat_Bgra);

			if(SUCCEEDED(hResult)){
				cv::resize(bufferMat_color, colorMat, cv::Size(), 0.5, 0.5);
				cv::resize(bufferMat_color, bodyMat, cv::Size(), 0.5, 0.5);
			}

			hResult = pColorFrame->CopyConvertedFrameDataToArray(bufferSize_color, reinterpret_cast<BYTE*>(bufferMat_face.data), ColorImageFormat_Bgra);

			if(SUCCEEDED(hResult)){
				cv::resize( bufferMat_face, faceMat, cv::Size(), 0.5, 0.5 );
			}

		}


		/****** BODY INDEX ******/

		hResult = pBodyIndexReader->AcquireLatestFrame(&pBodyIndexFrame);

		if(SUCCEEDED(hResult)){
			unsigned int bufferSize = 0;
			unsigned char* buffer = nullptr;
			hResult = pBodyIndexFrame->AccessUnderlyingBuffer(&bufferSize, &buffer);


			if(SUCCEEDED(hResult)){
				for (int y = 0; y<height; y++){
					for (int x = 0; x < width; x++){
						unsigned int index = y * width + x;
						if (buffer[index] != 0xff){
							bodyIndexMat.at<cv::Vec3b>(y, x) = color[buffer[index]];
						}
						else{
							bodyIndexMat.at<cv::Vec3b>(y, x) = cv::Vec3b(0,0,0);
						}
					}
				}
			}
		}

		/****** BODY ******/

		hResult = pBodyReader -> AcquireLatestFrame(&pBodyFrame);

		if(SUCCEEDED (hResult)){
			IBody* pBody[BODY_COUNT] = {0};
			hResult = pBodyFrame->GetAndRefreshBodyData (BODY_COUNT, pBody);
			if(SUCCEEDED(hResult)){

				//COLOR EACH FRAME BLACK
				cv::rectangle(handcoord_img, cv::Point(0,0), cv::Point(960, 540), cv::Scalar(0,0,0), -1, CV_AA);

				for (count = 0; count < BODY_COUNT; count++){
					BOOLEAN bTracked = false;
					hResult = pBody[count]->get_IsTracked (&bTracked);
					if(SUCCEEDED(hResult) && bTracked){
						Joint joint[JointType::JointType_Count];
						hResult = pBody[count]->GetJoints(JointType::JointType_Count, joint);

						/*for face recognition*/
						// Set TrackingID to Detect Face
						UINT64 trackingId = _UI64_MAX;
						hResult = pBody[count]->get_TrackingId( &trackingId );
						if( SUCCEEDED( hResult ) ){
							pFaceSource[count]->put_TrackingId( trackingId );
						}
						/*for face recognition*/


						if(SUCCEEDED(hResult)){
							time_t now2 = time(NULL);
							struct tm pnow2;
							localtime_s(&pnow2, &now2);
							sprintf_s(time_buff, 50, "%04d%02d%02d_%02d%02d%02d", pnow2.tm_year + 1900, pnow2.tm_mon + 1, pnow2.tm_mday, pnow2.tm_hour, pnow2.tm_min, pnow2.tm_sec);
							outputfile << time_buff << ",";

							//LEFT HAND!
							HandState leftHandState = HandState::HandState_Unknown;
							hResult = pBody[count]->get_HandLeftState(&leftHandState);
							if(SUCCEEDED(hResult)){
								std::cout << count << "の左手をGETしています" << std::endl;
								ColorSpacePoint colorSpacePoint = {0};
								hResult = pCoordinateMapper->MapCameraPointToColorSpace(joint[JointType::JointType_HandLeft].Position, &colorSpacePoint);
								if(SUCCEEDED(hResult)){
									int x = static_cast<int>(colorSpacePoint.X);
									int y = static_cast<int>(colorSpacePoint.Y);
									/*
									if((x >= 0)&&(x < width)&&(y>=0)&&(y<height)){
									if(leftHandState==HandState::HandState_Open){
									cv::circle(bufferMat_color, cv::Point(x, y), 75, cv::Scalar(0, 128, 0), 5, CV_AA);
									}
									if(leftHandState==HandState::HandState_Closed){
									cv::circle(bufferMat_color, cv::Point(x, y), 75, cv::Scalar(0, 0, 128), 5, CV_AA);
									}
									if(leftHandState==HandState::HandState_Lasso){
									cv::circle(bufferMat_color, cv::Point(x, y), 75, cv::Scalar(128, 0, 0), 5, CV_AA);
									}

									}
									*/
									if(x > 1920)x = 1920;
									if(x <= 0)x = 0;
									if(y > 1080)y = 1080;
									if(y <= 0)y = 0;
									std::cout << "PERSON[" << count << "] " << "LeftHandX : " << x << ", LeftHandY : " << y << std::endl;
									outputfile << "PERSON[" << count << "]," << "LeftHandX," << x << ",LeftHandY," << y;
									cv::circle(handcoord_img, cv::Point(x*930/1920, y*510/1080), 20, color_opencv[count], -1, CV_AA);
								}
							}

							//RIGHT HAND!
							HandState rightHandState = HandState::HandState_Unknown;
							hResult = pBody[count]->get_HandRightState(&rightHandState);
							if(SUCCEEDED(hResult)){
								ColorSpacePoint colorSpacePoint = {0};
								hResult = pCoordinateMapper->MapCameraPointToColorSpace(joint[JointType::JointType_HandRight].Position, &colorSpacePoint);
								if(SUCCEEDED(hResult)){
									int x = static_cast<int>(colorSpacePoint.X);
									int y = static_cast<int>(colorSpacePoint.Y);
									/*
									if((x >= 0)&&(x < width)&&(y>=0)&&(y<height)){
									if(rightHandState==HandState::HandState_Open){
									cv::circle(bufferMat_color, cv::Point(x, y), 75, cv::Scalar(0, 128, 0), 5, CV_AA);
									}
									if(rightHandState==HandState::HandState_Closed){
									cv::circle(bufferMat_color, cv::Point(x, y), 75, cv::Scalar(0, 0, 128), 5, CV_AA);
									}
									if(rightHandState==HandState::HandState_Lasso){
									cv::circle(bufferMat_color, cv::Point(x, y), 75, cv::Scalar(128, 0, 0), 5, CV_AA);
									}
									}
									*/
									if(x > 1920)x = 1920;
									if(x <= 0)x = 0;
									if(y > 1080)y = 1080;
									if(y <= 0)y = 0;
									std::cout << "PERSON[" << count << "] " << "RightHandX : " << x << ", RightHandY : " << y << std::endl;
									outputfile << "," << "RightHandX," << x << ",RightHandY," << y;
									cv::circle(handcoord_img, cv::Point(x*930/1920, y*510/1080), 20, color_opencv[count], -1, CV_AA);
								}
							}

							/*
							// Joint
							for( int type = 0; type < JointType::JointType_Count; type++ ){
							ColorSpacePoint colorSpacePoint = { 0 };
							pCoordinateMapper->MapCameraPointToColorSpace( joint[type].Position, &colorSpacePoint );
							int x = static_cast<int>( colorSpacePoint.X );
							int y = static_cast<int>( colorSpacePoint.Y );
							if( ( x >= 0 ) && ( x < width ) && ( y >= 0 ) && ( y < height ) ){
							cv::circle( bufferMat, cv::Point( x, y ), 5, static_cast< cv::Scalar >( color[count] ), -1, CV_AA );
							}
							}
							*/

						}

						// Lean
						PointF amount;
						hResult = pBody[count]->get_Lean( &amount );
						if( SUCCEEDED( hResult ) ){
							std::cout << "PERSON[" << count << "] "<< "leanX : " << amount.X << ",leanY : " << amount.Y << std::endl;
							outputfile << ","<< "leanX," << amount.X << ",leanY," << amount.Y << std::endl;
						}
						send_UDP(abs(amount.Y), 3);
					}
				}//for loop

				cv::resize( bufferMat_color, bodyMat, cv::Size(), 0.5, 0.5 );

			}
			/*
			for( int count = 0; count < BODY_COUNT; count++ ){
			SafeRelease( pBody[count] );
			}
			*/

		}


		/****** FACE RECOGNITION ******/
		// Face Frame
		for( int count = 0; count < BODY_COUNT; count++ ){
			IFaceFrame* pFaceFrame = nullptr;
			hResult = pFaceReader[count]->AcquireLatestFrame( &pFaceFrame );
			if( SUCCEEDED( hResult ) && pFaceFrame != nullptr ){
				BOOLEAN bFaceTracked = false;
				hResult = pFaceFrame->get_IsTrackingIdValid( &bFaceTracked );
				if( SUCCEEDED( hResult ) && bFaceTracked ){
					IFaceFrameResult* pFaceResult = nullptr;
					hResult = pFaceFrame->get_FaceFrameResult( &pFaceResult );
					if( SUCCEEDED( hResult ) && pFaceResult != nullptr ){
						std::vector<std::string> result;

						// Face Point
						PointF facePoint[FacePointType::FacePointType_Count];
						hResult = pFaceResult->GetFacePointsInColorSpace( FacePointType::FacePointType_Count, facePoint );
						if( SUCCEEDED( hResult ) ){
							facetrack_check = 1;
							cv::circle( bufferMat_face, cv::Point( static_cast<int>( facePoint[0].X ), static_cast<int>( facePoint[0].Y ) ), 5, static_cast<cv::Scalar>( color[count] ), -1, CV_AA ); // Eye (Left)
							outputfile << time_buff << "," <<  "PERSON[" << count << "]"<< ",lefteyeX," << static_cast<int>( facePoint[0].X ) << ",lefteyeY," << static_cast<int>( facePoint[0].Y );
							cv::circle( bufferMat_face, cv::Point( static_cast<int>( facePoint[1].X ), static_cast<int>( facePoint[1].Y ) ), 5, static_cast<cv::Scalar>( color[count] ), -1, CV_AA ); // Eye (Right)
							outputfile << ",righteyeX," << static_cast<int>( facePoint[1].X ) << ",righteyeY," << static_cast<int>( facePoint[1].Y );
							cv::circle( bufferMat_face, cv::Point( static_cast<int>( facePoint[2].X ), static_cast<int>( facePoint[2].Y ) ), 5, static_cast<cv::Scalar>( color[count] ), -1, CV_AA ); // Nose
							outputfile << ",noseX," << static_cast<int>( facePoint[2].X ) << ",noseY," << static_cast<int>( facePoint[2].Y );
							cv::circle( bufferMat_face, cv::Point( static_cast<int>( facePoint[3].X ), static_cast<int>( facePoint[3].Y ) ), 5, static_cast<cv::Scalar>( color[count] ), -1, CV_AA ); // Mouth (Left)
							outputfile << ",leftmouthX," << static_cast<int>( facePoint[3].X ) << ",leftmouthY," << static_cast<int>( facePoint[3].Y );
							cv::circle( bufferMat_face, cv::Point( static_cast<int>( facePoint[4].X ), static_cast<int>( facePoint[4].Y ) ), 5, static_cast<cv::Scalar>( color[count] ), -1, CV_AA ); // Mouth (Right)
							outputfile << ",rightmouthX," << static_cast<int>( facePoint[4].X ) << ",rightmouthY," << static_cast<int>( facePoint[4].Y );
						}

						// Face Bounding Box
						RectI boundingBox;
						hResult = pFaceResult->get_FaceBoundingBoxInColorSpace( &boundingBox );
						if( SUCCEEDED( hResult ) ){
							cv::rectangle( bufferMat_face, cv::Rect( boundingBox.Left, boundingBox.Top, boundingBox.Right - boundingBox.Left, boundingBox.Bottom - boundingBox.Top ), static_cast<cv::Scalar>( color[count] ) );
						}

						// Face Rotation
						Vector4 faceRotation;
						hResult = pFaceResult->get_FaceRotationQuaternion( &faceRotation );
						if( SUCCEEDED( hResult ) ){
							int pitch, yaw, roll;
							ExtractFaceRotationInDegrees( &faceRotation, &pitch, &yaw, &roll );
							result.push_back( "Pitch, Yaw, Roll : " + std::to_string( pitch ) + ", " + std::to_string( yaw ) + ", " + std::to_string( roll ) );
							std::cout << "Pitch, Yaw, Roll : " + std::to_string( pitch ) + ", " + std::to_string( yaw ) + ", " + std::to_string( roll ) << std::endl;
							outputfile << ",pitch," << std::to_string( pitch ) << ",yaw," << std::to_string( yaw ) << ",roll," << std::to_string( roll );
						}

						// Face Property
						DetectionResult faceProperty[FaceProperty::FaceProperty_Count];
						hResult = pFaceResult->GetFaceProperties( FaceProperty::FaceProperty_Count, faceProperty );
						if( SUCCEEDED( hResult ) ){
							for( int count = 0; count < FaceProperty::FaceProperty_Count; count++ ){
								switch( faceProperty[count] ){
									case DetectionResult::DetectionResult_Unknown:
										result.push_back( property[count] + " : Unknown" );
										//std::cout << property[count] + " : Unknown" << std::endl;
										outputfile << "," << property[count] << ",Unknown";
										break;
									case DetectionResult::DetectionResult_Yes:
										result.push_back( property[count] + " : Yes" );
										//std::cout << property[count] + " : Yes" << std::endl;
										outputfile << "," << property[count] << ",Yes";
										break;
									case DetectionResult::DetectionResult_No:
										result.push_back( property[count] + " : No" );
										//std::cout << property[count] + " : No" << std::endl;
										outputfile << "," << property[count] << ",No";
										break;
									case DetectionResult::DetectionResult_Maybe:
										result.push_back( property[count] + " : Maybe" );
										//std::cout << property[count] + " : Maybe" << std::endl;
										outputfile << "," << property[count] << ",Maybe";
										break;
									default:
										break;
								}
							}
						}

						if( boundingBox.Left && boundingBox.Bottom ){
							int offset = 30;
							for( std::vector<std::string>::iterator it = result.begin(); it != result.end(); it++, offset += 30 ){
								//cv::putText( bufferMat_face, *it, cv::Point( boundingBox.Left, boundingBox.Bottom + offset ), cv::FONT_HERSHEY_COMPLEX, 1.0f, static_cast<cv::Scalar>( color[count] ), 2, CV_AA );
							}
						}
						outputfile << std::endl;
					}
					SafeRelease( pFaceResult );
				}
			}
			SafeRelease( pFaceFrame );
		}
		cv::resize( bufferMat_face, faceMat, cv::Size(), 0.5, 0.5 );
		/*for face recognition*/


		/****** AUDIO ******/
		hResult = pAudioReader->AcquireLatestBeamFrames( &pAudioFrameList );
		if( SUCCEEDED( hResult ) ){
			//UINT count = 0;
			hResult = pAudioFrameList->get_BeamCount( &count );
			if( SUCCEEDED( hResult ) ){
				for( int index = 0; index < count; index++ ){
					// Frame
					IAudioBeamFrame* pAudioFrame = nullptr;
					//ComPtr<IAudioBeamFrame> pAudioFrame;
					hResult = pAudioFrameList->OpenAudioBeamFrame( index, &pAudioFrame );
					if( SUCCEEDED( hResult ) ){

						IAudioBeamSubFrame* audioBeamSubFrame = NULL;
						//ComPtr<IAudioBeamSubFrame> audioBeamSubFrame;
						hResult = pAudioFrame->GetSubFrame( 0, &audioBeamSubFrame );
						if(SUCCEEDED(hResult)){
							audioBeamSubFrame->CopyFrameDataToArray( subFrameLengthInBytes, (BYTE*)&audioBuffer[0] );

							double sum = 0;
							int stride = sizeof(float);
							float sq = 0.0f;
							int sampleCounter = 0;
							float energy = 0;


							// Get Beam Angle and Confidence
							IAudioBeam* pAudioBeam = nullptr;
							hResult = pAudioFrame->get_AudioBeam( &pAudioBeam );
							if( SUCCEEDED( hResult ) ){
								FLOAT angle = 0.0f;
								FLOAT confidence = 0.0f;
								pAudioBeam->get_BeamAngle( &angle ); // radian [-0.872665f, 0.872665f]
								pAudioBeam->get_BeamAngleConfidence( &confidence ); // confidence [0.0f, 1.0f]


								//COLOR EACH FRAME BLACK
								cv::rectangle(judge_img, cv::Point(0,0), cv::Point(500, 500), cv::Scalar(0,0,0), -1, CV_AA);
								cv::rectangle(rate_img, cv::Point(0,0), cv::Point(500, 500), cv::Scalar(0,0,0), -1, CV_AA);

								if(confidence > 0.3f){
									if(angle * 180.0f / M_PI > 5){
										/*timestamp*/
										time_t now3 = time(NULL);
										struct tm pnow3;
										localtime_s(&pnow3, &now3);
										sprintf_s(time_buff, 50, "%04d%02d%02d_%02d%02d%02d", pnow3.tm_year + 1900, pnow3.tm_mon + 1, pnow3.tm_mday, pnow3.tm_hour, pnow3.tm_min, pnow3.tm_sec);
										outputfile << time_buff << ",";

										volume_flag = 1;
										std::cout << "A is speaking" << std::endl;
										outputfile << "A is speaking,";
										count_a++;
										std::cout << "count_a: " << count_a; 
										total_speak_count++;
										//cv::rectangle(judge_img, cv::Point(250,0), cv::Point(500, 500), cv::Scalar(200,0,0), -1, CV_AA);
										//cv::rectangle(judge_img, cv::Point(0,0), cv::Point(250, 500), cv::Scalar(0,0,0), -1, CV_AA);
										cv::circle(judge_img, cv::Point(375, 250), renormalized_energy*110, cv::Scalar(200,0,0), -1, CV_AA);
										//audioFile.Write( &audioBuffer[0], subFrameLengthInBytes );
									}
									else if(angle * 180.0f / M_PI < -5){
										/*timestamp*/
										time_t now4 = time(NULL);
										struct tm pnow4;
										localtime_s(&pnow4, &now4);
										sprintf_s(time_buff, 50, "%04d%02d%02d_%02d%02d%02d", pnow4.tm_year + 1900, pnow4.tm_mon + 1, pnow4.tm_mday, pnow4.tm_hour, pnow4.tm_min, pnow4.tm_sec);
										outputfile << time_buff << ",";

										volume_flag = 2;
										std::cout << "B is speaking" << std::endl;
										outputfile << "B is speaking,";
										count_b++;
										std::cout << " count_b: " << count_b;
										total_speak_count++;
										std::cout << " total_count: " << total_speak_count << std::endl;
										//cv::rectangle(judge_img, cv::Point(0,0), cv::Point(250, 500), cv::Scalar(0,0,200), -1, CV_AA);
										//cv::rectangle(judge_img, cv::Point(250,0), cv::Point(500, 500), cv::Scalar(0,0,0), -1, CV_AA);
										cv::circle(judge_img, cv::Point(125, 250), renormalized_energy*110, cv::Scalar(0,0,200), -1, CV_AA);
										//audioFile2.Write( &audioBuffer[0], subFrameLengthInBytes );
									}
								}

								float rate_a = 0.0, rate_b = 0.0;
								if(total_speak_count != 0){
									rate_a = (float)count_a / (float)total_speak_count;
									rate_b = (float)count_b / (float)total_speak_count;
									std::cout << "rate_a: " << rate_a << " " << "rate_b: " << rate_b << std::endl;
									cv::rectangle(rate_img, cv::Point(100,100+((1-rate_a)*300)), cv::Point(200, 400), cv::Scalar(200,0,0), -1, CV_AA);
									cv::rectangle(rate_img, cv::Point(300,100+((1-rate_b)*300)), cv::Point(400, 400), cv::Scalar(0,0,200), -1, CV_AA);
									send_UDP(rate_a, 2);
								}
								
								if(volume_flag > 0){
									// Loop over audio data bytes…
									for (int i = 0; i < audioBuffer.size(); i += stride)
									{
										memcpy(&sq, &audioBuffer[i], sizeof(float));
										sum += sq * sq;
										sampleCounter++;
										if (sampleCounter < 40){continue;}
										// Convert to dB
										energy = (float) (10.0 * log10(sum / sampleCounter));
										renormalized_energy = (-90 - energy) / -90;
										if(renormalized_energy > 1.0) renormalized_energy = 1.0;
										if(renormalized_energy <= 0) renormalized_energy = 0.0;
										sampleCounter = sum = 0;
										outputfile << "volume," << renormalized_energy << std::endl;
										total_energy_count += renormalized_energy;
										float average_energy = (float)total_energy_count / (float)total_speak_count;
										//send_UDP(renormalized_energy, 1);
										std::cout << "Average Energy: " << average_energy << std::endl;
										send_UDP(average_energy, 1);
										
										if (renormalized_energy > 0.8){
											std::cout << "Energy is : " << renormalized_energy << std::endl;
											break;
										}
										

									}
								}
								
								volume_flag = 0;

							}

							SafeRelease( pAudioBeam );
							SafeRelease (audioBeamSubFrame);
						}
					}
					SafeRelease( pAudioFrame );
				}
			}
		}



		SafeRelease( pDepthFrame );
		SafeRelease( pColorFrame);
		SafeRelease( pBodyIndexFrame);
		SafeRelease( pBodyFrame);
		//SafeRelease( pAudioFrameList);

		//cv::imshow( "Depth", depthMat );
		//cv::imshow("Color", colorMat);
		cv::imshow("BodyIndex", bodyIndexMat);
		//cv::imshow("Body", bodyMat);
		cv::imshow("Speaker_Judge", judge_img);
		cv::imshow("Speaking_Rate", rate_img);
		cv::imshow("Hand Coordination", handcoord_img);
		cv::imshow( "Face", faceMat );

		if( cv::waitKey( 30 ) == VK_ESCAPE ){
			break;
		}
	}

	SafeRelease( pDepthSource );
	SafeRelease( pDepthReader );
	SafeRelease( pDescription );
	SafeRelease( pColorSource );
	SafeRelease( pColorReader );
	SafeRelease( pDescription_color );
	SafeRelease( pBodyIndexSource );
	SafeRelease( pBodyIndexReader );
	SafeRelease( pBodySource );
	SafeRelease( pBodyReader );
	SafeRelease( pAudioSource );
	SafeRelease( pAudioReader );
	SafeRelease( pDescription_bodyindex );
	for( int count = 0; count < BODY_COUNT; count++ ){
		SafeRelease( pFaceSource[count] );
		SafeRelease( pFaceReader[count] );
	}
	if( pSensor ){
		pSensor->Close();
	}
	SafeRelease( pSensor );

	outputfile.close();

	cv::destroyAllWindows();

	//return (int)msg.wParam;
	return 0;
}

