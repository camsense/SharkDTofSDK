#include "LidarTest.h"
#include <chrono>



LidarTest::LidarTest()
{
	printf("LidarTest: LidarTest()\n");

	m_threadWork = std::thread(&LidarTest::initLidar, this);
}
LidarTest::~LidarTest()
{
	printf("LidarTest: ~LidarTest()\n");
	m_bRun = false;
	if (m_threadWork.joinable())
		m_threadWork.join();

}


void LidarTest::sdkCallBackFunErrorCode(int iErrorCode)
{
	printf("HCSDK Main: sdkCallBackFunErrorCode ErrorCode=%d" , iErrorCode);
}

void LidarTest::sdkCallBackFunSecondInfo(CamsenseDTOF::tsSDKStatistic sInfo)
{
	printf("HCSDK Main: sdkCallBackFunSecondInfo time=%lld s,points=%d,GrayBytes=%d,FPS=%lld,speed=%0.2f,PPS=%d,valid=%d,invalid=%d,ErrorPacket=%lld\n" , 
		sInfo.u64TimeStampS , sInfo.iNumPerPacket, sInfo.iGrayBytes , sInfo.u64FPS
		,sInfo.dRMS , sInfo.iPacketPerSecond,sInfo.iValid , sInfo.iInvalid
		, sInfo.u64ErrorPacketCount);
}

void LidarTest::sdkCallBackFunPointCloud(CamsenseDTOF::LstPointCloud lstG)
{
	printf("HCSDK Main: sdkCallBackFunPointCloud Rx Points=%d\n",lstG.size());
	for (auto sInfo : lstG)
	{
		//std::cout << "Main: Angle=" << sInfo.dAngle  << ",AngleRaw=" << sInfo.dAngleRaw << ",Dist=" << sInfo.u16Dist << std::endl;
	}
}

void LidarTest::sdkCallBackFunDistQ2(CamsenseDTOF::LstNodeDistQ2 lstG)
{
	std::cout << "Main: sdkCallBackFunDistQ2 Rx Points=" << lstG.size() << std::endl;
	for (auto sInfo : lstG)
	{
		//std::cout << "Main: Angle=" << sInfo.angle_q6_checkbit / 64.0f << ",Dist=" << sInfo.distance_q2 / 4 << std::endl;
	}
}

int getPort()
{
	printf("Please select COM:\n");
	int iPort = 3;
    std::cin >> iPort;
	return iPort;
}
void LidarTest::initLidar()
{
	CamsenseDTOF::HCLidar device;
	int rtn = 0;

	bool bPollMode = true;
	bool bDistQ2 = false;
	bool bLoop = false;

	printf("Main: SDK verion=%s\n" , device.getSDKVersion().c_str() );

	/*
	auto funErrorCode = std::bind(&LidarTest::sdkCallBackFunErrorCode, this,std::placeholders::_1);
	device.setCallBackFunErrorCode(funErrorCode);

	auto funSecondInfo = std::bind(&LidarTest::sdkCallBackFunSecondInfo, this,std::placeholders::_1);
	device.setCallBackFunSecondInfo(funSecondInfo);

	if (!bPollMode)//call back
	{
		auto funPointCloud = std::bind(&LidarTest::sdkCallBackFunPointCloud, this,std::placeholders::_1);
		device.setCallBackFunPointCloud(funPointCloud);

		auto funDistQ2 = std::bind(&LidarTest::sdkCallBackFunDistQ2, this,std::placeholders::_1);
		device.setCallBackFunDistQ2(funDistQ2);
	}
	*/

	int iPort = getPort();

	int iBaud = 230400;

#ifdef _WIN32
	rtn = device.initialize("//./com5", iBaud, "PNA3" );                     // For windows OS
#else
	rtn = device.initialize("/dev/ttyUSB0",  iBaud,  "PNA3" );               // For Linux OS
#endif

	if (rtn != 1)
	{
		int iErrorCode = device.getLastErrCode();
		if (iErrorCode == CamsenseDTOF::ERR_SERIAL_INVALID_HANDLE || iErrorCode == CamsenseDTOF::ERR_SERIAL_READFILE_FAILED)
		{
			device.cleanup();
			printf("LidarTest: Init sdk failed!\n");
			return ;
		}

	}
		
#if 0
	std::string strFile = "./fwPNA3_app_V2.13_20260429_7Hz.bin";
	if (device.otaUpgrade(strFile.c_str()))
	{
		printf("LidarTest: Upgrade Fireware success!\n");
	}
	else
	{
		printf("LidarTest: Upgrade Fireware failed!\n");
	}
	device.cleanup();
	return;
#endif		
	if (!device.start())
	{
		device.cleanup();
		printf("LidarTest: Start lidar Parse thread!\n");
		return;
		
	}

	//device.setCircleDataMode();


	//Power on lidar
	


	if (device.getLidarInfo())
	{
		printf("LidarTest: Lidar ID=%s\n", device.getLidarID().c_str());
		printf("LidarTest: Factory Info:%s\n", device.getFactoryInfo().c_str());
		printf("LidarTest: Lidar model:%s\n", device.getLidarModel().c_str());
		printf("LidarTest: Firmware Version:%s\n", device.getFirmwareVersion().c_str());

	}
	

	while (m_bRun)
	{

		if(bPollMode)
		{
			CamsenseDTOF::LstPointCloud lstG;
			if (device.getRxPointClouds(lstG))
			{
				//printf("LidarTest: Poll Rx Points=%d\n", lstG.size());
				for (auto sInfo : lstG)
				{
					printf("LidarTest: Angle=%0.4f,Dist=%d\n", sInfo.dAngle, sInfo.u16Dist);
				}
			}
			else
			{
				int iError = device.getLastErrCode();
				if (iError != CamsenseDTOF::LIDAR_SUCCESS)
				{
					printf("LidarTest: Poll Rx Points error code=%d\n", iError);
					switch (iError)
					{
					case CamsenseDTOF::ERR_SHARK_MOTOR_BLOCKED:
						break;
					case CamsenseDTOF::ERR_SHARK_INVALID_POINTS:
						break;
					case CamsenseDTOF::ERR_LIDAR_SPEED_LOW:
						break;
					case CamsenseDTOF::ERR_LIDAR_SPEED_HIGH:
						break;
					case CamsenseDTOF::ERR_DISCONNECTED:
						break;
					case CamsenseDTOF::ERR_LIDAR_FPS_INVALID:
						break;
					default:
						break;
					}
				}
			}
		}


		std::this_thread::sleep_for(std::chrono::milliseconds(2));
		std::this_thread::yield();
	}
	device.stop();
	device.cleanup();
}

