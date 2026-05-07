#ifndef _HCSDK_H_
#define _HCSDK_H_

#include "HcData.h"

#ifdef __cplusplus
extern "C" {
#endif

	bool hcSDKInitialize(const char* chPort, int iBaud, const char* chLidarModel);
	bool hcSDKUnInit();
	bool hcSDKStart();

	//set callback function for error code
	void setSDKCallBackFunErrorCode(CamsenseDTOF::CallBackFunErroCode fun);

	//set callback function for Statistic infomation
	void setSDKCallBackFunSecondInfo(CamsenseDTOF::CallBackFunSecondInfo fun);

	//set callback function for rx pointclouds
	void setSDKCallBackFunPointCloud(CamsenseDTOF::CallBackFunPointCloud fun);


	//get error code
	int getSDKLastErrCode();

	bool getSDKLidarInfo();

	//get SDK status
	int getSDKStatus();

	// get SDK Version
	char* getSDKVersion();

	//get lidar ID
	char* getSDKLidarID();


	// get factory infomation
	char* getSDKFactoryInfo();

	//get lidar model
	char* getSDKLidarModel();

	//get firmware version
	char* getSDKFirmwareVersion();

	//get hardware version
	char* getSDKHardwareVersion();

	// set work parameter
	void setSDKWorkPara(CamsenseDTOF::tsSDKPara& sSDKPara);

	//poll mode,get pointclouds
	bool getSDKRxPointClouds(CamsenseDTOF::LstPointCloud& lstG);


	//set one circle data to output 
	void setSDKCircleDataMode();

	//set the lidar status is powered
	void setSDKLidarPowerOn(bool bPowerOn);

	//set the lidar low speed status
	void setSDKLidarLowSpeed(bool bLow);

	//零度角补偿开关
	void setSDKAngOffset(bool bAngOffSet);

	//杂点 射线过滤开关
	//void setSDKFilter(bool bfilter);

	void setSDKPointCloudLattice(bool bDistinguish);

	void setSDKD2M7SubComp(bool bComp);



	CamsenseDTOF::tsLDSAttr getSDKLidarAttr();

#ifdef __cplusplus
};
#endif

#endif

