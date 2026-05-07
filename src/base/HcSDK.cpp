
#include "HcSDK.h"
#include <string>
#include <iostream>
#include "hclidar.h"

#ifdef __cplusplus
extern "C" {
#endif

std::string  g_strSDKVer = "";
std::string  g_strSDKLidarID = "";
std::string  g_strSDKFactoryInfo = "";
std::string  g_strSDKLidarModel = "";
std::string  g_strSDKFirmwareVersion = ""; 
std::string  g_strSDKHardwareVersion = ""; 

CamsenseDTOF::HCLidar& g_device = CamsenseDTOF::HCLidar::getInstance();

bool hcSDKInitialize(const char* chPort, int iBaud, const char* chLidarModel)
{
	return g_device.initialize(chPort, iBaud, chLidarModel);
}

bool hcSDKUnInit()
{
	return g_device.cleanup();
}
bool hcSDKStart()
{
	return g_device.start();
}
void setSDKCallBackFunErrorCode(CamsenseDTOF::CallBackFunErroCode fun)
{
	g_device.setCallBackFunErrorCode(fun);
}

void setSDKCallBackFunSecondInfo(CamsenseDTOF::CallBackFunSecondInfo fun)
{
	g_device.setCallBackFunSecondInfo(fun);
}

//set callback function for rx pointclouds
void setSDKCallBackFunPointCloud(CamsenseDTOF::CallBackFunPointCloud fun)
{
	g_device.setCallBackFunPointCloud(fun);
}


//get error code
int getSDKLastErrCode()
{
	return g_device.getLastErrCode();
}

bool getSDKLidarInfo()
{
	return g_device.getLidarInfo();
}

//get SDK status
int getSDKStatus()
{
	return g_device.getSDKStatus();
}

// get SDK Version
char* getSDKVersion()
{
	g_strSDKVer = g_device.getSDKVersion();
	return (char*)g_strSDKVer.c_str();
}

//get lidar ID
char* getSDKLidarID()
{
	g_strSDKLidarID = g_device.getLidarID();
	return (char*)g_strSDKLidarID.c_str();
}


// get factory infomation
char* getSDKFactoryInfo()
{
	g_strSDKFactoryInfo = g_device.getFactoryInfo();
	return (char*)g_strSDKFactoryInfo.c_str();
}

//get lidar model
char* getSDKLidarModel()
{
	g_strSDKLidarModel = g_device.getLidarModel();
	return (char*)g_strSDKLidarModel.c_str();
}

//get firmware version
char* getSDKFirmwareVersion()
{
	g_strSDKFirmwareVersion = g_device.getFirmwareVersion();
	return (char*)g_strSDKFirmwareVersion.c_str();
}

//get hardware version
char* getSDKHardwareVersion()
{
	g_strSDKHardwareVersion = g_device.getHardwareVersion();
	return (char*)g_strSDKHardwareVersion.c_str();
}

// set work parameter
void setSDKWorkPara(CamsenseDTOF::tsSDKPara& sSDKPara)
{
	g_device.setWorkPara(sSDKPara);
}

//poll mode,get pointclouds
bool getSDKRxPointClouds(CamsenseDTOF::LstPointCloud& lstG)
{
	return g_device.getRxPointClouds(lstG);
}



void setSDKCircleDataMode()
{
	g_device.setCircleDataMode();
}

void setSDKLidarPowerOn(bool bPowerOn)
{
	g_device.setLidarPowerOn(bPowerOn);
}

void setSDKLidarLowSpeed(bool bLow)
{
	g_device.setLidarLowSpeed(bLow);
}

void setSDKAngOffset(bool bAngOffSet)
{
	g_device.setAngOffset(bAngOffSet);
}

void setSDKPointCloudLattice(bool bDistinguish)
{
	g_device.setPointCloudLattice(bDistinguish);
}

void setSDKD2M7SubComp(bool bComp)
{
	g_device.setD2M7SubComp(bComp);
}


CamsenseDTOF::tsLDSAttr getSDKLidarAttr()
{
	return g_device.getLidarAttr();
}

#ifdef __cplusplus
};
#endif