#ifndef LIDARTEST_H
#define LIDARTEST_H

#include "base/hclidar.h"

#include <stdio.h>



class LidarTest
{
public:
    LidarTest();
	~LidarTest();

	void initLidar();


private:
	void sdkCallBackFunErrorCode(int iErrorCode);
	void sdkCallBackFunSecondInfo(CamsenseDTOF::tsSDKStatistic sInfo);
	void sdkCallBackFunPointCloud(CamsenseDTOF::LstPointCloud lstG);
	void sdkCallBackFunDistQ2(CamsenseDTOF::LstNodeDistQ2 lstG);

private:
	//CamsenseDTOF::HCLidar *m_device;
	std::thread  m_threadWork;
	bool     m_bRun = true;
};


#endif // LIDARTEST_H
