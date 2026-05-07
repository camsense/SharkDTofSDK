
#include "base/hclidar.h"
#include "LidarTest.h"
#include <stdio.h>
#include <iostream>
#include <string> 
#include <sstream>
#include <algorithm>
#include <fstream>

#include "base/HcData.h"
#include "base/HcSDK.h"

std::string  g_strLidarID = "";



int main()
{
	

   
	while (1)
	{
		LidarTest *lidarTest = new LidarTest();

		std::this_thread::sleep_for(std::chrono::seconds(100));
		std::this_thread::yield();

		if (lidarTest) 
		{
			delete lidarTest;
			lidarTest = nullptr;
		}
	}
	
    return 0;

}
