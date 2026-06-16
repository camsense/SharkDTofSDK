#include "hclidar.h"
#include <vector>
#include <set>

#include <algorithm>
#include <stdio.h>
#include <numeric>
#include <sstream>
#include <fstream>

#include <iostream>
#include <sstream>
#include <iomanip>


namespace CamsenseDTOF {

HCLidar::HCLidar()
{
	initPara();
	memset(&m_sPackUID, 0, sizeof(tsPackUID));
    //m_p8Buff = new UCHAR[READ_BUFF_SIZE];
}

HCLidar::~HCLidar()
{

    cleanup();
}

void HCLidar::initPara()
{
	m_iSDKStatus = SDK_UNINIT;
	m_bScanning = false;
	m_iLastErrorCode = 0;
	m_bDisconnect = true;
	m_bHadID = false;
	m_bGetIDTimeOut = false;
	m_bHadFact = false;
	m_bGetFactTimeOut = false;
	m_bInitTimeout = false;
	m_sStatistic.resetAll();
	m_bCheckSpeed = false;
	m_bYJStart = false;
	
}


void HCLidar::lidarReConnect()
{
	m_bInitTimeout = false;
    m_bHadID=false;
    m_bGetIDTimeOut=false;
    m_bHadFact=false;
    m_bGetFactTimeOut=false;
}

std::string HCLidar::getLidarID()
{
	return m_strDevID;		
}



void HCLidar::setWorkPara(tsSDKPara& sSDKPara)
{
	m_sSDKPara = sSDKPara;
}

bool HCLidar::setLidarPara(const char* chLidarModel)
{
    m_strLidarModel = chLidarModel;
    if (m_strLidarModel == PXA2)
	{
		m_sAttr.dAngleOffsetD = 0;
		m_sAttr.dBaseline_mm = 17;
		m_sAttr.dTheta_d = 0;
		m_sAttr.iFPSMax = FPS_PXA2_MAX;
		m_sAttr.iFPSMin = FPS_PXA2_MIN;
		m_sAttr.iSpeedMax = SPEED_300_MAX;
		m_sAttr.iSpeedMin = SPEED_300_MIN;
		m_sAttr.dAngleStep = 360.0 * SPEED_300_NOR / 60 / FPS_PXA2_NOR;
		m_sAttr.dCirclePoints = FPS_PXA2_NOR * 60 / SPEED_300_NOR;
		m_sAttr.u64TSStepNs = 1e9 / FPS_PXA2_NOR;
		m_sAttr.bAngOffset = true;

		m_sAttr.iFPSNor = FPS_PXA2_NOR;
		m_sAttr.iSpeedNor = SPEED_300_NOR;
	}
	else if (m_strLidarModel == PXA2_A)
	{
		m_sAttr.dAngleOffsetD = 0;
		m_sAttr.dBaseline_mm = 17;
		m_sAttr.dTheta_d = 0;
		m_sAttr.iFPSMax = FPS_PMA1_MAX;
		m_sAttr.iFPSMin = FPS_PMA1_MIN;
		m_sAttr.iSpeedMax = SPEED_420_MAX;
		m_sAttr.iSpeedMin = SPEED_420_MIN;
		m_sAttr.dAngleStep = 360.0 * SPEED_420_NOR / 60 / FPS_4000_PMA1;
		m_sAttr.dCirclePoints = FPS_4000_PMA1 * 60 / SPEED_420_NOR;
		m_sAttr.u64TSStepNs = 1e9 / FPS_4000_PMA1;
		m_sAttr.bAngOffset = true;

		m_sAttr.iFPSNor = FPS_4000_PMA1;
		m_sAttr.iSpeedNor = SPEED_420_NOR;
		m_sAttr.u64TransNs = 2.604 * 1000 * 1000;//(16*3+12)*10/230400 * 1000 * 1000;
	}
	else if (m_strLidarModel == PNA3)
	{
		m_sAttr.dAngleOffsetD = 0;
		m_sAttr.dBaseline_mm = 17;
		m_sAttr.dTheta_d = 0;
		m_sAttr.iFPSMax = FPS_PMA1_MAX;
		m_sAttr.iFPSMin = FPS_PMA1_MIN;
		m_sAttr.iSpeedMax = SPEED_360_MAX;
		m_sAttr.iSpeedMin = SPEED_360_MIN;
		m_sAttr.dAngleStep = 360.0 * SPEED_360_NOR / 60 / FPS_4000_PMA1;
		m_sAttr.dCirclePoints = FPS_4000_PMA1 * 60 / SPEED_360_NOR;
		m_sAttr.u64TSStepNs = 1e9 / FPS_4000_PMA1;
		m_sAttr.bAngOffset = true;

		m_sAttr.iFPSNor = FPS_4000_PMA1;
		m_sAttr.iSpeedNor = SPEED_360_NOR;

		m_sAttr.u64TransNs = 2.604 * 1000 * 1000;//(16*3+12)*10/230400 * 1000 * 1000;
	}
	else if (m_strLidarModel == PMF3)
	{
		m_sAttr.dAngleOffsetD = 0;
		m_sAttr.dBaseline_mm = 17;
		m_sAttr.dTheta_d = 0;
		m_sAttr.iFPSMax = FPS_PMA1_MAX;
		m_sAttr.iFPSMin = FPS_PMA1_MIN;
		m_sAttr.iSpeedMax = SPEED_360_MAX;
		m_sAttr.iSpeedMin = SPEED_360_MIN;
		m_sAttr.dAngleStep = 360.0 * SPEED_360_NOR / 60 / FPS_4000_PMA1;
		m_sAttr.dCirclePoints = FPS_4000_PMA1 * 60 / SPEED_360_NOR;
		m_sAttr.u64TSStepNs = 1e9 / FPS_4000_PMA1;
		m_sAttr.bAngOffset = true;

		m_sAttr.iFPSNor = FPS_4000_PMA1;
		m_sAttr.iSpeedNor = SPEED_360_NOR;

		m_sAttr.u64TransNs = 2.604 * 1000 * 1000;//(16*3+12)*10/230400 * 1000 * 1000;
	}
	else if (m_strLidarModel == PMA2)
	{
		m_sAttr.dAngleOffsetD = 0;
		m_sAttr.dBaseline_mm = 17;
		m_sAttr.dTheta_d = 0;
		m_sAttr.iFPSMax = FPS_PXA2_MAX;
		m_sAttr.iFPSMin = FPS_PXA2_MIN;
		m_sAttr.iSpeedMax = SPEED_300_MAX;
		m_sAttr.iSpeedMin = SPEED_300_MIN;
		m_sAttr.dAngleStep = 360.0 * SPEED_300_NOR / 60 / FPS_PXA2_NOR;
		m_sAttr.dCirclePoints = FPS_PXA2_NOR * 60 / SPEED_300_NOR;
		m_sAttr.u64TSStepNs = 1e9 / FPS_PXA2_NOR;
		m_sAttr.bAngOffset = true;

		m_sAttr.iFPSNor = FPS_PXA2_NOR;
		m_sAttr.iSpeedNor = SPEED_300_NOR;
	}
    else
    {

        return false;
    }
    return true;
}
//bGetLoopData 是否输出一圈的数据
//BOOL HCLidar::initialize(const char* chPort, const char* chLidarModel,int iBaud, int iReadTimeoutMs,  bool bDistQ2,bool bGetLoopData, bool bPollMode)
BOOL HCLidar::initialize(const char* chPort, int iBaud, const char* chLidarModel)
{
	LOG_INFO("Init sdk\n");
    if(m_bScanning)
    {

		LOG_WARNING( "Had been init.\n");
        setReadCharsError(ERR_SDK_HAD_BEEN_INIT);
        return false;
    }
    if(chPort == NULL)
    {
		LOG_ERROR("port is null.\n" );
        setReadCharsError(ERR_SDK_INIT_PARA);
        return false;
    }

    if(chLidarModel == NULL)
    {
		LOG_ERROR("lidar mode is null.\n");
        setReadCharsError(ERR_SDK_INIT_PARA);
        return false;
    }

    m_iReadTimeOutms = 2;
    if( m_iReadTimeOutms<= 0 || m_iReadTimeOutms >= 50)
    {
        m_iReadTimeOutms = READ_TIMEOUT_MS;
    }
	
    if(!setLidarPara(chLidarModel))
    {
		LOG_ERROR("Lidar mode does not exists.\n");
        setReadCharsError(ERR_MODEL_NOT_EXISTS);
        return false;
    }
	resetParam();

	m_iBaud = iBaud;

    if (m_serial.openDevice(chPort, m_iBaud) != 1)
    {

		LOG_ERROR("open serial port failed.\n");
        setReadCharsError(ERR_SERIAL_INVALID_HANDLE);
        return false;
    }

    m_iSDKStatus = SDK_INIT;
	m_bScanning = false;
    m_bGetLoopData = false;
    m_bPollMode = true;



	LOG_INFO("Init complete\n");


	return true;
}
bool HCLidar::start()
{
	if (m_bScanning)
		return false;
	initPara();
	try
	{
		//m_fut = m_prom.get_future();
		m_bScanning = true;
		m_threadWork = std::thread(&HCLidar::threadWork, this);
		m_threadParse = std::thread(&HCLidar::threadParse, this);
		return true;
	}
	catch (...)
    {
        if (m_threadWork.joinable())
        {
            m_threadWork.join();
        }
        if (m_threadParse.joinable())
        {
            m_threadParse.join();
        }
        
        m_bScanning = false; // 

        return false; // 
    }
}
bool HCLidar::getLidarInfo()
{
	if (m_bHadID)
	{
		return true;
	}
	const auto timeout = std::chrono::seconds(1);
	std::unique_lock<std::mutex> lck(m_mtxInit);
	while (!m_bReady) {
		if (m_cvInit.wait_for(lck, timeout) == std::cv_status::timeout) {
			LOG_ERROR("Wait SN signal timeout\n");
			break; // 
		}
	}

	return m_bHadID;
}
bool HCLidar::stop()
{
	if (m_bScanning)
	{
		m_bScanning = false;
		if (m_threadWork.joinable())
			m_threadWork.join();


		if(m_threadParse.joinable())
			m_threadParse.join();
	}
	return true;
	
}
bool HCLidar::cleanup()
{
	stop();	   
	m_serial.closeDevice();
	initPara();
    return true;
}


void HCLidar::setReadCharsError(int errCode)
{
    switch (errCode)
    {
	case -6:
		m_iLastErrorCode = ERR_SERIAL_READFILE_FAILED;
		break;
    case -5:
        m_iLastErrorCode = ERR_SERIAL_INVALID_HANDLE;
        break;
    case -4:
        m_iLastErrorCode = ERR_SERIAL_SETCOMMTIMEOUTS_FAILED;
        break;
    case -3:
        m_iLastErrorCode = ERR_SERIAL_READFILE_FAILED;
        break;
    case -1:
        m_iLastErrorCode = ERR_SERIAL_READFILE_FAILED;
        break;
    case 0:
        m_iLastErrorCode = ERR_SERIAL_READFILE_ZERO;
        break;
    default:
        m_iLastErrorCode = errCode;
        break;
    }
	int iTemp = m_iLastErrorCode;
	LOG_ERROR("Error code:%d\n", iTemp);

    if(m_funErrorCode)
    {
        m_funErrorCode(m_iLastErrorCode);
    }

	std::lock_guard<std::mutex> lock(m_mtxError);
	m_mapErrorCode[m_iLastErrorCode] = HCHead::getCurrentTimestampUs();
}

void HCLidar::threadWork()
{
    while (m_bScanning)
    {
		//if(m_iSDKStatus < SDK_GOT_SN)
			//readSNFromLidar();
		//else
			readData();
    }
}

void HCLidar::readSNFromLidar()
{
	std::vector<UCHAR>       lstSNData;
	UINT64 u64Start = HCHead::getCurrentTimestampUs();
	m_iSDKStatus = SDK_READING_SN;
	while (true)
	{
		memset(m_p8Buff,0, READ_BUFF_SIZE);

		int iRC = m_serial.readData(m_p8Buff, READ_BUFF_SIZE, m_iReadTimeOutms);
		if (iRC < 1)
		{			
			if (iRC == 0)
			{
				// LOG_INFO("HCSDK Info,read data from serial port is 0\n");
				std::this_thread::sleep_for(std::chrono::milliseconds(1));
				std::this_thread::yield();
			}
			else
			{
				setReadCharsError(iRC);
				return;
			}
		}
		UINT64 u64End = HCHead::getCurrentTimestampUs();
		if (u64End >= (u64Start + 3000 * 1000))//2000ms rx sn
		{
			m_bGetIDTimeOut = true;
			m_bHadID = false;
			LOG_INFO("Get Device ID timeout\n");
			m_iSDKStatus = SDK_GOT_SN;
			setReadCharsError(ERR_NOT_ID);
			return;
		}

		for (int i = 0; i < iRC; i++)
		{
			lstSNData.push_back(m_p8Buff[i]);
		}

		if (lstSNData.size() < 100)//68byte*10=680
			continue;

		int iIndex = -1;
		for (int i = 0; i < lstSNData.size() - 1; i++)
		{
			if ((UCHAR)lstSNData.at(i) == 0x5A && (UCHAR)lstSNData.at(i + 1) == 0xAA)//SN
			{
				iIndex = i;
				break;
			}
		}

		if(iIndex < 0)
		{
			if((UCHAR)lstSNData.at(lstSNData.size()-1) != 0x5A)
				lstSNData.clear();

			continue;
		}

		if(iIndex>0)
		{
			HCHead::eraseBuff(lstSNData,iIndex);
		}

		if (!getNewSNInfo(lstSNData))
		{
			continue;
		}
		else
		{
			//m_prom.set_value();
			std::unique_lock <std::mutex> lck(m_mtxInit);
            m_bReady = true;
            m_cvInit.notify_all();

			m_bGetIDTimeOut = false;
			m_bHadID = true;
			m_iSDKStatus = SDK_GOT_SN;
			return;
		}
	}

    
}

void HCLidar::readData()
{

    //memset(m_p8Buff,0, READ_BUFF_SIZE);

    //read data number
    int iRC = m_serial.readData(m_p8Buff, READ_BUFF_SIZE, m_iReadTimeOutms);
    if (iRC < 1)
    {
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		std::this_thread::yield();

        if(iRC == 0)
        {

			if (m_u64StartTimeNoData > 0)
			{
				UINT64 endTime = HCHead::getCurrentTimestampUs();
				if (endTime >= (m_u64StartTimeNoData+m_sSDKPara.iNoDataMS * 1000))
				{
					LOG_WARNING("Lidar read timeout!\n");

					setReadCharsError(iRC);
					m_u64StartTimeNoData = 0;

					m_iReadTimeoutCount++;

					int iTemp2 = m_sSDKPara.iDisconnectMS / m_sSDKPara.iNoDataMS;
					if (m_iReadTimeoutCount >= iTemp2)
					{
						LOG_ERROR("Lidar disconnect! status=%d\n", static_cast<int>(m_iSDKStatus));

						m_iReadTimeoutCount = 0;
						m_bDisconnect = true;
						lidarReConnect();
						m_iSDKStatus = SDK_DISCONNECT;


						setReadCharsError(ERR_DISCONNECTED);
					}
				}
			}
			else
			{
				m_u64StartTimeNoData = HCHead::getCurrentTimestampUs();
			}
			

        }
        else
        {
            setReadCharsError(iRC);
        }

        return;
    }



    m_iReadTimeoutCount=0;
	m_u64StartTimeNoData = 0;
    m_bDisconnect = false;

	std::lock_guard<std::mutex> lock(m_mtxBuff);
	for (int i = 0; i < iRC; i++)
	{
		m_lstTemp.push_back(m_p8Buff[i]);
	}

    
}

void HCLidar::threadParse()
{
	m_u64TSEnd = HCHead::getCurrentTimestampNs();
	UINT64 u64Start = HCHead::getCurrentTimestampUs();

	while (m_bScanning)
	{
		/*if (m_iSDKStatus <= SDK_READING_SN )
		{			
			u64Start = HCHead::getCurrentTimestampUs();
			continue;
		}*/

		checkReadPacketData();

		{
			std::lock_guard<std::mutex> lock(m_mtxBuff);
			if (m_lstTemp.size() > 0)
			{
				std::vector<UCHAR>  lstTemp;
				lstTemp.swap(m_lstTemp);

				m_lstBuff.insert(m_lstBuff.end(), lstTemp.begin(), lstTemp.end());
			}

		}
		
		if (m_bDisconnect)
		{
			std::vector<UCHAR>  lstTemp;
			lstTemp.swap(m_lstBuff);

			u64Start = HCHead::getCurrentTimestampUs();

			m_sStatistic.resetAll();
		}
		else
		{
			if (m_sStatistic.u64TSRxPacketFirst == 0)
			{
				m_sStatistic.u64TSRxPacketFirst = HCHead::getCurrentTimestampUs();
				m_u64StartMS = m_sStatistic.u64TSRxPacketFirst;
			}
			processMain();


		}
			   
		checkChangeSpeed();

		std::this_thread::sleep_for(std::chrono::milliseconds(2));
		std::this_thread::yield();
	}
}



void HCLidar::checkReadPacketData()
{

   // if(m_sStatistic.u64TSRxPacketFirst == 0)
        //return;


    if(bIntervalOneSecond(m_u64StartMS))
    {
        m_sStatistic.u64CurrentS++;
		if (m_strLidarModel != D2YE)
		{
			m_sStatistic.u64FPS = m_sStatistic.iPacketPerSecond*m_sStatistic.iNumPerPacket;
		}

        m_sStatistic.u64TimeStampS = HCHead::getCurrentTimestampUs()/1e6;
        checkInvalidFPS(m_sStatistic.u64FPS);

		m_sStatistic.dRMS = 0;
		double dSum = std::accumulate(m_lstSpeed.begin(), m_lstSpeed.end(), 0);
		if (m_lstSpeed.size() > 0)
		{
			m_sStatistic.dRMS = dSum / m_lstSpeed.size(); //均值
			m_lstSpeed.clear();
		}
			
		calLidarParaBySpeed();


        if(m_funSecondInfo)
        {
            m_funSecondInfo(m_sStatistic);
        }
        m_sStatistic.reset();

		m_u64CountS++;
		//checkLDSVoltage();

		
    }
}

void HCLidar::calLidarParaBySpeed()
{
	if (m_strLidarModel == D3YD)
	{
		if (m_sStatistic.dRMS > SPEED_180_MIN && m_sStatistic.dRMS < SPEED_312_MAX)
		{
			m_sAttr.iSpeedMax = m_sStatistic.dRMS + SPEED_250_RANGE;
			m_sAttr.iSpeedMin = m_sStatistic.dRMS - SPEED_250_RANGE;
			m_sAttr.dAngleStep = 360.0 * m_sStatistic.dRMS / 60 / FPS_2000_NOR;
			m_sAttr.dCirclePoints = FPS_2000_NOR * 60 / m_sStatistic.dRMS;
			m_sAttr.iSpeedNor = m_sStatistic.dRMS;
		}
		
	}
	
}
void HCLidar::processMain()
{
    if(m_lstBuff.size()==0)
        return;
    while (true)
    {
		if (!processData())
				break;
        if(m_lstBuff.size()==0)
            break;

    }
    //printf("Info: buff size = %d\n",m_lstBuff.size());
}


bool HCLidar::checkContinuePacketErr()
{
	if (m_sStatistic.iErrorCountContinue >= NUMBER_CONTINUE_ERROR_PACKET)
	{
		m_sStatistic.iErrorCountContinue = 0;
		if (m_lstBuff.size() > 3)
			HCHead::eraseBuff(m_lstBuff, m_lstBuff.size() - 1);
		else
			m_lstBuff.clear();
		LOG_ERROR("Continuous Rx error packet:%d!\n", NUMBER_CONTINUE_ERROR_PACKET);
		setReadCharsError(ERR_RX_CONTINUE);
		return true;
	}
	return false;
}
bool HCLidar::processData()
{
    if(m_lstBuff.size()<=2)
        return false;

	if (checkContinuePacketErr())
		return false;
	
    int iIndex = -1;
    int iMsgID = 0;
    for(int i=0;i<m_lstBuff.size()-1;i++)
    {
		if ((UCHAR)m_lstBuff.at(i) == 0x5A && (UCHAR)m_lstBuff.at(i + 1) == 0xAA)//SN
		{
			iMsgID = MSG_NEW_SN;
			iIndex = i;
			break;
		}
        
        if((UCHAR)m_lstBuff.at(i) == 0x55 && (UCHAR)m_lstBuff.at(i+1) == 0xAA)//point cloud
        {
            iMsgID = MSG_POINTCLOUD;
            iIndex = i;
            break;
        }
        
    }
    if(iIndex < 0)
    {
		//LOG_INFO("HCSDK Error: rx data not mes header\n" );
		if((UCHAR)m_lstBuff.at(m_lstBuff.size()-1) != 0x55)
			m_lstBuff.clear();
		
		checkFindPackHeader();
        return false;
    }
	else
	{
		m_u64StartTimeFindPackHeader = 0;
	}

    if(iIndex>0)
    {
        HCHead::eraseBuff(m_lstBuff,iIndex);
		//LOG_INFO("HCSDK Error: find mes header,buff size=%d\n" , m_lstBuff.size());
    }

	bool bGetSN = false;
    switch (iMsgID) {
    case MSG_NEW_SN:
	{
		bGetSN = getNewSNInfo(m_lstBuff);

		if(!m_bHadID && bGetSN)
		{
			std::unique_lock <std::mutex> lck(m_mtxInit);
        	m_bReady = true;
            m_cvInit.notify_all();

			m_bGetIDTimeOut = false;
			m_bHadID = true;
			m_iSDKStatus = SDK_GOT_SN;
			
		}
		 
		return bGetSN;   
	}
		
	case MSG_POINTCLOUD:
		return getPointCloud(m_lstBuff);

    }

    return false;

}


void HCLidar::checkFindPackHeader()
{
	if (m_bInitTimeout)
	{
		if (m_u64StartTimeFindPackHeader > 0)
		{
			UINT64 u64Temp = HCHead::getCurrentTimestampUs();
			//UINT64 u64Int = u64Temp - m_u64StartTimeFindPackHeader;

			//if (u64Int >= 100000) //100ms
			if (u64Temp >= (m_u64StartTimeFindPackHeader+100000)) //100ms
			{
				m_u64StartTimeFindPackHeader = 0;
				setReadCharsError(ERR_FIND_HEAD_TIMEOUT);
			}
		}
		else
		{
			m_u64StartTimeFindPackHeader = HCHead::getCurrentTimestampUs();
		}
	}
	else
	{
		m_u64StartTimeFindPackHeader = 0;
	}
}




bool HCLidar::calStartInfo(char* ch,int iLen)
{
    UINT sum = 0;

    for (UINT i = 0; i < iLen; i++)
    {
        if(i==4 || i==5)
        {

        }
        else
            sum += (UCHAR)ch[i];
    }

    const UINT sum_check = ((UCHAR)ch[5] << 8) | (UCHAR)ch[4];
    return (sum_check == sum);
}

bool HCLidar::getNewSNInfo(std::vector<UCHAR>& lstBuff)
{

	//LOG_INFO("getNewSNInfo!\n");

	if (lstBuff.size() < sizeof(tsSDKSN))
		return false;

	tsSDKSN sNewInfo;
	memcpy(&sNewInfo, lstBuff.data(), sizeof(tsSDKSN));

	if (sNewInfo.u16Len == 112)//D2M7
	{
		if (lstBuff.size() < sizeof(tsSDKIDD2M7))
			return false;

		tsSDKIDD2M7 sD2M7;
		memcpy(&sD2M7, lstBuff.data(), sizeof(tsSDKIDD2M7));
		if (calStartInfo((char*)lstBuff.data(), sizeof(tsSDKIDD2M7)))
		{
			HCHead::eraseBuff(lstBuff, sizeof(tsSDKIDD2M7));

			char chTemp[128] = { 0 };
			sprintf(chTemp, "%c%c",
				sD2M7.sSN.u8FacInfo[0], sD2M7.sSN.u8FacInfo[1]);
			m_strFactoryInfo = chTemp;

			memset(chTemp, 0, 128);
			if (m_bSetAngOffset) {
				if (m_sAttr.bAngOffset && sD2M7.sSN.u16Ang != 0xffff) {
					m_dAngOffset = (short)sD2M7.sSN.u16Ang * 0.01;
					m_dAngOffset = fabs(m_dAngOffset) > 3.0 ? 0 : m_dAngOffset;
					LOG_INFO("ZeroAngle=%0.2f\n", m_dAngOffset);
				}
			}
			if (sD2M7.sSN.u8Reserve1[0] == 0xFF)
				sprintf(chTemp, "%c%d%c", sD2M7.sSN.u8FacInfo[2], sD2M7.sSN.u8FacInfo[3], sD2M7.sSN.u8FacInfo[4]);
			else
			{
				if (sD2M7.sSN.u8FacInfo[3] >= 0x40)
					sprintf(chTemp, "%c%c%c%c", sD2M7.sSN.u8FacInfo[2], sD2M7.sSN.u8FacInfo[3], sD2M7.sSN.u8FacInfo[4], sD2M7.sSN.u8Reserve1[0]);
				else
					sprintf(chTemp, "%c%d%c%c", sD2M7.sSN.u8FacInfo[2], sD2M7.sSN.u8FacInfo[3], sD2M7.sSN.u8FacInfo[4], sD2M7.sSN.u8Reserve1[0]);
			}
				

			std::string strModel = chTemp;
			if (strModel != m_strLidarModel)
			{
				LOG_WARNING("Lidar model error init:%s,device:%s\n", (char*)m_strLidarModel.c_str(), (char*)strModel.c_str());
				//setReadCharsError(ERR_DEV_MODEL);
				setLidarPara(chTemp);
			}

			memset(chTemp, 0, 128);

			sprintf(chTemp, "%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c",
				sD2M7.sSN.u8SN[0],
				sD2M7.sSN.u8SN[1],
				sD2M7.sSN.u8SN[2],
				sD2M7.sSN.u8SN[3],
				sD2M7.sSN.u8SN[4],
				sD2M7.sSN.u8SN[5],
				sD2M7.sSN.u8SN[6],
				sD2M7.sSN.u8SN[7],
				sD2M7.sSN.u8SN[8],
				sD2M7.sSN.u8SN[9],
				sD2M7.sSN.u8SN[10],
				sD2M7.sSN.u8SN[11],
				sD2M7.sSN.u8SN[12],
				sD2M7.sSN.u8SN[13],
				sD2M7.sSN.u8SN[14],
				sD2M7.sSN.u8SN[15],
				sD2M7.sSN.u8SN[16],
				sD2M7.sSN.u8SN[17],
				sD2M7.sSN.u8SN[18],
				sD2M7.sSN.u8SN[19]);


			LOG_INFO("Get SN:%s\n", chTemp);
			m_strDevID = chTemp;

			memset(chTemp, 0, 128);
			sprintf(chTemp, "00.00.%02X.%02X",
				sD2M7.sSN.u8CalVer[0], sD2M7.sSN.u8CalVer[1]);

			m_strFirmwareVer = chTemp;


			//m_bHadID = true;


			memcpy(&m_sPackUID, &sD2M7.sSN, sizeof(tsSDKSN));

			memcpy(&m_sD2M7, &sD2M7, sizeof(tsSDKIDD2M7));


			//LOG_INFO("New lidar factory info:%s,Hardware ver:%s\n", (char*)m_strFactoryInfo.c_str(), (char*)m_strHardwareVer.c_str());
			return true;
		}
		else
		{
			HCHead::eraseBuff(lstBuff, sizeof(tsSDKIDD2M7));

			LOG_ERROR("New lidar factory info cal error\n");
			setReadCharsError(ERR_START_INFO);
			return false;

		}
	}
	else if (sNewInfo.u16Len == 70)
	{
		if (lstBuff.size() < sizeof(tsSDKIDD3YD))
			return false;
		tsSDKIDD3YD sD3YD;
		memcpy(&sD3YD, lstBuff.data(), sizeof(tsSDKIDD3YD));
		if (calStartInfo((char*)lstBuff.data(), sizeof(tsSDKIDD3YD)))
		{
			HCHead::eraseBuff(lstBuff, sizeof(tsSDKIDD3YD));

			char chTemp[128] = { 0 };
			sprintf(chTemp, "%c%c",
				sNewInfo.u8FacInfo[0], sNewInfo.u8FacInfo[1]);
			m_strFactoryInfo = chTemp;

			memset(chTemp, 0, 128);
			if (m_bSetAngOffset) {
				if (m_sAttr.bAngOffset && sNewInfo.u16Ang != 0xffff) {
					m_dAngOffset = (short)sNewInfo.u16Ang * 0.01;
					m_dAngOffset = fabs(m_dAngOffset) > 3.0 ? 0 : m_dAngOffset;
					LOG_INFO("ZeroAngle=%0.2f\n", m_dAngOffset);
				}
			}
			if (sNewInfo.u8Reserve1[0] == 0xFF)
				sprintf(chTemp, "%c%d%c", sNewInfo.u8FacInfo[2], sNewInfo.u8FacInfo[3], sNewInfo.u8FacInfo[4]);
			else
			{
				if (sNewInfo.u8FacInfo[3] >= 0x40)
					sprintf(chTemp, "%c%c%c%c", sNewInfo.u8FacInfo[2], sNewInfo.u8FacInfo[3], sNewInfo.u8FacInfo[4], sNewInfo.u8Reserve1[0]);
				else
					sprintf(chTemp, "%c%d%c%c", sNewInfo.u8FacInfo[2], sNewInfo.u8FacInfo[3], sNewInfo.u8FacInfo[4], sNewInfo.u8Reserve1[0]);
			}
				

			std::string strModel = chTemp;
			if (strModel != m_strLidarModel)
			{
				LOG_WARNING("Lidar model error init:%s,device:%s\n", (char*)m_strLidarModel.c_str(), (char*)strModel.c_str());
				//setReadCharsError(ERR_DEV_MODEL);
				setLidarPara(chTemp);
			}

			memset(chTemp, 0, 128);

			sprintf(chTemp, "%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c",
				sNewInfo.u8SN[0],
				sNewInfo.u8SN[1],
				sNewInfo.u8SN[2],
				sNewInfo.u8SN[3],
				sNewInfo.u8SN[4],
				sNewInfo.u8SN[5],
				sNewInfo.u8SN[6],
				sNewInfo.u8SN[7],
				sNewInfo.u8SN[8],
				sNewInfo.u8SN[9],
				sNewInfo.u8SN[10],
				sNewInfo.u8SN[11],
				sNewInfo.u8SN[12],
				sNewInfo.u8SN[13],
				sNewInfo.u8SN[14],
				sNewInfo.u8SN[15],
				sNewInfo.u8SN[16],
				sNewInfo.u8SN[17],
				sNewInfo.u8SN[18],
				sNewInfo.u8SN[19]);


			LOG_INFO("Get SN:%s\n", chTemp);
			m_strDevID = chTemp;

			memset(chTemp, 0, 128);
			sprintf(chTemp, "00.00.%02X.%02X",
				sNewInfo.u8CalVer[0], sNewInfo.u8CalVer[1]);
			m_strFirmwareVer = chTemp;


			//m_bHadID = true;


			memcpy(&m_sPackUID, &sNewInfo, sizeof(tsSDKSN));

			////YJ SN
			memset(chTemp, 0, 128);

			sprintf(chTemp, "%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c",
				sD3YD.u8YJSN[0],
				sD3YD.u8YJSN[1],
				sD3YD.u8YJSN[2],
				sD3YD.u8YJSN[3],
				sD3YD.u8YJSN[4],
				sD3YD.u8YJSN[5],
				sD3YD.u8YJSN[6],
				sD3YD.u8YJSN[7],
				sD3YD.u8YJSN[8],
				sD3YD.u8YJSN[9],
				sD3YD.u8YJSN[10],
				sD3YD.u8YJSN[11],
				sD3YD.u8YJSN[12],
				sD3YD.u8YJSN[13],
				sD3YD.u8YJSN[14],
				sD3YD.u8YJSN[15]);
			LOG_INFO("Get SN:%s\n", chTemp);

			//LOG_INFO("New lidar factory info:%s,Hardware ver:%s\n", (char*)m_strFactoryInfo.c_str(), (char*)m_strHardwareVer.c_str());
			return true;
		}
		else
		{
			HCHead::eraseBuff(lstBuff, sizeof(tsSDKSN));

			LOG_ERROR("New lidar factory info cal error\n");
			setReadCharsError(ERR_START_INFO);
			return false;

		}
	}
	else if (sNewInfo.u16Len == 61)//pna3
	{
		if (lstBuff.size() < sizeof(tsSDKPNA3SN))
			return false;
		tsSDKPNA3SN sPNA3;
		memcpy(&sPNA3, lstBuff.data(), sizeof(tsSDKPNA3SN));

		if (calStartInfo((char*)lstBuff.data(), sizeof(tsSDKPNA3SN)))
		{
			HCHead::eraseBuff(lstBuff, sizeof(tsSDKPNA3SN));

			char chTemp[128] = { 0 };
			sprintf(chTemp, "%c%c",
				sPNA3.u8FacInfo[0], sPNA3.u8FacInfo[1]);
			m_strFactoryInfo = chTemp;

			memset(chTemp, 0, 128);
			if (m_bSetAngOffset) {
				if (m_sAttr.bAngOffset && sPNA3.u16Ang != 0xffff) {
					m_dAngOffset = (short)sPNA3.u16Ang * 0.01;
					m_dAngOffset = fabs(m_dAngOffset) > 3.0 ? 0 : m_dAngOffset;
					LOG_INFO("ZeroAngle=%0.2f\n", m_dAngOffset);
				}
			}
			if (sPNA3.u8Reserve1[0] == 0xFF)
				sprintf(chTemp, "%c%d%c", sPNA3.u8FacInfo[2], sPNA3.u8FacInfo[3], sPNA3.u8FacInfo[4]);
			else
			{
				if (sPNA3.u8FacInfo[3] >= 0x40)
					sprintf(chTemp, "%c%c%c%c", sPNA3.u8FacInfo[2], sPNA3.u8FacInfo[3], sPNA3.u8FacInfo[4], sPNA3.u8Reserve1[0]);
				else
					sprintf(chTemp, "%c%d%c%c", sPNA3.u8FacInfo[2], sPNA3.u8FacInfo[3], sPNA3.u8FacInfo[4], sPNA3.u8Reserve1[0]);
			}

			std::string strModel = chTemp;
			if (strModel != m_strLidarModel)
			{
				LOG_WARNING("Lidar model error init:%s,device:%s\n", (char*)m_strLidarModel.c_str(), (char*)strModel.c_str());
				//setReadCharsError(ERR_DEV_MODEL);
				setLidarPara(chTemp);
			}

			memset(chTemp, 0, 128);

			sprintf(chTemp, "%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c",
				sPNA3.u8SN[0],
				sPNA3.u8SN[1],
				sPNA3.u8SN[2],
				sPNA3.u8SN[3],
				sPNA3.u8SN[4],
				sPNA3.u8SN[5],
				sPNA3.u8SN[6],
				sPNA3.u8SN[7],
				sPNA3.u8SN[8],
				sPNA3.u8SN[9],
				sPNA3.u8SN[10],
				sPNA3.u8SN[11],
				sPNA3.u8SN[12],
				sPNA3.u8SN[13],
				sPNA3.u8SN[14],
				sPNA3.u8SN[15],
				sPNA3.u8SN[16],
				sPNA3.u8SN[17],
				sPNA3.u8SN[18],
				sPNA3.u8SN[19]);


			LOG_INFO("Get 27byte SN:%s\n", chTemp);
			m_strDevID = chTemp;

			memset(chTemp, 0, 128);
			sprintf(chTemp, "00.00.%02X.%02X",
			sPNA3.u8CalVer[0], sPNA3.u8CalVer[1]);

			m_strFirmwareVer = chTemp; 


			//m_bHadID = true;

			//memcpy(&m_sPackUID, &sPNA3, sizeof(tsSDKPNA3SN));


			//LOG_INFO("New lidar factory info:%s,Hardware ver:%s\n", (char*)m_strFactoryInfo.c_str(), (char*)m_strHardwareVer.c_str());
			return true;
		}
		else
		{
			HCHead::eraseBuff(lstBuff, sizeof(tsSDKPNA3SN));

			LOG_ERROR("New lidar factory info cal error\n");
			setReadCharsError(ERR_START_INFO);
			return false;

		}
	}
	else
	{
		if (calStartInfo((char*)lstBuff.data(), sizeof(tsSDKSN)))
		{
			HCHead::eraseBuff(lstBuff, sizeof(tsSDKSN));

			char chTemp[128] = { 0 };
			sprintf(chTemp, "%c%c",
				sNewInfo.u8FacInfo[0], sNewInfo.u8FacInfo[1]);
			m_strFactoryInfo = chTemp;

			memset(chTemp, 0, 128);
			if (m_bSetAngOffset) {
				if (m_sAttr.bAngOffset && sNewInfo.u16Ang != 0xffff) {
					m_dAngOffset = (short)sNewInfo.u16Ang * 0.01;
					m_dAngOffset = fabs(m_dAngOffset) > 3.0 ? 0 : m_dAngOffset;
					LOG_INFO("ZeroAngle=%0.2f\n", m_dAngOffset);
				}
			}
			if (sNewInfo.u8Reserve1[0] == 0xFF)
				sprintf(chTemp, "%c%d%c", sNewInfo.u8FacInfo[2], sNewInfo.u8FacInfo[3], sNewInfo.u8FacInfo[4]);
			else
			{
				if (sNewInfo.u8FacInfo[3] >= 0x40)
					sprintf(chTemp, "%c%c%c%c", sNewInfo.u8FacInfo[2], sNewInfo.u8FacInfo[3], sNewInfo.u8FacInfo[4], sNewInfo.u8Reserve1[0]);
				else
					sprintf(chTemp, "%c%d%c%c", sNewInfo.u8FacInfo[2], sNewInfo.u8FacInfo[3], sNewInfo.u8FacInfo[4], sNewInfo.u8Reserve1[0]);
			}

			std::string strModel = chTemp;
			if (strModel != m_strLidarModel)
			{
				LOG_WARNING("Lidar model error init:%s,device:%s\n", (char*)m_strLidarModel.c_str(), (char*)strModel.c_str());
				//setReadCharsError(ERR_DEV_MODEL);
				setLidarPara(chTemp);
			}

			memset(chTemp, 0, 128);

			sprintf(chTemp, "%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c",
				sNewInfo.u8SN[0],
				sNewInfo.u8SN[1],
				sNewInfo.u8SN[2],
				sNewInfo.u8SN[3],
				sNewInfo.u8SN[4],
				sNewInfo.u8SN[5],
				sNewInfo.u8SN[6],
				sNewInfo.u8SN[7],
				sNewInfo.u8SN[8],
				sNewInfo.u8SN[9],
				sNewInfo.u8SN[10],
				sNewInfo.u8SN[11],
				sNewInfo.u8SN[12],
				sNewInfo.u8SN[13],
				sNewInfo.u8SN[14],
				sNewInfo.u8SN[15],
				sNewInfo.u8SN[16],
				sNewInfo.u8SN[17],
				sNewInfo.u8SN[18],
				sNewInfo.u8SN[19]);


			LOG_INFO("Get 20byte SN:%s\n", chTemp);
			m_strDevID = chTemp;

			memset(chTemp, 0, 128);
			sprintf(chTemp, "00.00.%02X.%02X",
			sNewInfo.u8CalVer[0], sNewInfo.u8CalVer[1]);

			m_strFirmwareVer = chTemp; 


			//m_bHadID = true;



			memcpy(&m_sPackUID, &sNewInfo, sizeof(tsSDKSN));


			//LOG_INFO("New lidar factory info:%s,Hardware ver:%s\n", (char*)m_strFactoryInfo.c_str(), (char*)m_strHardwareVer.c_str());
			return true;
		}
		else
		{
			HCHead::eraseBuff(lstBuff, sizeof(tsSDKSN));

			LOG_ERROR("New lidar factory info cal error\n");
			setReadCharsError(ERR_START_INFO);
			return false;

		}
	}
    return false;
}
 
bool HCLidar::calMCUFrame(char* ch,int iLen)
{
    UINT iSUM = 0;
    for (UINT i = 0; i < iLen-1; i++)
    {
        iSUM += (UCHAR)ch[i];
    }

    UCHAR u8Sum = (UCHAR)iSUM;
    return (u8Sum == (UCHAR)ch[iLen-1]);
}

bool HCLidar::getMCUCmd(std::vector<UCHAR>& lstBuff)
{

    tsBlockMessage  sBlockMessage;
    memcpy(&sBlockMessage, lstBuff.data(), sizeof(tsBlockMessage));

    int iPackLen = sBlockMessage.u8Len; // u8Num is length
    if(iPackLen <= lstBuff.size())//have a good packet
    {
        if(calMCUFrame((char *)lstBuff.data(),iPackLen))
        {
            if(sBlockMessage.u8MsgID == 0x45) //block message for motor
            {
                if(sBlockMessage.u8Code == 0x01)
                {
					LOG_INFO("MCU message <Motor blocked>\n" );

                    /*if(m_iLastErrorCode != ERR_MOTOR_BLOCKED)
                    {
                        setReadCharsError(ERR_MOTOR_BLOCKED);


                    }*/
                    checkSharkBlocked();
                }
                else
                {

					LOG_INFO("MCU message <Lidar reboot>\n");
                    /*if(m_iLastErrorCode != ERR_REBOOT_LIDAR)
                        setReadCharsError(ERR_REBOOT_LIDAR);*/
                }
            }
        }
        HCHead::eraseBuff(lstBuff,iPackLen);
        return true;
    }
    else
    {
        return false;
    }
}

bool HCLidar::getHealthMsg(std::vector<UCHAR>& lstBuff)
{

	tsHealthMessage  sMessage;
	memcpy(&sMessage, lstBuff.data(), sizeof(tsHealthMessage));

	int iPackLen = sMessage.u8Len; // u8Num is length
	if (iPackLen <= lstBuff.size())//have a good packet
	{
		
		if (sMessage.u8Status > 0)
		{
			//LOG_INFO("New Health msg status exceptional: %X\n", sMessage.u8Status);
		}
		HCHead::eraseBuff(lstBuff, iPackLen);
		return true;
	}
	else
	{
		return false;
	}
}

bool HCLidar::getPointCloud(std::vector<UCHAR>& lstBuff)
{

	//printf("Info: getPointCloud!\n");

	int iLen = sizeof(tsPointCloudHead);
	int iMin = iLen < lstBuff.size() ? iLen : lstBuff.size();
	if (lstBuff.size() < iLen)
		return false;

	tsPointCloudHead  sPointCloudHead;
	memcpy(&sPointCloudHead, lstBuff.data(), iMin);

	bool bBlockMessage = false;
	int iPointBytes = 3;
	if (sPointCloudHead.u8Info == 0x02)
	{
		m_sStatistic.iGrayBytes = 0;
		iPointBytes = 2;
	}
	else if (sPointCloudHead.u8Info == 0x07)
	{
		m_sStatistic.iGrayBytes = 2;
		iPointBytes = 4;
	}
	else if (sPointCloudHead.u8Info == 0x80)//MCU message
	{
		bBlockMessage = true;
	}
	else if (sPointCloudHead.u8Info == 0xC2)//MCU message
	{
		return getHealthMsg(lstBuff);
	}
	else
	{
		m_sStatistic.iGrayBytes = 1;
		iPointBytes = 3;
	}

	if (bBlockMessage)
	{
		return getMCUCmd(lstBuff);
	}


	int iPackLen = sPointCloudHead.u8Num * iPointBytes + sizeof(tsPointCloudHead) + sizeof(tsPointCloudTail);

	if (iPackLen <= lstBuff.size())//have a good packet
	{
		/*if (m_sStatistic.u64TSRxPacketFirst == 0)
		{

			m_sStatistic.u64TSRxPacketFirst = HCHead::getCurrentTimestampUs();
			m_u64StartMS = m_sStatistic.u64TSRxPacketFirst;

		}*/
		if (checkDataCal(lstBuff, iPackLen))//good packet
		{

			m_sStatistic.iErrorCountContinue=0;
			m_sStatistic.u64RxPacketCount++;
			m_sStatistic.iNumPerPacket = sPointCloudHead.u8Num;

			parserRangeEX(m_resultRange, (char *)lstBuff.data(), iPackLen, sPointCloudHead.u8Num, iPointBytes);
			if (m_bPollMode)
			{
				
				
				pollModePointCloud();
			}
			else//call back mode
			{
				callBackFunPointCloud();
				
			}

			//LOG_INFO("lidar point cal ok! Point cloud buff size=%d\n" ,m_resultRange.size() );
		}
		else
		{
			m_sStatistic.iErrorCountContinue++;
			LOG_ERROR("lidar point cloud error!\n");
			m_sStatistic.u64ErrorPacketCount++;
			setReadCharsError(ERR_CHECKDATA_INVALID);

		}
		m_sStatistic.iPacketPerSecond++;

		HCHead::eraseBuff(lstBuff, iPackLen);

		return true;
	}
	return false;

}

bool HCLidar::parserRangeEX(LstPointCloud &resultRange, const char * chBuff, int iIndex, int in_numData, int iPointSize)
{

	UINT64 epochTime = HCHead::getCurrentTimestampNs();

	m_u64TSStart = epochTime;

	UINT64 u64Inter = (m_u64TSStart - m_u64TSEnd)/ (in_numData - 1);
	//LOG_INFO("TS:%lld , %lld\n", u64Inter, m_sAttr.u64TSStepNs);

	const int data_size = iPointSize;//const int data_size = 3;
	int id_start = 6;//int id_start = 2;
	double FA = ((UCHAR)chBuff[id_start + 1] - 0xA0 + (UCHAR)chBuff[id_start] / 256.0) * 4;
	int id_LA_start = id_start + in_numData * data_size + 2;
	double LA = ((UCHAR)chBuff[id_LA_start + 1] - 0xA0 + (UCHAR)chBuff[id_LA_start] / 256.0) * 4;
	if (LA < FA) { LA += 360; }
	double dAngle = (LA - FA) / (in_numData - 1);        // angle info for each sampling
	UCHAR *data = new UCHAR[data_size];//unsigned char data[3];
	int pre_bytes = 8;          // 4 bytes before sampling data in each data package
	unsigned int speed = ((UCHAR)chBuff[5] << 8 | (UCHAR)chBuff[4]) / 64;
	m_lstSpeed.push_back(speed);


	//LOG_INFO("Start=%0.4f,end=%0.4f\n", FA, LA);

	double angle_cur = 0;
	const double angle_offset = m_sAttr.dAngleOffsetD;
	for (int i = 0; i < in_numData; ++i)
	{

		angle_cur = FA + dAngle * i;

		//double Aftercompensation = angle_cur;
		if (angle_cur > 360)
		{
			angle_cur -= 360;
		}


		tsPointCloud sData;
		sData.u64TimeStampNs = epochTime - u64Inter *(in_numData-1-i) - m_sAttr.u64TransNs;
		//LOG_INFO("TS:%lld\n", sData.u64TimeStampNs);

		memcpy(data, (UCHAR*)chBuff + pre_bytes + i * data_size, sizeof(UCHAR) * data_size);
		if (((data[1] >> 7) & 0x01) == 1)
			sData.bValid = false;//(data[1] >> 7) & 0x01;

		
		sData.u16DistRaw = ((data[1] & 0x003F) << 8) | data[0];

		if (!sData.bValid)
		{
			sData.u16DistRaw = 0;
		}

		sData.u16Dist = sData.u16DistRaw;
		if (data_size == 2)
			sData.u16Gray = 0;
		else if (data_size == 3)
			sData.u16Gray = data[2] & 0xff;
		else if (data_size == 4)
		{
			sData.bGrayTwoByte = true;
			sData.u16Gray = ((data[3] & 0x00ff) << 8) | data[2];//data[2] + ((data[3]&0x00ff) << 8);
		}
		else
			sData.u16Gray = 0;
		
		sData.u16Speed = speed;
		sData.dAngleRaw = angle_cur;
		sData.dAngle = sData.dAngleRaw;
		sData.dAngleDisp = sData.dAngle;
			
		

		if (m_bSetAngOffset) {
			if (m_sAttr.bAngOffset) {
				sData.dAngle += m_dAngOffset;
			}
		}

		if (m_bDistinguish)
			sData.u16Gray = getPointCloudLatticePeak(sData.u16Dist, sData.u16Gray);


		m_sStatistic.dRMS = speed;
		if (m_bPollMode)
		{
			std::lock_guard<std::mutex> lock(m_mtxData);
			resultRange.push_back(sData);
		}
		else
			resultRange.push_back(sData);

		//std::cout << "Info: Angle=" << sData.dAngle  << ",AngleRaw=" << sData.dAngleRaw << ",Dist=" << sData.u16Dist << std::endl;
		checkSharkInvalidPoints(sData);
	}
	if (data)
	{
		delete[] data;
		data = NULL;
	}

	checkInvalidLowSpeed(speed);
	checkInvalidHighSpeed(speed);
	checkEncoderError(speed);

	if (!m_sAttr.bBlockSpeed) {
		checkBlockSpeed(speed);
	}


	m_u64TSEnd = HCHead::getCurrentTimestampNs();

	return true;
}



void HCLidar::compensate(double &angle, UINT16 &dist, const double theta_d, const double baseline_mm)
{
    if (0 != dist)
    {
        const double theta = theta_d / 180. * PI_HC;
        const double beta = atan(dist / baseline_mm);
        double angle_correct = PI_HC / 2. - (theta + beta);
        angle = angle + angle_correct / PI_HC * 180;
        //angle = angle + 80 - atan(dist / baseline_mm)/ PI_HC * 180;  //X2
		//angle = angle + 118.5 - atan(dist / baseline_mm) / PI_HC * 180; //D2
        if (angle > 360)
        {
            angle -= 360;
        }
        else if (angle < 0)
        {
            angle += 360;
        }

        dist = sqrt(baseline_mm * baseline_mm + dist * dist);
    }
	else
	{
		double dTemp = atan(999999999 / baseline_mm) / PI_HC * 180;
		angle = angle + 80 - dTemp;
		if (angle > 360)
		{
			angle -= 360;
		}
		else if (angle < 0)
		{
			angle += 360;
		}
	}
}

bool HCLidar::checkDataCalTof(std::vector<UCHAR>& lstBuff, int iIndex)
{
	int sum = 0;
	for (int i = 0; i < iIndex - 2; ++i)
	{
		if (i % 2 == 0)
		{
			sum = sum + lstBuff[i];
		}
		else
		{
			sum = sum + (lstBuff[i] << 8);
		}
	}
	unsigned char low = sum;
	unsigned char high = (sum >> 8);
	if (low != lstBuff[iIndex - 2] || high != lstBuff[iIndex - 1])
	{
		//printf("low:%02x %02x, hight:%02x, %02x\n", low, lstBuff[iIndex - 2], high, lstBuff[iIndex - 1]);
		//printf("MSG:");
		//for (int i = 0; i < iIndex; ++i)
		//{
			//printf("%02x  ", lstBuff[i]);
		//}

		//printf("\n");
		return false;
	}
	return true;
}

bool HCLidar::checkDataCal(std::vector<UCHAR>& lstBuff, int iIndex)
{
	const int iB = iIndex - 2;
	std::vector<int> temp(iB / 2, 0);
	//temp[0] = 0x55 + (0xAA << 8);
	for (int i = 0; i < temp.size(); i++)
	{
		temp[i] = lstBuff.at(2 * i) + (lstBuff.at(2 * i + 1) << 8);
	}

	int chk32 = 0;
	for (int i = 0; i < temp.size(); i++)
	{
		chk32 = (chk32 << 1) + temp[i];
	}
	
	UINT16 checksum_target = (chk32 & 0x7FFF) + (chk32 >> 15);
	checksum_target = checksum_target & 0x7FFF;
	UINT16 checksum_cur = lstBuff.at(iIndex - 2) + (lstBuff.at(iIndex - 1) << 8);
	bool is_equal = (checksum_target == checksum_cur);
	return is_equal;
}


bool HCLidar::getRxPointClouds(LstPointCloud& lstG)
{
	if (!m_bPollMode)
	{
		setReadCharsError(ERR_POLL_MODE);
		return false;
	}
	if (lstG.size() > 0)
	{
		LstPointCloud tmp;
		tmp.swap(lstG);
	}

	std::lock_guard<std::mutex> lock(m_mtxData);
	if (m_bCircle)
	{
		if (m_Circles.size() > 0)
		{
			LstPointCloud tmp = *m_Circles.begin();

			lstG.swap(tmp);
			m_Circles.erase(m_Circles.begin());
		}
		else
			return false;

	}
	else
	{
		if (m_resultRange.size() > 0)
		{
			lstG.swap(m_resultRange);
		}
		else
			return false;
	}

	return true;
}


void HCLidar::checkInvalidFPS(int iFPS)
{
	if (m_sStatistic.u64CurrentS < 2)
		return;
    if (iFPS < m_sAttr.iFPSMin || iFPS > m_sAttr.iFPSMax)
    {
        ++m_iInvalidFPSSecond;
		int iTemp = m_sSDKPara.iFPSContinueMS / 1000;
        if (m_iInvalidFPSSecond > iTemp)
        {
			LOG_ERROR("FPS:%d \n" , iFPS );
            setReadCharsError(ERR_LIDAR_FPS_INVALID);

            m_iInvalidFPSSecond = 0;
        }
    }
    else
    {
        m_iInvalidFPSSecond = 0;
    }

	/*if (m_u64CountS>= SENSOR_ERROR_SECOND && (iFPS == 0 || m_iLastErrorCode == ERR_LIDAR_FPS_INVALID))
	{
		if (!getErrorCode(ERR_MOTOR_BLOCKED, SENSOR_ERROR_TIME_MS))
			setReadCharsError(ERR_LIDAR_SENSOR);
	}*/
}


void HCLidar::checkInvalidLowSpeed(UINT16 u16Speed)
{
	if (m_sStatistic.u64CurrentS < 2)
		return;
    if (u16Speed < m_sAttr.iSpeedMin)
    {
        if (m_u64StartTimeLowSpeed != 0)
        {
            UINT64 endTime = HCHead::getCurrentTimestampUs();
            //if ((endTime - m_u64StartTimeLowSpeed) >= m_sSDKPara.iSpeedContinueMS*1000)
			if (endTime >= (m_u64StartTimeLowSpeed + m_sSDKPara.iSpeedContinueMS * 1000))
            {
                setReadCharsError(ERR_LIDAR_SPEED_LOW);
                m_u64StartTimeLowSpeed = 0;
            }
        }
        else
        {
            m_u64StartTimeLowSpeed = HCHead::getCurrentTimestampUs();
        }
    }
    else
    {
        m_u64StartTimeLowSpeed = 0;
    }
}

void HCLidar::checkInvalidHighSpeed(UINT16 u16Speed)
{
	if (m_sStatistic.u64CurrentS < 2)
		return;
    if (u16Speed > m_sAttr.iSpeedMax)
    {
        if (m_u64StartTimeHighSpeed != 0)
        {
            int64_t endTime = HCHead::getCurrentTimestampUs();
            //if ((endTime - m_u64StartTimeHighSpeed) >= m_sSDKPara.iSpeedContinueMS*1000)
			if (endTime   >= (m_u64StartTimeHighSpeed + m_sSDKPara.iSpeedContinueMS * 1000))
            {
                setReadCharsError( ERR_LIDAR_SPEED_HIGH);
                m_u64StartTimeHighSpeed = 0;
            }
        }
        else
        {
            m_u64StartTimeHighSpeed = HCHead::getCurrentTimestampUs();
        }
    }
    else
    {
        m_u64StartTimeHighSpeed = 0;
    }
}

void HCLidar::checkEncoderError(UINT16 u16Speed)
{
	if (u16Speed == 0 || u16Speed == 1023)
	{
		//LOG_INFO("Speed error=%d\n", u16Speed);
		if (m_u64StartTimeSpeed != 0)
		{
			UINT64 endTime = HCHead::getCurrentTimestampUs();
			//if ((u16Speed != m_u16Speed) && (endTime - m_u64StartTimeSpeed) <= ENCODER_ERROR_TIME_MS*1000)
			if ((u16Speed != m_u16Speed) && (endTime  <= (m_u64StartTimeSpeed+ENCODER_ERROR_TIME_MS * 1000)))
			{
				if (m_sStatistic.u64CurrentS > ENCODER_ERROR_SECOND)
				{
					if(!getErrorCode(ERR_MOTOR_BLOCKED, ENCODER_ERROR_TIME_MS))
						setReadCharsError(ERR_LIDAR_ENCODER);
				}
					
				m_u64StartTimeSpeed = 0;
			}
		}
		else
		{
			m_u64StartTimeSpeed = HCHead::getCurrentTimestampUs();
		}

		
	}
	else
	{
		m_u64StartTimeSpeed = 0;
	}
	m_u16Speed = u16Speed;
}

void HCLidar::checkLDSVoltage()
{
	if (m_u64CountS >= LDS_VOLTAGE_ERROR_SECOND && m_sStatistic.u64FPS == 0 && !m_bHadID && m_bDisconnect)
	{
		setReadCharsError(ERR_LIDAR_VOLTAGE);
	}
}

void HCLidar::checkPDCurrent()
{
	if(!getErrorCode(ERR_LIDAR_FPS_INVALID, PD_ERROR_TIME_MS) && (!getErrorCode(ERR_LIDAR_SPEED_LOW, PD_ERROR_TIME_MS) || !getErrorCode(ERR_LIDAR_SPEED_HIGH, PD_ERROR_TIME_MS)))
	{
		setReadCharsError(ERR_LIDAR_PD_CURRENT);
	}
		
}

void HCLidar::checkSharkBlocked()
{
    if (m_u64StartTimeSharkBlock != 0)
    {
        UINT64 endTime = HCHead::getCurrentTimestampUs();
        UINT64 u64Temp = endTime - m_u64StartTimeSharkBlock;

        if(u64Temp >= (m_sSDKPara.iBlockContinueMS+2000)*1000) //3.5+2S
        {
            m_u64StartTimeSharkBlock = 0;
            m_iSharkBlockCount=0;
        }
        else if(u64Temp >= m_sSDKPara.iBlockContinueMS*1000 && u64Temp < (m_sSDKPara.iBlockContinueMS+2000)*1000)//3.5 - 4.5s
        {
            if(m_iSharkBlockCount>0)
            {
				//LOG_ERROR("Shark moto block\n");
                setReadCharsError(ERR_SHARK_MOTOR_BLOCKED);
                m_u64StartTimeSharkBlock = 0;
                m_iSharkBlockCount=0;
            }
            else
            {
                m_u64StartTimeSharkBlock = 0;
                m_iSharkBlockCount=0;
            }

        }
        else if(u64Temp < 100*1000)//100ms
        {
            m_u64StartTimeSharkBlock = endTime;
        }
        else if(u64Temp >= MCU_BLOCK_TIME_MS*1000)
        {
            m_iSharkBlockCount++;
        }
        else
        {
        }
    }
    else
    {

        m_u64StartTimeSharkBlock = HCHead::getCurrentTimestampUs();
        m_iSharkBlockCount=0;

    }
}


void HCLidar::checkSharkInvalidPoints(tsPointCloud& sData)
{
    if (!sData.bValid)
    {
		m_iInvalidPointsCount++;
        if (m_u64StartTimeInvalidPoints != 0)
        {
            int64_t endTime = HCHead::getCurrentTimestampUs();
			int iTemp = m_sSDKPara.iCoverContinueMS / 1000;
            //if ((endTime - m_u64StartTimeInvalidPoints) >= m_sSDKPara.iCoverContinueMS*1000 && m_iInvalidPointsCount> m_sAttr.iFPSNor*iTemp)

			if ((endTime  >= (m_u64StartTimeInvalidPoints + m_sSDKPara.iCoverContinueMS * 1000 )) && (m_iInvalidPointsCount > m_sAttr.iFPSNor*iTemp))
            {
                setReadCharsError( ERR_SHARK_INVALID_POINTS);
                m_u64StartTimeInvalidPoints = 0;
            }
        }
        else
        {
            m_u64StartTimeInvalidPoints = HCHead::getCurrentTimestampUs();
        }
    }
    else
    {
        m_u64StartTimeInvalidPoints = 0;
		m_iInvalidPointsCount = 0;
    }
}




void HCLidar::checkInvalidLidarNumber(int validNumber)
{
    if (validNumber < VALID_NUMBER_COUNT)
    {
        ++m_iInvalidNumberContinue;
        if (m_iInvalidNumberContinue >= NUMBER_CONTINUE_CIRCLE)
        {
            setReadCharsError(ERR_LIDAR_NUMBER_INVALID);
            m_iInvalidNumberContinue = 0;

			checkPDCurrent();
        }
    }
    else
    {
        m_iInvalidNumberContinue = 0;
    }
}


void HCLidar::pollModePointCloud()
{
	//LOG_INFO("pollModePointCloud\n");

	
	if (m_bCircle)
	{
		if (getOneCircleData())
		{
			std::lock_guard<std::mutex> lock(m_mtxData);
			m_Circles.push_back(m_lstCircle);
			LstPointCloud tmp;
			tmp.swap(m_lstCircle);

		}
		std::lock_guard<std::mutex> lock(m_mtxData);
		if (m_Circles.size() > m_sSDKPara.iCirclesBuffSize)
		{
			m_Circles.erase(m_Circles.begin());
			setReadCharsError(ERR_BUFF_FULL);
		}
	}
	else
	{
		std::lock_guard<std::mutex> lock(m_mtxData);
		if (m_resultRange.size() > m_sSDKPara.iPollBuffSize)
		{
			HCHead::eraseRangeData(m_resultRange, m_resultRange.size() - m_sSDKPara.iPollBuffSize + 100);
			setReadCharsError(ERR_BUFF_FULL);
		}
	}
}
void HCLidar::callBackFunPointCloud()
{
	if (m_bCircle)
	{
		if (getOneCircleData())
		{
			if (m_funPointCloud)
			{

				m_funPointCloud(m_lstCircle);
			}
			else
			{
				setReadCharsError(ERR_CALLBACK_FUN);
			}
			LstPointCloud tmp;
			tmp.swap(m_lstCircle);
		}
	}
	else
	{
		if (m_resultRange.size() >= m_sSDKPara.iCallbackBuffSize)
		{
			if (m_funPointCloud)
			{
				m_funPointCloud(m_resultRange);
			}
			else
			{
				setReadCharsError(ERR_CALLBACK_FUN);
			}
			LstPointCloud tmp;
			tmp.swap(m_resultRange);
		}
	}

	
}

bool HCLidar::getOneCircleData()
{

	if (m_strLidarModel == D2YE)
	{
		if (m_iPackIndex == 0)
		{
			//LOG_ERROR("Get one circle-----------%d\n", m_lstCircle.size());
			checkInvalidLidarNumber(m_iValidNumber);
			m_iValidNumber = 0;

			return true;

		}
		else
		{
			for (auto sInfo : m_resultRange)
			{
				if (sInfo.bValid)
					++m_iValidNumber;

				m_lstCircle.push_back(sInfo);
			}

			//LOG_ERROR("=====m_lstCircle=%d,m_resultRange=%d\n", m_lstCircle.size(), m_resultRange.size());
			LstPointCloud tmp;
			tmp.swap(m_resultRange);
		}
		return false;
	}

#if ANGLE_SOLUTION
	bool bHadOne = false;
	int iCount = 0;
	for (auto sInfo : m_resultRange)
	{
		m_dAngleCur = sInfo.dAngleRaw;
		if (sInfo.bValid)
			++m_iValidNumber;

		if (m_lstCircle.size() > 0)
		{
			auto sTemp = m_lstCircle.front();
			double dTemp = sTemp.dAngleRaw - m_dAngleCur;
			if ((dTemp <= m_sAttr.dAngleStep && dTemp > 0) || (m_lstCircle.size() >= m_sAttr.dCirclePoints))
			{
				bHadOne = true;
				iCount++;
				m_lstCircle.push_back(sInfo);
				m_resultRange.erase(m_resultRange.begin(), m_resultRange.begin() + iCount);
				checkInvalidLidarNumber(m_iValidNumber);
				m_iValidNumber = 0;
				break;
			}

		}

		iCount++;

		m_lstCircle.push_back(sInfo);

		m_dAnglePre = m_dAngleCur;
	}
#else
	bool bHadOne = false;
	int iCount = 0;
	for (auto sInfo : m_resultRange)
	{
		m_dAngleCur = sInfo.dAngleRaw;
		if (sInfo.bValid)
			++m_iValidNumber;
		if (m_lstCircle.size() > 200)
		{
			auto sTemp = m_lstCircle.back();
			m_dAnglePre = sTemp.dAngleRaw;
			if (m_dAnglePre > m_dAngleCur && (m_dAnglePre - m_dAngleCur) > 280)
			{
				bHadOne = true;
								
				m_resultRange.erase(m_resultRange.begin(), m_resultRange.begin() + iCount);
				//iCount++;
				//m_lstCircle.push_back(sInfo);
				checkInvalidLidarNumber(m_iValidNumber);
				m_iValidNumber = 0;
				break;
			}
		}
		
		

		iCount++;

		m_lstCircle.push_back(sInfo);

		m_dAnglePre = m_dAngleCur;
	}
#endif

	if (bHadOne)
	{
		//std::stable_sort(m_lstCircle.begin(), m_lstCircle.end(), newComparator);		
	}
	else
	{
		LstPointCloud tmp;
		tmp.swap(m_resultRange);
	}

	return bHadOne;
	   	
}


bool HCLidar::getErrorCode(int iError, int iMs)
{
	std::lock_guard<std::mutex> lock(m_mtxError);
	std::map<int, UINT64>::iterator it = m_mapErrorCode.find(iError);
	if (it != m_mapErrorCode.end())
	{
		UINT64 u64TS = it->second;
		UINT64 u64EndUs = HCHead::getCurrentTimestampUs();
		//UINT64 u64Int = u64EndUs - u64TS;

		//if (u64Int <= iMs*1000)
		if (u64EndUs <= (u64TS + iMs * 1000))
		{
			return true;
		}
	}
	return false;
}

void HCLidar::setLidarPowerOn(bool bPowerOn)
{
	m_bDisconnect = !bPowerOn;
}

///////////////////////////////
bool HCLidar::rockCheckLDSInfo(UINT8* buffer, unLidarInfo& lds_info)
{

	int size = 0;
	int decode_size = 0;

	UINT8 decode_buff[RCV_SIZE];

	rockDecodeLDSInfoPacket(buffer, decode_buff, decode_size);

	if (decode_size < sizeof(unLidarInfo))
	{
		//qDebug() << "Fail to Check LDS info, decoded buff size error.";
		return false;
	}

	memcpy(&lds_info, &decode_buff, sizeof(unLidarInfo));

	if (!rockCheckCRC(lds_info))
	{
		//qDebug() << "Wrong CRC for LDS info.";
		return false;
	}

	//qDebug() << "Get LDS Info success";

	return true;
}

void HCLidar::rockDecodeLDSInfoPacket(UINT8* src, UINT8* dest, int& decode_size)
{
	decode_size = 0;
	for (int i = 0; i < RCV_SIZE; i++)
	{
		if (src[i] == (LDS_INFO_START - 1))
		{
			if (i < RCV_SIZE - 1)
			{
				dest[decode_size++] = (src[i + 1] == 0x01 ? LDS_INFO_START : (LDS_INFO_START - 1));
				i++;
			}
		}
		else
		{
			dest[decode_size++] = src[i];
		}
	}
}

bool HCLidar::rockCheckCRC(unLidarInfo lds_info)
{
	UINT16 u16CheckSum = 0;
	for (int i = 0; i < (sizeof(unLidarInfo) / sizeof(UINT16) - 1); i++)
	{
		u16CheckSum += lds_info.u16DataHex[i];
	}
	if (u16CheckSum != lds_info.sAttr.u16CheckSum)
	{
		//qDebug() << "LDS info checksum: 0x" << lds_info.lds_attributes.check_sum << " VS  Count_result: 0x" << checksum;
		return false;
	}

	return true;
}



void HCLidar::checkChangeSpeed()
{
	if (!m_bCheckSpeed)
	{
		return;
	}

	if (m_sStatistic.dRMS >= m_sAttr.iSpeedMin && m_sStatistic.dRMS <= m_sAttr.iSpeedMax)
	{
		m_iCheckSpeedCount = 0;
	}
	else
	{
		m_iCheckSpeedCount++;
	}

	UINT64 endTime = HCHead::getCurrentTimestampUs();
	//UINT64 u64Temp = endTime - m_u64StartTimeCheckSpeed;

	//if (u64Temp >= (m_sSDKPara.iChangeSpeedMS ) * 1000) //2.5S
	if (endTime >= (m_u64StartTimeCheckSpeed +m_sSDKPara.iChangeSpeedMS * 1000)) //2.5S
	{
		if (m_iCheckSpeedCount>10)
		{
			setReadCharsError(ERR_LIDAR_CHANGE_SPEED);
		}

		
		m_bCheckSpeed = false;
		m_u64StartTimeCheckSpeed = 0;
	}

}



void HCLidar::checkBlockSpeed(UINT16 u16speed)
{
	if (u16speed <= m_sSDKPara.iMinBlockSpeed) {
		if (m_u64checkBlockSpeed != 0) {
			std::chrono::milliseconds curtime(HCHead::getCurrentTimestampMs());
			std::chrono::milliseconds st(m_u64checkBlockSpeed);
			if (curtime - st >= std::chrono::milliseconds(m_sSDKPara.iBlockContinueMS)) {
				setReadCharsError(ERR_SHARK_MOTOR_BLOCKED);
				m_u64checkBlockSpeed = 0;
			}

		}
		else {
			m_u64checkBlockSpeed = HCHead::getCurrentTimestampMs();
		}
	}
	else {
		m_u64checkBlockSpeed = 0;

	}
}

void HCLidar::resetParam()
{
	LOG_INFO("resetParam\n");
	m_iInvalidFPSSecond = 0;
	m_u64StartTimeLowSpeed = 0;
	m_u64StartTimeHighSpeed = 0;
	m_u64StartTimeSharkBlock = 0;
	m_iSharkBlockCount = 0;
	m_u64StartTimeInvalidPoints = 0;
	m_iValidNumber = 0;
	m_u64checkBlockSpeed = 0;
	m_iLastErrorCode = 0;
	m_iInvalidPointsCount = 0;
}


void HCLidar::setLidarLowSpeed(bool bLow)
{
	LOG_INFO("Change speed\n");
	if (bLow)
	{
		if (m_strLidarModel == X2YE)
		{
			if (m_sAttr.dAngleStep != ANGLE_RESOLV_NARWAL_LOW)
			{
				m_bCheckSpeed = true;
				m_u64StartTimeCheckSpeed = HCHead::getCurrentTimestampUs();
			}

			m_sAttr.iSpeedMax = SPEED_250_MAX;
			m_sAttr.iSpeedMin = SPEED_250_MIN;
			m_sAttr.dAngleStep = ANGLE_RESOLV_NARWAL_LOW;
			m_sAttr.dCirclePoints = CICRLE_MAX_NARWAL_LOW;

			UCHAR u8Temp[5] = { 0xAA,0x55,0xF1,0x0E };
			m_serial.writeData2(u8Temp, 4);
		}
		else if (m_strLidarModel == X2BF)
		{
			if (m_sAttr.dAngleStep != ANGLE_RESOLV_3000_3HZ)
			{
				m_bCheckSpeed = true;
				m_u64StartTimeCheckSpeed = HCHead::getCurrentTimestampUs();
			}

			m_sAttr.iSpeedMax = SPEED_180_MAX;
			m_sAttr.iSpeedMin = SPEED_180_MIN;
			m_sAttr.dAngleStep = ANGLE_RESOLV_3000_3HZ;
			m_sAttr.dCirclePoints = CICRLE_MAX_3000_3HZ;

		}
		else if (m_strLidarModel == X2MF)
		{
			if (m_sAttr.dAngleStep != ANGLE_RESOLV_2000_3HZ)
			{
				m_bCheckSpeed = true;
				m_u64StartTimeCheckSpeed = HCHead::getCurrentTimestampUs();
			}

			m_sAttr.iSpeedMax = SPEED_180_MAX;
			m_sAttr.iSpeedMin = SPEED_180_MIN;
			m_sAttr.dAngleStep = ANGLE_RESOLV_2000_3HZ;
			m_sAttr.dCirclePoints = CICRLE_MAX_2000_3HZ;

		}
		else if (m_strLidarModel == D2PD)
		{
			if (m_sAttr.dAngleStep != ANGLE_RESOLV_2000_3HZ)
			{
				m_bCheckSpeed = true;
				m_u64StartTimeCheckSpeed = HCHead::getCurrentTimestampUs();
			}

			m_sAttr.iSpeedMax = SPEED_180_MAX;
			m_sAttr.iSpeedMin = SPEED_180_MIN;
			m_sAttr.dAngleStep = ANGLE_RESOLV_2000_3HZ;
			m_sAttr.dCirclePoints = CICRLE_MAX_2000_3HZ;

		}
		else if (m_strLidarModel == D2AE)
		{
			if (m_sAttr.dAngleStep != ANGLE_RESOLV_2000_3HZ)
			{
				m_bCheckSpeed = true;
				m_u64StartTimeCheckSpeed = HCHead::getCurrentTimestampUs();
			}

			m_sAttr.iSpeedMax = SPEED_180_MAX;
			m_sAttr.iSpeedMin = SPEED_180_MIN;
			m_sAttr.dAngleStep = ANGLE_RESOLV_2000_3HZ;
			m_sAttr.dCirclePoints = CICRLE_MAX_2000_3HZ;

		}
		else if (m_strLidarModel == A200)
		{
			if (m_sAttr.dAngleStep != ANGLE_RESOLV_2000_3HZ)
			{
				m_bCheckSpeed = true;
				m_u64StartTimeCheckSpeed = HCHead::getCurrentTimestampUs();
			}

			m_sAttr.iSpeedMax = SPEED_180_MAX;
			m_sAttr.iSpeedMin = SPEED_180_MIN;
			m_sAttr.dAngleStep = ANGLE_RESOLV_2000_3HZ;
			m_sAttr.dCirclePoints = CICRLE_MAX_2000_3HZ;

		}
		else if (m_strLidarModel == A201)
		{
			if (m_sAttr.dAngleStep != ANGLE_RESOLV_2000_3HZ)
			{
				m_bCheckSpeed = true;
				m_u64StartTimeCheckSpeed = HCHead::getCurrentTimestampUs();
			}

			m_sAttr.iSpeedMax = SPEED_180_MAX;
			m_sAttr.iSpeedMin = SPEED_180_MIN;
			m_sAttr.dAngleStep = ANGLE_RESOLV_2000_3HZ;
			m_sAttr.dCirclePoints = CICRLE_MAX_2000_3HZ;

		}
		else if (m_strLidarModel == D2AG)
		{
			if (m_sAttr.dAngleStep != ANGLE_RESOLV_2000_3HZ)
			{
				m_bCheckSpeed = true;
				m_u64StartTimeCheckSpeed = HCHead::getCurrentTimestampUs();
			}

			m_sAttr.iSpeedMax = SPEED_180_MAX;
			m_sAttr.iSpeedMin = SPEED_180_MIN;
			m_sAttr.dAngleStep = ANGLE_RESOLV_2000_3HZ;
			m_sAttr.dCirclePoints = CICRLE_MAX_2000_3HZ;

		}
		else if (m_strLidarModel == D2CE)
		{
			if (m_sAttr.dAngleStep != ANGLE_RESOLV_2000_3HZ)
			{
				m_bCheckSpeed = true;
				m_u64StartTimeCheckSpeed = HCHead::getCurrentTimestampUs();
			}

			m_sAttr.iSpeedMax = SPEED_180_MAX;
			m_sAttr.iSpeedMin = SPEED_180_MIN;
			m_sAttr.dAngleStep = ANGLE_RESOLV_2000_3HZ;
			m_sAttr.dCirclePoints = CICRLE_MAX_2000_3HZ;

		}
		
	}
	else
	{
		if (m_strLidarModel == X2YE)
		{
			if (m_sAttr.dAngleStep != ANGLE_RESOLV_NARWAL_NOR)
			{
				m_bCheckSpeed = true;
				m_u64StartTimeCheckSpeed = HCHead::getCurrentTimestampUs();
			}


			m_sAttr.iSpeedMax = SPEED_315_MAX;
			m_sAttr.iSpeedMin = SPEED_315_MIN;
			m_sAttr.dAngleStep = ANGLE_RESOLV_NARWAL_NOR;
			m_sAttr.dCirclePoints = CICRLE_MAX_NARWAL_NOR;

			UCHAR u8Temp[5] = { 0xAA,0x55,0xF2,0x0D };
			m_serial.writeData2(u8Temp, 4);
		}
		else if (m_strLidarModel == X2BF)
		{
			if (m_sAttr.dAngleStep != ANGLE_RESOLV_3000)
			{
				m_bCheckSpeed = true;
				m_u64StartTimeCheckSpeed = HCHead::getCurrentTimestampUs();
			}


			m_sAttr.iSpeedMax = SPEED_360_MAX;
			m_sAttr.iSpeedMin = SPEED_360_MIN;
			m_sAttr.dAngleStep = ANGLE_RESOLV_3000;
			m_sAttr.dCirclePoints = CICRLE_MAX_3000;
		}
		else if (m_strLidarModel == X2MF)
		{
			if (m_sAttr.dAngleStep != ANGLE_RESOLV_2000_6HZ)
			{
				m_bCheckSpeed = true;
				m_u64StartTimeCheckSpeed = HCHead::getCurrentTimestampUs();
			}

			m_sAttr.iSpeedMax = SPEED_360_MAX;
			m_sAttr.iSpeedMin = SPEED_360_MIN;
			m_sAttr.dAngleStep = ANGLE_RESOLV_2000_6HZ;
			m_sAttr.dCirclePoints = CICRLE_MAX_2000_6HZ;

		}
		else if (m_strLidarModel == D2PD)
		{
			if (m_sAttr.dAngleStep != ANGLE_RESOLV_2000)
			{
				m_bCheckSpeed = true;
				m_u64StartTimeCheckSpeed = HCHead::getCurrentTimestampUs();
			}

			m_sAttr.iSpeedMax = SPEED_312_MAX;
			m_sAttr.iSpeedMin = SPEED_312_MIN;
			m_sAttr.dAngleStep = ANGLE_RESOLV_2000;
			m_sAttr.dCirclePoints = CICRLE_MAX_2000;

		}
		else if (m_strLidarModel == D2AE)
		{
			if (m_sAttr.dAngleStep != ANGLE_RESOLV_2000)
			{
				m_bCheckSpeed = true;
				m_u64StartTimeCheckSpeed = HCHead::getCurrentTimestampUs();
			}

			m_sAttr.iSpeedMax = SPEED_312_MAX;
			m_sAttr.iSpeedMin = SPEED_312_MIN;
			m_sAttr.dAngleStep = ANGLE_RESOLV_2000;
			m_sAttr.dCirclePoints = CICRLE_MAX_2000;

		}
		else if (m_strLidarModel == A200)
		{
			if (m_sAttr.dAngleStep != ANGLE_RESOLV_2000)
			{
				m_bCheckSpeed = true;
				m_u64StartTimeCheckSpeed = HCHead::getCurrentTimestampUs();
			}

			m_sAttr.iSpeedMax = SPEED_312_MAX;
			m_sAttr.iSpeedMin = SPEED_312_MIN;
			m_sAttr.dAngleStep = ANGLE_RESOLV_2000;
			m_sAttr.dCirclePoints = CICRLE_MAX_2000;

		}
		else if (m_strLidarModel == A201)
		{
			if (m_sAttr.dAngleStep != ANGLE_RESOLV_2000)
			{
				m_bCheckSpeed = true;
				m_u64StartTimeCheckSpeed = HCHead::getCurrentTimestampUs();
			}

			m_sAttr.iSpeedMax = SPEED_312_MAX;
			m_sAttr.iSpeedMin = SPEED_312_MIN;
			m_sAttr.dAngleStep = ANGLE_RESOLV_2000;
			m_sAttr.dCirclePoints = CICRLE_MAX_2000;

		}
		else if (m_strLidarModel == D2AG)
		{
			if (m_sAttr.dAngleStep != ANGLE_RESOLV_2000)
			{
				m_bCheckSpeed = true;
				m_u64StartTimeCheckSpeed = HCHead::getCurrentTimestampUs();
			}

			m_sAttr.iSpeedMax = SPEED_312_MAX;
			m_sAttr.iSpeedMin = SPEED_312_MIN;
			m_sAttr.dAngleStep = ANGLE_RESOLV_2000;
			m_sAttr.dCirclePoints = CICRLE_MAX_2000;

		}
		else if (m_strLidarModel == D2CE)
		{
			if (m_sAttr.dAngleStep != ANGLE_RESOLV_2000)
			{
				m_bCheckSpeed = true;
				m_u64StartTimeCheckSpeed = HCHead::getCurrentTimestampUs();
			}

			m_sAttr.iSpeedMax = SPEED_312_MAX;
			m_sAttr.iSpeedMin = SPEED_312_MIN;
			m_sAttr.dAngleStep = ANGLE_RESOLV_2000;
			m_sAttr.dCirclePoints = CICRLE_MAX_2000;

		}
	}
}

UINT16 HCLidar::getPointCloudLatticePeak(int dist_mm, UINT16 peak)
{
	if (peak == 0)
		return peak;

	if (m_sPackUID.u16A1 == 0xFFFF || m_sPackUID.u16B1 == 0xFFFF
		|| m_sPackUID.u16A2 == 0xFFFF || m_sPackUID.u16B2 == 0xFFFF)
	{
		return peak;
	}

	if (m_sPackUID.u16A1 == 0 && m_sPackUID.u16B1 == 0
		&& m_sPackUID.u16A2 == 0 && m_sPackUID.u16B2 == 0)
	{
		return peak;
	}

	if (isCheckMaxPeakLattice(m_sPackUID.u16A1, m_sPackUID.u16B1, m_sPackUID.u16A2,m_sPackUID.u16B2, dist_mm, peak))
	{
		return PC_GRAY_IS_LATTICE;
	}
	else
		return 0;
}

bool HCLidar::isCheckMaxPeakLattice(int a1, int b1, int a2, int b2, int dist_mm, UINT16 peak)
{
	if (a1 == 0 || b1 == 0 || b2 == 0)
	{
		return false;
	}
	double a1_tmp = (double)a1 / 1000.00;
	double a2_tmp = (double)a2 / 1000.00;

	double newpeak = 0;
	if (dist_mm >= 650)
	{
		newpeak = dist_mm * -a2_tmp + b2;
	}
	else if (dist_mm > 250 && dist_mm < 650)   //.300
	{
		newpeak = dist_mm * -a1_tmp + b1;
	}

	if (newpeak > 0 && newpeak < peak)
	{
		return true;
	}
	return false;
}

void HCLidar::compensateDistSubsection(tsSubsection& subsection_para, UINT16 & dist)
{
	if (!m_bComp)
		return;

	if (dist == 0)
	{
		return;
	}

	if (subsection_para.n1_org == 0xFFFFFFFF || subsection_para.n2_org == 0xFFFFFFFF)
	{
		return;
	}

	if (subsection_para.n1_org == 0 && subsection_para.n2_org == 0)
	{
		return;
	}

	double dist_s;
	double dist_temp = dist / 1000.0;

	//分段距离重构
	double cx = subsection_para.n1_org / dist_temp - subsection_para.n2_org;

	if (dist_temp > subsection_para.ulower_0 && dist_temp <= subsection_para.uupper_0)
	{
		dist_s = subsection_para.n1_subsection_0 / (cx + subsection_para.n2_subsection_0);
	}
	else if (dist_temp > subsection_para.ulower_1 && dist_temp <= subsection_para.uupper_1)
	{
		dist_s = subsection_para.n1_subsection_1 / (cx + subsection_para.n2_subsection_1);
	}
	else if (dist_temp > subsection_para.ulower_2 && dist_temp <= subsection_para.uupper_2)
	{
		dist_s = subsection_para.n1_subsection_2 / (cx + subsection_para.n2_subsection_2);
	}
	else if (dist_temp > subsection_para.ulower_3 && dist_temp <= subsection_para.uupper_3)
	{
		dist_s = subsection_para.n1_subsection_3 / (cx + subsection_para.n2_subsection_3);
	}
	else if (dist_temp > subsection_para.ulower_4 && dist_temp <= subsection_para.uupper_4 + 2)
	{
		dist_s = subsection_para.n1_subsection_4 / (cx + subsection_para.n2_subsection_4);
	}

	if (dist_temp <= subsection_para.uupper_4 + 2)
	{
		dist = dist_s * 1000;
	}
}

bool HCLidar::isYJCalGood(UCHAR* pData,int iLen)
{
	if (iLen < 3)
		return false;

	UCHAR u8Temp = pData[0];
	for (int i = 1; i < iLen - 1; i++)
	{
		u8Temp ^= pData[i];
	}

	if (u8Temp == pData[iLen - 1])
		return true;
	else
		return false;
}

bool HCLidar::isYJCalGood(std::vector<UCHAR>& lstBuff, int iLen)
{
	if (iLen < 3)
		return false;

	UCHAR u8Temp = (UCHAR)lstBuff.at(0);
	for (int i = 1; i < iLen - 1; i++)
	{
		u8Temp ^= (UCHAR)lstBuff.at(i);
	}

	if (u8Temp == (UCHAR)lstBuff.at(iLen - 1))
		return true;
	else
		return false;
}


// CRC-16/XMODEM calculation
unsigned short HCLidar::crc16Xmodem(unsigned char* ptr, int len)
{
	unsigned int i;
	unsigned short crc = 0x0000;
	const int size = len;
    int index{ 0 };
	while (len--)
	{
		if (index == (size - 4) || index == (size - 3)) {
            index++;
            continue;
        }
		crc ^= (unsigned short)(*(ptr + index)) << 8;
        index++;
        for (i = 0; i < 8; ++i) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
	}
	return crc;
}

// Send command and wait for response
bool HCLidar::sendCommandAndWaitAck(const std::vector<UCHAR>& cmd, std::vector<UCHAR>& response, int timeout_ms)
{
	const int maxRetries = 3;
	
	for (int retry = 0; retry < maxRetries; retry++)
	{
		// Clear response buffer
		response.clear();

		// Flush receive buffer before sending
		m_serial.flushReceiver();

		// Send command
		if (m_serial.writeData2((UCHAR*)cmd.data(), cmd.size()) != cmd.size())
		{
			LOG_ERROR("Failed to send command (retry %d/%d)\n", retry + 1, maxRetries);
			if (retry < maxRetries - 1)
			{
				std::this_thread::sleep_for(std::chrono::milliseconds(100));
				continue;
			}
			return false;
		}
	
		// Wait for response with header (A5 A5) and tail (5A 5A) parsing
		UINT64 startTime = HCHead::getCurrentTimestampMs();
		UCHAR readBuffer[2048];
		bool headerFound = false;
		
		while (HCHead::getCurrentTimestampMs() - startTime < timeout_ms)
		{
			int bytesRead = m_serial.readData(readBuffer, sizeof(readBuffer), 10);
			if (bytesRead > 0)
			{
				for (int i = 0; i < bytesRead; i++)
				{
					response.push_back(readBuffer[i]);
					
					// Check for header (A5 A5)
					if (!headerFound)
					{
						if (response.size() >= 2)
						{
							size_t len = response.size();
							if (response[len - 2] == 0xA5 && response[len - 1] == 0xA5)
							{
								headerFound = true;
								// Clear buffer and keep only the header
								std::vector<UCHAR> temp;
								temp.push_back(0xA5);
								temp.push_back(0xA5);
								response.swap(temp);
							}
						}
					}
					else
					{
						// Header found, now check for tail (5A 5A)
						if (response.size() >= 2)
						{
							size_t len = response.size();
							if (response[len - 2] == 0x5A && response[len - 1] == 0x5A)
							{
								// Complete packet found
								return true;
							}
						}
					}
				}
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}

		LOG_ERROR("Timeout waiting for response (retry %d/%d)\n", retry + 1, maxRetries);
		
		// Wait before retry
		if (retry < maxRetries - 1)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(200));
		}
	}

	LOG_ERROR("Failed after %d retries\n", maxRetries);
	return false;
}

// Query device info (0x70)
bool HCLidar::queryDeviceInfo(std::string& model, std::string& version, int& mode)
{
	// Request: A5 A5 70 00 00 CC 19 5A 5A
	std::vector<UCHAR> cmd = { 0xA5, 0xA5, 0x70, 0x00, 0x00, 0xCC, 0x19, 0x5A, 0x5A };
	std::vector<UCHAR> response;

	if (!sendCommandAndWaitAck(cmd, response, 1000))
	{
		LOG_ERROR("Failed to query device info\n");
		return false;
	}

	// Response format: A5 A5 70 [length] [model(4 bytes)] [version(2 bytes)] [mode(1 byte)] [crc(2 bytes)] 5A 5A
	if (response.size() < 12 || response[0] != 0xA5 || response[1] != 0xA5 || response[2] != 0x70)
	{
		LOG_ERROR("Invalid device info response\n");
		return false;
	}

	// Extract model (4 bytes)
	model.clear();
	for (int i = 4; i < 8 && i < response.size(); i++)
	{
		model += (char)response[i];
	}

	// Extract version (2 bytes)
	char versionStr[16] = { 0 };
	if (response.size() >= 10)
	{
		sprintf(versionStr, "%d.%02d", response[8], response[9]);
		version = versionStr;
	}

	// Extract mode (1 byte)
	if (response.size() >= 11)
	{
		mode = response[10];
	}

	LOG_INFO("Device info - Model: %s, Version: %s, Mode: %d\n", model.c_str(), version.c_str(), mode);
	return true;
}

// Jump to boot mode (0xFA)
bool HCLidar::jumpToBootMode()
{
	// Request: A5 A5 54 FA 00 00 01 00 00 00 00 00 4F 5A 5A
	std::vector<UCHAR> cmd = { 0xA5, 0xA5, 0x54, 0xFA, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x4F, 0x5A, 0x5A };
	std::vector<UCHAR> response;

	if (!sendCommandAndWaitAck(cmd, response, 500))
	{
		LOG_ERROR("Failed to send jump to boot command\n");
		return false;
	}

	// Ack: A5 A5 54 FA 00 00 01 00 00 00 00 00 4F 5A 5A
	if (response.size() >= 14 && response[0] == 0xA5 && response[1] == 0xA5 && 
		response[2] == 0x54 && response[3] == 0xFA)
	{
		LOG_INFO("Jump to boot mode success\n");
		return true;
	}

	LOG_ERROR("Jump to boot mode failed\n");
	return false;
}

// Send firmware data packet (0x73)
bool HCLidar::sendFirmwarePacket(UINT8 packetIndex, const UINT8* data, int dataLen, int& nextExpectedIndex)
{
	// Max data length is 2048
	if (dataLen > 2048)
	{
		LOG_ERROR("Data length exceeds maximum (2048)\n");
		return false;
	}

	// Calculate total packet length: header(5) + index(1) + reserved(4) + data + crc(2) + tail(2)
	int totalLen = 5 + 1  + 4 + dataLen + 2 + 2;
	std::vector<UCHAR> packet(totalLen);

	// Header: A5 A5
	packet[0] = 0xA5;
	packet[1] = 0xA5;

	// Command: 0x73
	packet[2] = 0x73;

	// Length: data length + 5 (state + index + reserved)
	UINT16 len = dataLen + 5;
	packet[3] = (UINT8)(len & 0xFF);
	packet[4] = (UINT8)((len >> 8) & 0xFF);


	// Packet index
	packet[5] = packetIndex;

	// Reserved: 00 00 00 00
	packet[6] = 0x00;
	packet[7] = 0x00;
	packet[8] = 0x00;
	packet[9] = 0x00;

	// Data
	memcpy(&packet[10], data, dataLen);

	// Tail: 5A 5A
	packet[totalLen - 2] = 0x5A;
	packet[totalLen - 1] = 0x5A;

	// Calculate CRC for all bytes except CRC itself and tail
	unsigned short crc = crc16Xmodem((unsigned char*)packet.data(), totalLen);
	packet[10 + dataLen] = (UINT8)(crc & 0xFF);
	packet[10 + dataLen + 1] = (UINT8)((crc >> 8) & 0xFF);

	

	// Send packet and wait for response
	std::vector<UCHAR> response;
	if (!sendCommandAndWaitAck(packet, response, 2000))
	{
		LOG_ERROR("Failed to send firmware packet %d\n", packetIndex);
		return false;
	}

	// Response format: A5 A5 73 06 01 [next_index] 00 00 00 00 00 [crc] 5A 5A
	if (response.size() >= 14 && response[0] == 0xA5 && response[1] == 0xA5 && 
		response[2] == 0x73 && response[3] == 0x06)
	{
		// Check status
		if (response[4] == 0x01)
		{
			// Success - get next expected index
			nextExpectedIndex = response[5]+1;
			LOG_INFO("Firmware packet %d sent successfully, next expected: %d\n", packetIndex, nextExpectedIndex);
			return true;
		}
		else
		{
			// Error
			LOG_ERROR("Firmware packet %d failed with status: %d\n", packetIndex, response[4]);
			return false;
		}
	}

	LOG_ERROR("Invalid firmware packet response\n");
	return false;
}

// OTA upgrade interface
bool HCLidar::otaUpgrade(const char* firmwareFilePath)
{
	LOG_INFO("Starting OTA upgrade, firmware file: %s\n", firmwareFilePath);

	// Open firmware file
	std::ifstream firmwareFile(firmwareFilePath, std::ios::binary | std::ios::ate);
	if (!firmwareFile.is_open())
	{
		LOG_ERROR("Failed to open firmware file: %s\n", firmwareFilePath);
		return false;
	}

	// Get file size
	std::streampos fileSize = firmwareFile.tellg();
	firmwareFile.seekg(0, std::ios::beg);

	// Read firmware data
	std::vector<UCHAR> firmwareData(fileSize);
	firmwareFile.read((char*)firmwareData.data(), fileSize);
	firmwareFile.close();

	LOG_INFO("Firmware file size: %d bytes\n", (int)fileSize);
	if(fileSize<100)
		return false;


	// Step 1: Query current device info
	std::string model, version;
	int currentMode = 0;

	LOG_INFO("Device in app mode, jumping to boot mode...\n");
	if (!jumpToBootMode())
	{
		LOG_ERROR("Failed to jump to boot mode\n");
		std::this_thread::sleep_for(std::chrono::milliseconds(200));
		LOG_INFO("Try get device info\n");
		if (!queryDeviceInfo(model, version, currentMode))
		{
			LOG_ERROR("Failed to query device info,not in boot mode\n");
			return false;
		}
		
	}
	else
	{
		// Wait for device to reboot into boot mode
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));

		// Verify we're in boot mode
		if (!queryDeviceInfo(model, version, currentMode))
		{
			LOG_ERROR("Failed to query device info after jump\n");
			return false;
		}
	}

	if (currentMode != 0x01) // boot mode
	{
		LOG_ERROR("Device not in boot mode after jump\n");
		return false;
	}


	// Step 3: Send firmware packets
	const int MAX_PACKET_SIZE = 512;
	int packetIndex = 0;
	int bytesSent = 0;
	int totalPackets = (firmwareData.size() + MAX_PACKET_SIZE - 1) / MAX_PACKET_SIZE;

	LOG_INFO("Starting to send %d firmware packets...\n", totalPackets);
	if(totalPackets>255)
	{
		LOG_ERROR("File size too large\n");
		return false;
	}

	while (bytesSent < firmwareData.size())
	{
		int remaining = firmwareData.size() - bytesSent;
		int packetSize = (remaining > MAX_PACKET_SIZE) ? MAX_PACKET_SIZE : remaining;

		int nextExpected = 0;
		if (!sendFirmwarePacket((UINT8)packetIndex, &firmwareData[bytesSent], packetSize, nextExpected))
		{
			LOG_ERROR("Failed to send packet %d\n", packetIndex);
			return false;
		}

		// Verify we got the expected next index
		if (nextExpected != (packetIndex + 1) % 256)
		{
			LOG_ERROR("Unexpected next index: expected %d, got %d\n", (packetIndex + 1) % 256, nextExpected);
			return false;
		}

		bytesSent += packetSize;
		packetIndex++;

		// Progress logging
		if (packetIndex % 10 == 0 || bytesSent >= firmwareData.size())
		{
			LOG_INFO("OTA progress: %d/%d packets (%d%%)\n", packetIndex, totalPackets, 
				(int)((bytesSent * 100) / firmwareData.size()));
		}
	}

	LOG_INFO("All firmware packets sent successfully\n");

	// Step 4: Wait for device to reboot and verify
	LOG_INFO("Waiting for device to reboot...\n");
	std::this_thread::sleep_for(std::chrono::milliseconds(3000));

	// Step 5: Query device info again to verify upgrade
	if (!queryDeviceInfo(model, version, currentMode))
	{
		LOG_ERROR("Failed to verify upgrade\n");
		return false;
	}

	if (currentMode == 0x02) // Should be in app mode after successful upgrade
	{
		LOG_INFO("OTA upgrade completed successfully!\n");
		return true;
	}
	else
	{
		LOG_ERROR("Device not in app mode after upgrade\n");
		return false;
	}
}

}
