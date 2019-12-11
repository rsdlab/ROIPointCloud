// -*- C++ -*-
/*!
 * @file  ROIPointCloud.cpp
 * @brief ROI of PointCloud 
 * @date $Date$
 *
 * $Id$
 */

#include "ROIPointCloud.h"

// Module specification
// <rtc-template block="module_spec">
static const char* roipointcloud_spec[] =
  {
    "implementation_id", "ROIPointCloud",
    "type_name",         "ROIPointCloud",
    "description",       "ROI of PointCloud ",
    "version",           "1.0.0",
    "vendor",            "Masayuki Fukao, Robot System Design Laboratory, Meijo University",
    "category",          "Pointcloud",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
ROIPointCloud::ROIPointCloud(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_InROICornarIn("InROICornar", m_InROICornar),
    m_InPointCloudIn("InPointCloud", m_InPointCloud),
    m_ROIPointCloudOut("ROIPointCloud", m_ROIPointCloud)

    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
ROIPointCloud::~ROIPointCloud()
{
}



RTC::ReturnCode_t ROIPointCloud::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("InROICornar", m_InROICornarIn);
  addInPort("InPointCloud", m_InPointCloudIn);
  
  // Set OutPort buffer
  addOutPort("ROIPointCloud", m_ROIPointCloudOut);
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // </rtc-template>
  
  return RTC::RTC_OK;
}

RTC::ReturnCode_t ROIPointCloud::onActivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t ROIPointCloud::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t ROIPointCloud::onExecute(RTC::UniqueId ec_id)
{
  //データポートが更新されているかの確認
  if(m_InROICornarIn.isNew()){
    //データポートを読み込む
    m_InROICornarIn.read();
    m_InPointCloudIn.read();
      
    //データポートから点群のサイズの取得
    m_ROIPointCloud.width = m_InPointCloud.width;
    m_ROIPointCloud.height = m_InPointCloud.height;
      
    //出力データのメモリの確保
    m_ROIPointCloud.raw_XYZdata.length(m_InPointCloud.width * m_InPointCloud.height * 3);
    m_ROIPointCloud.raw_RGBdata.length(m_InPointCloud.width * m_InPointCloud.height * 3);
      
    //ROIの作成パート
    for(int i = 0 ; i < m_InPointCloud.height ; i++){
      for(int j = 0 ; j < m_InPointCloud.width ; j++ ){
	int xyzrgbcounter = i * m_InPointCloud.width * 3 + j * 3;
	int depthcounterfront = m_InROICornar.cornerdata[0].y * m_InPointCloud.width * 3 + m_InROICornar.cornerdata[0].x + 2;
	int depthcounterback = m_InROICornar.cornerdata[2].y * m_InPointCloud.width * 3 + m_InROICornar.cornerdata[2].x + 2;
	//コーナー座標から幅、高さのROIを作成
	if(m_InROICornar.cornerdata[0].x < j && j < m_InROICornar.cornerdata[2].x && i < m_InROICornar.cornerdata[2].y && m_InROICornar.cornerdata[0].y < i){
	  //デプス情報の限定
	  if(m_InPointCloud.raw_XYZdata[depthcounterfront] < m_InPointCloud.raw_XYZdata[xyzrgbcounter + 2] && m_InPointCloud.raw_XYZdata[xyzrgbcounter + 2] < m_InPointCloud.raw_XYZdata[depthcounterback]){
	      
	    //データ入力
	    //x : raw_XYZdata[xyzrgbcounter]
	    //y : raw_XYZdata[xyzrgbcounter+1]
	    //z : raw_XYZdata[xyzrgbcounter+2]
	    //r : raw_RGBdata[xyzrgbcounter]
	    //g : raw_RGBdata[xyzrgbcounter+1]
	    //b : raw_RGBdata[xyzrgbcounter+2]
	    //で格納される
	    m_ROIPointCloud.raw_XYZdata[xyzrgbcounter] = m_InPointCloud.raw_XYZdata[xyzrgbcounter ] ;
	    m_ROIPointCloud.raw_XYZdata[xyzrgbcounter + 1] = m_InPointCloud.raw_XYZdata[xyzrgbcounter + 1 ] ;
	    m_ROIPointCloud.raw_XYZdata[xyzrgbcounter + 2] = m_InPointCloud.raw_XYZdata[xyzrgbcounter + 2 ] ;
	    m_ROIPointCloud.raw_RGBdata[xyzrgbcounter] = m_InPointCloud.raw_RGBdata[xyzrgbcounter ] ;
	    m_ROIPointCloud.raw_RGBdata[xyzrgbcounter + 1] = m_InPointCloud.raw_RGBdata[xyzrgbcounter + 1 ] ;
	    m_ROIPointCloud.raw_RGBdata[xyzrgbcounter + 2] = m_InPointCloud.raw_RGBdata[xyzrgbcounter + 2 ] ;

	  }else{
	    //他部分は０代入
	    m_ROIPointCloud.raw_XYZdata[xyzrgbcounter] = 0 ;
	    m_ROIPointCloud.raw_XYZdata[xyzrgbcounter + 1] = 0 ;
	    m_ROIPointCloud.raw_XYZdata[xyzrgbcounter + 2] = 0 ;
	    m_ROIPointCloud.raw_RGBdata[xyzrgbcounter] = 0 ;
	    m_ROIPointCloud.raw_RGBdata[xyzrgbcounter + 1] = 0 ;
	    m_ROIPointCloud.raw_RGBdata[xyzrgbcounter + 2] = 0 ;  
	  }
	}else{
	  //他部分は０代入
	  m_ROIPointCloud.raw_XYZdata[xyzrgbcounter] = 0 ;
	  m_ROIPointCloud.raw_XYZdata[xyzrgbcounter + 1] = 0 ;
	  m_ROIPointCloud.raw_XYZdata[xyzrgbcounter + 2] = 0 ;
	  m_ROIPointCloud.raw_RGBdata[xyzrgbcounter] = 0 ;
	  m_ROIPointCloud.raw_RGBdata[xyzrgbcounter + 1] = 0 ;
	  m_ROIPointCloud.raw_RGBdata[xyzrgbcounter + 2] = 0 ;
	}
      }
    }
      
    m_ROIPointCloudOut.write();
  } 

   
  return RTC::RTC_OK;
}

extern "C"
{
 
  void ROIPointCloudInit(RTC::Manager* manager)
  {
    coil::Properties profile(roipointcloud_spec);
    manager->registerFactory(profile,
                             RTC::Create<ROIPointCloud>,
                             RTC::Delete<ROIPointCloud>);
  }
  
};


