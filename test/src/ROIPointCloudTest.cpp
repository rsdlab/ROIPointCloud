// -*- C++ -*-
/*!
 * @file  ROIPointCloudTest.cpp
 * @brief ROI of PointCloud 
 * @date $Date$
 *
 * $Id$
 */

#include "ROIPointCloudTest.h"

// Module specification
// <rtc-template block="module_spec">
static const char* roipointcloud_spec[] =
  {
    "implementation_id", "ROIPointCloudTest",
    "type_name",         "ROIPointCloudTest",
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
ROIPointCloudTest::ROIPointCloudTest(RTC::Manager* manager)
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
ROIPointCloudTest::~ROIPointCloudTest()
{
}



RTC::ReturnCode_t ROIPointCloudTest::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("ROIPointCloud", m_ROIPointCloudIn);
  
  // Set OutPort buffer
  addOutPort("InROICornar", m_InROICornarOut);
  addOutPort("InPointCloud", m_InPointCloudOut);
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // </rtc-template>
  
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t ROIPointCloudTest::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ROIPointCloudTest::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ROIPointCloudTest::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t ROIPointCloudTest::onActivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t ROIPointCloudTest::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t ROIPointCloudTest::onExecute(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t ROIPointCloudTest::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ROIPointCloudTest::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ROIPointCloudTest::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ROIPointCloudTest::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ROIPointCloudTest::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{
 
  void ROIPointCloudTestInit(RTC::Manager* manager)
  {
    coil::Properties profile(roipointcloud_spec);
    manager->registerFactory(profile,
                             RTC::Create<ROIPointCloudTest>,
                             RTC::Delete<ROIPointCloudTest>);
  }
  
};


