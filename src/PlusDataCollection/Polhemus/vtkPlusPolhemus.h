/*=Plus=header=begin======================================================
Program: Plus
Copyright (c) Laboratory for Percutaneous Surgery. All rights reserved.
See License.txt for details.
=========================================================Plus=header=end*/

#include "vtkPlusDataCollectionExport.h"

#include "vtkPlusDevice.h"

#include "ViperInterface.h"
#include "viper_usb.h"
#include "viper_queue.h"
#include "VPif.h"
#include "Pose.h"
#include "pose_math.h"

#define TX_BUF_SIZE 0x400
#define RX_BUF_SIZE 0x400

/*!
\class vtkPlusPolhemus
\brief Interface for the Polhemus EM tracker

\ingroup PlusLibDataCollection
*/
class vtkPlusDataCollectionExport vtkPlusPolhemus : public vtkPlusDevice
{
public:
  static vtkPlusPolhemus *New();
  vtkTypeMacro(vtkPlusPolhemus, vtkPlusDevice);

  virtual bool IsTracker() const { return true; }

  /*! Connect to device */
  PlusStatus InternalConnect();

  /*! Disconnect from device */
  virtual PlusStatus InternalDisconnect();

  /*! Read main configuration from xml data */
  virtual PlusStatus ReadConfiguration(vtkXMLDataElement *config);

  /*! Write main configuration to xml data */
  virtual PlusStatus WriteConfiguration(vtkXMLDataElement *config);

  /*!
  Get an update from the tracking system and push the new transforms
  to the tools.  This should only be used within vtkTracker.cxx.
  This method is called by the tracker thread.
  */
  PlusStatus InternalUpdate();

  vtkGetMacro(FTTMode, int);

protected:
  vtkPlusPolhemus();
  ~vtkPlusPolhemus();

  vtkSetMacro(FTTMode, int);

  /*!
  Start the tracking system.  The tracking system is brought from its ground state into full tracking mode.
  The device will only be reset if communication cannot be established without a reset.
  */
  PlusStatus InternalStartRecording();

  /*! Stop the tracking system and bring it back to its ground state: Initialized, not tracking, at 9600 Baud. */
  PlusStatus InternalStopRecording();

private: // Definitions.
  enum
  {
    TRANSMITTER_OFF = -1
  };

private: // Functions.
  vtkPlusPolhemus(const vtkPlusPolhemus &);
  void operator=(const vtkPlusPolhemus &);

  // Variables
  int max_sensors;    // Maximum number of sensors that can be connected
  uint32_t resp_size; // Max size of the entire packet returned from the port which includes all sensor values
  int FTTMode;        // For more information on FTT mode, refer to the VIPER User Manual, page 19:
                      // https://www.creact.co.jp/measure/mocap/viper/file/VIPER_User_Manual_URM18PH392-A.pdf

  viper_usb viper;
  uint32_t keep_reading_usb;
  uint32_t is_continuous;
  uint32_t num_sensors;

  

  viper_queue cmd_queue;
  viper_queue pno_queue;
  uint8_t *resp_pkg;



  const uint16_t crctable[256] =

      {
          0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
          0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
          0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
          0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
          0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
          0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
          0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
          0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
          0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
          0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
          0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
          0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
          0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
          0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
          0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
          0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
          0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
          0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
          0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
          0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
          0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
          0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
          0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
          0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
          0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
          0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
          0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
          0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
          0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
          0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
          0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
          0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040};

  const uint32_t CRC_SIZE = sizeof(uint32_t);
  uint32_t CalcCrc16(uint8_t *b, uint32_t len);

  void readUsbData(viper_usb *pvpr);
  void printPnoRecord(SENFRAMEDATA *pfd);
  uint32_t readSingleFrame(viper_usb *pvpr);
  uint32_t numSensors(viper_usb *pvpr);
  uint32_t readSensors(viper_usb *pvpr);
  bool checkSensorChange();

  int CmdUnits(CUnitsCfg& ucfg, viper_usb* pvpr);
  int CmdHemi(CHemisphereCfg &hemicfg, viper_usb* pvpr);
  int CmdFtt(CEnumCfg& fttcfg, viper_usb* pvpr, uint32_t ftt_sens);

  const uint32_t CMD_DELAY = 200;
  uint8_t g_txbuf[TX_BUF_SIZE];
  int g_ntxcount;

  uint8_t g_rxbuf[RX_BUF_SIZE];
  int g_nrxcount;

  // Uninitialized length arrays
  bool *sensor_state; // Create array to see which sensors are connected. True represents connected. Array when initialized is all false
  Pose *data_storage; // Create array to store sensors values. This array is creted with maximum size with ports-1 as index. The ports not used are default value
};
