/*=Plus=header=begin======================================================
Program: Plus
Copyright (c) Laboratory for Percutaneous Surgery. All rights reserved.
See License.txt for details.
=========================================================Plus=header=end*/
#include "vtkPlusPolhemus.h"

#include "PlusConfigure.h"
#include "PlusConfigure.h"
#include "vtkMatrix4x4.h"
#include "vtkObjectFactory.h"
#include "vtkPlusDataSource.h"
#include "vtkTransform.h"
#include "vtkXMLDataElement.h"
#include "vtksys/SystemTools.hxx"
#include <sstream>

#include <string.h>
#include <chrono>
#include <queue>
#include <condition_variable>
#include <algorithm>



// #define PrintPnoRecord
// #define debug

#define RESET   "\033[0m"
#define RED     "\033[31m"      /* Red */
#define GREEN   "\033[32m"      /* Green */
#define YELLOW  "\033[33m"      /* Yellow */

vtkStandardNewMacro(vtkPlusPolhemus);

//-------------------------------------------------------------------------
vtkPlusPolhemus::vtkPlusPolhemus()
{
  std::cout << "Constructor Called" << std::endl;
  keep_reading_usb = 0;
  is_continuous = 0;
  cmd_queue.init(&keep_reading_usb);
  pno_queue.init(&is_continuous);
  
  // Maximum number of sensors that can be connected 
  max_sensors = 8;

  // Create array to store sensors values. This array is creted with maximum size with ports-1 as index. The ports not used are default value
  data_storage = new Pose[max_sensors];

  // Create array to see which sensors are connected. Intialize it to all false
  sensor_state = new bool[max_sensors];
  std::fill(sensor_state, sensor_state + max_sensors, false);

  // Max size of the entire packet returned from the port which includes all sensor values
  resp_size = sizeof(uint32_t) * 11 + sizeof(SENFRAMEDATA) * 16 + CRC_SIZE; // max no. of sensors

  // Regularly poll for changes
  this->StartThreadForInternalUpdates = true;
  this->AcquisitionRate = 50;

  this->FTTMode = 0;
}

//-------------------------------------------------------------------------
vtkPlusPolhemus::~vtkPlusPolhemus()
{
  std::cout << "Destructor Called" << std::endl;
  if (this->Recording)
  {
    this->StopRecording();
  }
  delete data_storage;
  delete sensor_state;
}

//-------------------------------------------------------------------------
PlusStatus vtkPlusPolhemus::InternalConnect()
{
  std::cout << "Internal Connect Called" << std::endl;

  // Connect to the port Viper EM is connected to
  if (this->viper.usb_connect() != 0)
  {
    std::cout << RED << "\n\nError connecting to Viper over USB\nNode communication for EM tracker failed\n\n" << RESET << std::endl; 
    return PLUS_FAIL;
  }
  else
  {
    #ifdef debug
    std::cout << GREEN << "\n\nConnected to Viper over USB\nNode communication for EM tracker successful\n\n" << RESET << std::endl;
    #endif
  }

  // Initialize the number of sensors. Also checks if there are less sensors than needed
  uint32_t rvNum = numSensors(&viper);
  if(rvNum != 0){
    std::cout << RED << "Error reading number of sensors. CODE - " << rvNum << RESET << std::endl;
    return PLUS_FAIL;
  }
  else if(std::count(sensor_state, sensor_state + max_sensors, true) > this->num_sensors){
    std::cout << RED << "Error number of sensors in XML is more than number of sensors connected" << RESET << std::endl;
    return PLUS_FAIL;
  }
  else{
    #ifdef debug
    std::cout << GREEN << "Number of sensors connected- " << this->num_sensors << RESET << std::endl;
    #endif
  }

  // Setting FTT Mode to on
  CEnumCfg fttcfg;
  fttcfg.Fill(FTTMode);

  // Setting units to centimeters and quaternion
  CUnitsCfg ucfg;
  ucfg.Fill(POS_CM, ORI_QUATERNION);

  // Setting Auto-Hemisphere
  CHemisphereCfg hemicfg;
  hemicfg.Fill(hemicfg.POS_X, true, true); // hemi, autohemi, autotrack

  uint32_t ftt_sens = -1;
  // Configure FTT mode for all sensors
  if (CmdFtt(fttcfg,&viper, ftt_sens)==0)
  {
    cout << "CmdFTT error" << endl;
    //return 1;
  }
  else
  {
    cout << "FTT Mode - " << FTTMode << endl;
  }

  // Configure Units
  if (CmdUnits(ucfg, &viper) == 0)
  {
    cout << "CmdUnits error" << endl;
    // return 1;
  }
  else
  {
    cout << "Units set to CM, Quaternions " << endl;
  }

  if (CmdHemi(hemicfg, &viper) == 0)
  {
    cout << "Hemisphere Setting error" << endl;
  }
  else
  {
    cout << "Hemisphere Set" << endl;
  }

  return PLUS_SUCCESS;
}

//-------------------------------------------------------------------------
PlusStatus vtkPlusPolhemus::InternalDisconnect()
{
  std::cout << "Internal Disconnect Called" << std::endl;
  
  // Disconnect the usb device
  this->viper.usb_disconnect();
  
  return PLUS_SUCCESS;
}

//-------------------------------------------------------------------------
PlusStatus vtkPlusPolhemus::InternalStartRecording()
{
  std::cout << "Internal Start Recording Called" << std::endl;
  return PLUS_SUCCESS;
}

//-------------------------------------------------------------------------
PlusStatus vtkPlusPolhemus::InternalStopRecording()
{
  std::cout << "Internal Stop Recording Called" << std::endl;
  return PLUS_SUCCESS;
}

//-------------------------------------------------------------------------
PlusStatus vtkPlusPolhemus::InternalUpdate()
{
  #ifdef debug
  std::cout << "Internal Update Called" << std::endl;
  #endif
  // Get information about the sensors
  uint32_t rvRead = readSensors(&viper);
  if(rvRead != 0){
    std::cout << RED << "Error Reading from the Sensors. CODE - " << rvRead << RESET << std::endl;
    return PLUS_FAIL;
  }
  else{
    #ifdef debug
    std::cout << GREEN << "Success Reading from the Sensors" << RESET << std::endl;
    #endif
  }

  // Iterate through the entire data_storage array and only check the sensors needed in the XML
  for (int i = 0; i < max_sensors; ++i)
  {
    if (sensor_state[i])
    {
      // Data structure used to store each sensor values and return it
      vtkSmartPointer<vtkMatrix4x4> mToolToTracker = vtkSmartPointer<vtkMatrix4x4>::New();
      
      // Create a transformation matrix from quarternions and position
      PoseToTransMat(data_storage[i], mToolToTracker);

      // The name of the port is the same as the port number
      std::ostringstream toolPortName;
      toolPortName << i+1;

      // Get the tool information using the port number
      vtkPlusDataSource *tool = NULL;
      if (this->GetToolByPortName(toolPortName.str().c_str(), tool) != PLUS_SUCCESS)
      {
        std::cout << RED << "Unable to find tool on port: " << toolPortName.str() << RESET << std::endl;
        continue;
      }

      // Time stamp for the reading
      const double unfilteredTimestamp = vtkIGSIOAccurateTimer::GetSystemTime();

      // Devices has no frame numbering, so just auto increment tool frame number
      unsigned long frameNumber = tool->GetFrameNumber() + 1;

      // Send the data to the tool Id
      this->ToolTimeStampedUpdate(tool->GetSourceId(), mToolToTracker, TOOL_OK, frameNumber, unfilteredTimestamp);
    }
  }
  return PLUS_SUCCESS;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusPolhemus::ReadConfiguration(vtkXMLDataElement *rootConfigElement)
{
  std::cout << "Read Configuration Called" << std::endl;

  XML_FIND_DEVICE_ELEMENT_REQUIRED_FOR_READING(deviceConfig, rootConfigElement);

  XML_READ_SCALAR_ATTRIBUTE_OPTIONAL(int, FTTMode, deviceConfig);

  XML_FIND_NESTED_ELEMENT_REQUIRED(dataSourcesElement, deviceConfig, "DataSources");

  for (int toolIndex = 0; toolIndex < dataSourcesElement->GetNumberOfNestedElements(); toolIndex++)
  {
    vtkXMLDataElement *toolDataElement = dataSourcesElement->GetNestedElement(toolIndex);
    if (STRCASECMP(toolDataElement->GetName(), "DataSource") != 0)
    {
      // if this is not a data source element, skip it
      continue;
    }

    if (toolDataElement->GetAttribute("Type") != NULL && STRCASECMP(toolDataElement->GetAttribute("Type"), "Tool") != 0)
    {
      // if this is not a Tool element, skip it
      continue;
    }

    const char *portName = toolDataElement->GetAttribute("PortName");
    if (portName == NULL)
    {
      std::cout << RED << "Cannot set sensor-specific parameters: tool portname is undefined" << RESET << std::endl;
      continue;
    }

    #ifdef debug
    std::cout << portName << "- Port Name" << std::endl;
    #endif

    // Convert portName to an integer and populate the sensor_state array to reflect which port are required
    try
    {
      int portNumber = std::stoi(portName);
      #ifdef debug
      std::cout << "Port number: " << portNumber << std::endl;
      #endif
      sensor_state[portNumber-1] = true;
    }
    catch (const std::exception &e)
    {
      std::cout << "Error: PortName should be an integer between 1-8" << std::endl;
      return PLUS_FAIL;
    }
  }
  return PLUS_SUCCESS;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusPolhemus::WriteConfiguration(vtkXMLDataElement *rootConfigElement)
{
  std::cout << "Write Configuration Called" << std::endl;
  XML_FIND_DEVICE_ELEMENT_REQUIRED_FOR_WRITING(trackerConfig, rootConfigElement);

  trackerConfig->SetIntAttribute("FTTMode", this->FTTMode());
	
  return PLUS_SUCCESS;
}

//----------------------------------------------------------------------------
uint32_t vtkPlusPolhemus::CalcCrc16(uint8_t *b, uint32_t len)
{
  uint32_t crc = 0;
  while (len--)
    crc = crctable[(crc ^ *b++) & 0xff] ^ (crc >> 8);
  return crc;
}

//----------------------------------------------------------------------------
// query for single frame of data. frompolhemus
uint32_t vtkPlusPolhemus::readSingleFrame(viper_usb *pvpr)
{
  const uint32_t HDR_END_LOC = sizeof(SEUCMD_HDR);
  uint32_t crc, br, num_sens, i;

  uint32_t cmd_size = sizeof(SEUCMD_HDR) + CRC_SIZE;
  uint8_t *cmd_pkg = new uint8_t[cmd_size];

  resp_pkg = new uint8_t[resp_size];

  SEUCMD_HDR *phdr = (SEUCMD_HDR *)cmd_pkg;
  memset(cmd_pkg, 0, cmd_size);
  phdr->preamble = VIPER_CMD_PREAMBLE;
  phdr->size = cmd_size - 8; // preamble and size not incl in size

  // Single Frame
  phdr->seucmd.cmd = CMD_SINGLE_PNO;
  phdr->seucmd.action = CMD_ACTION_GET;
  crc = CalcCrc16(cmd_pkg, cmd_size - CRC_SIZE);
  memcpy(cmd_pkg + HDR_END_LOC, &crc, CRC_SIZE);
  pvpr->usb_send_cmd(cmd_pkg, cmd_size);
  br = cmd_queue.wait_and_pop(resp_pkg, resp_size);
  delete[] cmd_pkg;

  // make sure you get an ack
  if (*(uint32_t *)(resp_pkg + 16) != CMD_ACTION_ACK)
    return 2;

  // check CRC
  crc = CalcCrc16(resp_pkg, br - CRC_SIZE);
  if (crc != *(uint32_t *)(resp_pkg + br - CRC_SIZE))
  {
    delete[] resp_pkg;
    return 3;
  }

  return 0;
}

//----------------------------------------------------------------------------
// get umber of sensors connected.
uint32_t vtkPlusPolhemus::numSensors(viper_usb *pvpr)
{
  readUsbData(&viper);
  uint32_t returnVal = readSingleFrame(&viper);
  if (returnVal != 0)
  {
    return returnVal;
  }

  num_sensors = *(uint32_t *)(resp_pkg + 40);
  delete[] resp_pkg;
  return 0;
}

//----------------------------------------------------------------------------
// get umber of sensors connected.
uint32_t vtkPlusPolhemus::readSensors(viper_usb *pvpr)
{
  readUsbData(&viper);
  uint32_t returnVal = readSingleFrame(&viper);
  if (returnVal != 0)
  {
    return returnVal;
  }

  if(checkSensorChange()){
    return 1;
  }

  SENFRAMEDATA *pfd;

  pfd = (SENFRAMEDATA *)(resp_pkg + 44);
  for (int i = 0; i < num_sensors; i++){
    int sensor_index = ((pfd + i)->SFinfo.bfSnum & 0xff);
    if(!sensor_state[sensor_index]){
      std::cout << YELLOW << "Port - " << sensor_index+1 << " not initialized" << RESET << std::endl;
      continue;
    }
    #ifdef debug
    std::cout << "Read from Port - " << sensor_index+1 << std::endl;
    #endif
    data_storage[sensor_index].x = (pfd + i)->pno.pos[0];
    data_storage[sensor_index].y = (pfd + i)->pno.pos[1];
    data_storage[sensor_index].z = (pfd + i)->pno.pos[2];
    data_storage[sensor_index].quatW = (pfd + i)->pno.ori[0];
    data_storage[sensor_index].quatX = (pfd + i)->pno.ori[1];
    data_storage[sensor_index].quatY = (pfd + i)->pno.ori[2];
    data_storage[sensor_index].quatZ = (pfd + i)->pno.ori[3];
    printPnoRecord(pfd + i);
  }
  #ifdef debug
  std::cout << " pno record sent" << std::endl;
  #endif
  delete[] resp_pkg;
  return 0;
}

//----------------------------------------------------------------------------
bool vtkPlusPolhemus::checkSensorChange(){
  uint32_t temp_num_sensors = *(uint32_t *)(resp_pkg + 40);

  if(temp_num_sensors != num_sensors){
    std::cout << "Num of Sensor Changed" << std::endl;
    return true;
  }
  return false;
}

//----------------------------------------------------------------------------
void vtkPlusPolhemus::readUsbData(viper_usb *pvpr)
{
  resp_pkg = new uint8_t[resp_size];
  int br;

  uint32_t t_o = 200;
  viper_queue *pqueue;

  br = pvpr->usb_rec_resp(resp_pkg, resp_size);
  if (br)
  {
    if (*(uint32_t *)resp_pkg == VIPER_PNO_PREAMBLE)
    {
      pqueue = &pno_queue;
      t_o = 2;
    }
    else
    {
      pqueue = &cmd_queue;
    }

    pqueue->push(resp_pkg, br);
  }
}

//----------------------------------------------------------------------------
void vtkPlusPolhemus::printPnoRecord(SENFRAMEDATA *pfd)
{
#ifdef PrintPnoRecord

  uint32_t sensor_id = (pfd->SFinfo.bfSnum & 0xff) + 1;
  std::string s = std::to_string(sensor_id);
  auto current_time = std::chrono::system_clock::now();
  std::time_t time_now = std::chrono::system_clock::to_time_t(current_time);

  std::cout << "Sensor: " << s.c_str() << std::endl
            << pfd->pno.pos[0] << "  " << pfd->pno.pos[1] << "  "
            << pfd->pno.pos[2] << "  " << pfd->pno.ori[0] << "  "
            << pfd->pno.ori[1] << "  " << pfd->pno.ori[2] << "  "
            << pfd->pno.ori[3] << std::endl;

#endif
}

int vtkPlusPolhemus::CmdHemi(CHemisphereCfg &hemicfg, viper_usb* pvpr)
{
  int r = 0;
  uint32_t sens = 0;
  CVPcmd cmd;
  cmd.Fill(0, CMD_HEMISPHERE, CMD_ACTION_SET, -1, 0, hemicfg, sizeof(HEMISPHERE_CONFIG));
  cmd.Prepare(g_txbuf, g_ntxcount);


  int nBytes = g_ntxcount;
  uint8_t* pbuf = g_txbuf;

  if (r = pvpr->usb_send_cmd(pbuf,nBytes))
  {
    std::cout << "Write CMD_ACTION_SET failed with error " << std::endl;
  }
  else
  {
    g_nrxcount = RX_BUF_SIZE;
    r = pvpr->usb_rec_resp(g_rxbuf, g_nrxcount);

    if (r == 0)
    {

      CFrameInfo fi(g_rxbuf, g_nrxcount);

      if ((fi.cmd() != CMD_UNITS) || !(fi.IsAck()))
      {
        r = -1;
      }
    }
  }
  cout << r;
  return r;

}

int vtkPlusPolhemus::CmdUnits(CUnitsCfg& ucfg, viper_usb* pvpr)
{
	int r = 0;

	CVPcmd cmd;
	cmd.Fill(0, CMD_UNITS, CMD_ACTION_SET, 0, 0, ucfg, sizeof(UNITS_CONFIG));
	cmd.Prepare(g_txbuf, g_ntxcount);


	int nBytes = g_ntxcount;
	uint8_t* pbuf = g_txbuf;

	if (r = pvpr->usb_send_cmd(pbuf,nBytes))
	{
		std::cout << "Write CMD_ACTION_SET failed with error " << std::endl;
	}
	else
	{
		g_nrxcount = RX_BUF_SIZE;
		r = pvpr->usb_rec_resp(g_rxbuf, g_nrxcount);

		if (r == 0)
		{

			CFrameInfo fi(g_rxbuf, g_nrxcount);

			if ((fi.cmd() != CMD_UNITS) || !(fi.IsAck()))
			{
				r = -1;
			}
		}
	}
	cout << r;
	return r;

}


int vtkPlusPolhemus::CmdFtt(CEnumCfg& fttcfg, viper_usb* pvpr, uint32_t num_sens)
{
	int r = 0;

	CVPcmd cmd;
  // using arg4 = -1 in cmd.Fill() to set all sensors!
	cmd.Fill(0, CMD_FTT_MODE, CMD_ACTION_SET, num_sens,0, fttcfg, sizeof(FTTMODE_CONFIG));
	cmd.Prepare(g_txbuf, g_ntxcount);


	int nBytes = g_ntxcount;
	uint8_t* pbuf = g_txbuf;

	if (r = pvpr->usb_send_cmd(pbuf,nBytes))
	{
		std::cout << "Write CMD_ACTION_SET failed with error " << std::endl;
	}
	else
	{
		g_nrxcount = RX_BUF_SIZE;
		r = pvpr->usb_rec_resp(g_rxbuf, g_nrxcount);

		if (r == 0)
		{

			CFrameInfo fi(g_rxbuf, g_nrxcount);

			if ((fi.cmd() != CMD_UNITS) || !(fi.IsAck()))
			{
				r = -1;
			}
		}
	}
	cout << r;
	return r;

}
