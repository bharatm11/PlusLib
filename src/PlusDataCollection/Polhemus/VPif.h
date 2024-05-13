// C++ wrappers for Viper Interface objects
/**
*  @file VPif.h
*
*  C++ implementation of ViperInterface.h structures and definitions.
*
*/

#pragma once
#include "VPcmdDefs.h"
#ifndef _WORKING
#include "ViperInterface.h"
#else
#include "ViperInterface-working.h"
#endif
#include <string>
#include <stdint.h>
#include <memory.h>
#include <algorithm>
#include <complex>

#ifndef CLAMP
#define CLAMP(val, min, max) ((val) < (min) ? (min) : ((val) > (max) ? (max) : (val)))
#endif

#ifndef FP_EQ
#define FP_EPSILON 0.00001
#define FP_EQ(x,v) (((v - FP_EPSILON) < x) && (x <( v + FP_EPSILON)))
#endif
#ifndef VPCMD_MAX
#define VPCMD_MAX CMD_MAX;
#endif

//typedef uint32_t VPCRC;
//// appears as the first 64 bytes of all frames to/from the device
//typedef struct _VP_FRAME_HDR
//{
//	uint32_t preamble;	///< 4 bytes that indicate what type of frame follows: 'VPRP' indicates a P&O frame, 'VPRC' indicates a command frame
//	uint32_t size;		///< 32-bit frame size indicates the number of bytes to follow, including terminating CRC.
//}VP_FRAME_HDR;

//                                     24            16                       32
#define MAX_VP_PNO_FRAME_SIZE (sizeof(SEUPNO_HDR) + (SENSORS_PER_SEU * sizeof(SENFRAMEDATA)) + sizeof(uint32_t)) //24+(16*32)+4 540 0x21C 
#define MIN_VP_PNO_FRAME_SIZE (sizeof(SEUPNO_HDR) + (0 *               sizeof(SENFRAMEDATA)) + sizeof(uint32_t)) //24+4 = 28 0x1C
//                                     28            4                       
#define MIN_VP_CMD_FRAME_SIZE (sizeof(SEUCMD_HDR) + sizeof(uint32_t))	// 28+4=32 0x20
#define MAX_VP_CMD_FRAME_SIZE UINT32_MAX // no max command frame size -- it is too variable
#define VP_NAK_FRAME_SIZE     (sizeof(SEUCMD_HDR) + sizeof(NAK_INFO) + sizeof(uint32_t)) //28+180+4=212 0xD4

#define VP_FRAME_PREAMBLE_SIZE (sizeof(uint32_t))
#define VP_FRAME_LEADER_SIZE   (sizeof(VP_FRAME_HDR))  //Size of preamble + size. Same for P&O and Cmd frames 
#define VP_FRAME_CRC_SIZE      (sizeof(uint32_t))      //Size of CRC field at end of all frames.

#define VP_PNO_FRAME_SIZE_FIELD_MIN	(MIN_VP_PNO_FRAME_SIZE - VP_FRAME_LEADER_SIZE) //28-8=20 0x14
#define VP_PNO_FRAME_SIZE_FIELD_MAX	(MAX_VP_PNO_FRAME_SIZE - VP_FRAME_LEADER_SIZE) //124-8=116 0x74

#define VP_CMD_FRAME_SIZE_FIELD_MIN	(MIN_VP_CMD_FRAME_SIZE - VP_FRAME_LEADER_SIZE)
#define VP_CMD_FRAME_SIZE_FIELD_MAX	(MAX_VP_CMD_FRAME_SIZE - VP_FRAME_LEADER_SIZE)

#define RPT_CRLF "\r\n"
#define RPT_INDENT "   "

extern char *RPT_szCRLF;// = RPT_CRLF;
extern char *RPT_szINDENT;// = "";


//! A small-footprint object used to pass around references to complete dev I/O frames
///   without copying and re-copying the frame buffer.  When a frame is received by the host, it is
///   read directly into a buffer owned by CVPdevIO.  From there it is not copied
///   again until the user app retrieves it.   Until that time it is referenced by an instance of
///   CFrameInfo that includes the following information:
///   - A pointer to the beginning of the frame in the CVPdevIO buffer.
///   - The total number of bytes in the frame.  (This is not the *size* field of the frame; it is the
///      count of bytes that came from the IO object from the first byte of the VP_FRAME_HDR to the last
///      of the CRC at the end.
typedef struct _vpFrameInfo
{
	uint8_t * pF;			///< Pointer to buffer location where raw frame bytes are stored 
	uint32_t uiSize;		///< Size of the P&O frame in bytes. Size of the entire buffer.
	uint32_t uiFCountRx;	///< Count of frames received from this dev since the dev was discovered. 
	int32_t iFrameErr;	    ///< Error status of the frame.
	uint64_t ts;			///< Optional timestamp: time since system clock epoch
} vpFrameInfo;

//! C++ class implentation of @ref vpFrameInfo struct.
/// A CFrameInfo object is a small-footprint object used to pass around complete dev I/O frames
///   without copying and re-copying the frame buffer.  When a frame is received by the host, it is
///   read directly into a buffer owned by CVPdevIO.  From there it is not copied
///   again until the user app retrieves it.   Until that time it is referenced by an instance of
///   CFrameInfo that includes the following information:
///   - A pointer to the beginning of the frame in the CVPdevIO buffer.
///   - The total number of bytes in the frame.  (This is not the *size* field of the frame; it is the
///      count of bytes that came from the IO object from the first byte of the VP_FRAME_HDR to the last
///      of the CRC at the end.
class CFrameInfo : public vpFrameInfo
{
public:
	CFrameInfo() { Init(); }
	
	CFrameInfo(uint8_t* p, uint32_t size, uint32_t FC, uint32_t FE)
	{
		Init();
		pF = p; uiSize = size;
		uiFCountRx = FC; 
		iFrameErr = FE;
	}
	
	CFrameInfo(uint8_t* p, uint32_t size, uint32_t fc=0, int32_t FE=0, uint64_t ats=0)
	{
		Init();
		pF = p; uiSize = size; uiFCountRx = fc; iFrameErr = FE; ts = ats;
	}
	
	//CFrameInfo(const CFrameInfo & rv)
	//{
	//	pF = rv.pF;
	//	uiSize = rv.uiSize;
	//	uiFCountRx = rv.uiFCountRx;
	//	iFrameErr = rv.iFrameErr;
	//  ts = rv.ts;
	//}

	CFrameInfo(vpFrameInfo & rv)
	{
		memcpy(&pF, &rv.pF, sizeof(vpFrameInfo));
	}

	CFrameInfo & operator=(const CFrameInfo & rv)
	{
		pF = rv.pF;
		uiSize = rv.uiSize;
		uiFCountRx = rv.uiFCountRx;
		iFrameErr = rv.iFrameErr;
		ts = rv.ts;

		return *this;
	}

	void DeepCopy(uint8_t *pbuf, uint32_t uiBufsize, const CFrameInfo & rv)
	{
		uiSize = std::min<uint32_t>(uiBufsize, rv.uiSize);
		uiFCountRx = rv.uiFCountRx;
		iFrameErr = rv.iFrameErr;
		ts = rv.ts;

		if (pbuf)
		{
			pF = pbuf;
			memcpy(pF, rv.pF, uiSize);
		}
	}

	void Init()
	{
		pF = 0; uiSize = 0;
		uiFCountRx = 0;  
		iFrameErr = 0;
		ts = 0;
	}
	
	uint32_t cmd() const
	{
		if (IsCmd())
			return ((SEUCMD_HDR*)pF)->seucmd.cmd;
		else
			return (uint32_t)-1;
	}
	
	uint32_t action() const 
	{
		if (IsCmd())
			return ((SEUCMD_HDR*)pF)->seucmd.action;
		else
			return (uint32_t)-1;
	}
	
	uint32_t devid() const
	{
		if (IsCmd() || IsPno())
			return ((SEUCMD_HDR*)pF)->seucmd.seuid;
		else
			return (uint32_t)-1;
	}

	uint32_t devfc() const
	{
		if (IsNull() || IsCmd())
			return 0;
		else
			return ((SEUPNO_HDR*)pF)->seupno.frame;
	}

	int32_t err() const
	{
		return iFrameErr;
	}

	uint64_t & TS() 
	{
		return ts;
	}

	bool IsCmd() const
	{
		if (IsNull())
			return false;
		else if (((VP_FRAME_HDR*)pF)->preamble == VIPER_CMD_PREAMBLE)
			return true;
		else
			return false;
	}
	//{
	//	if (IsNull())
	//		return false;
	//	else
	//		return (((LPVP_FRAME_HDR)pF)->preamble == VIPER_CMD_PREAMBLE);
	//}

	bool IsPno() const
	{
		if (IsNull())
			return false;
		else if (((VP_FRAME_HDR*)pF)->preamble == VIPER_PNO_PREAMBLE)
			return true;
		else
			return false;
	}

	uint32_t Preamble() const
	{
		return ((VP_FRAME_HDR*)pF)->preamble;
	}


	bool IsNull() const
	{
		return (pF == 0);
	}

	bool IsNak() const
	{
		if (IsCmd())
			return (action() == CMD_ACTION_NAK);

		else
			return false;
	}

	bool IsNakWarning() const
	{
		if (IsCmd())
			return (action() == CMD_ACTION_NAK_WARNING);

		else
			return false;
	}

	bool IsAck() const
	{
		if (IsCmd())
			return (action() == CMD_ACTION_ACK);

		else
			return false;
	}


	uint8_t * PCmdPayload()
	{
		if (IsCmd() && !IsNull())
			return &pF[sizeof(SEUCMD_HDR)];
		else
			return 0;
	}
	
	uint32_t CmdPayloadSize()
	{
		if (IsCmd() && !IsNull())
		{
			return ((SEUCMD_HDR*)pF)->size - sizeof(SEUCMD) - VP_FRAME_CRC_SIZE;
		}
		else
			return 0;
	}

	uint8_t *PPnoBody()
	{
		if (IsPno() && !IsNull())
			return &pF[sizeof(VP_FRAME_HDR)];
		else
			return 0;
	}
	SENFRAMEDATA *pSen(int32_t s = 0)
	{
		if (IsPno() && !IsNull())
			return (SENFRAMEDATA *)&pF[sizeof(SEUPNO_HDR) + (s * sizeof(SENFRAMEDATA))];
		else
			return 0;
	}
	uint32_t PnoBodySize()
	{
		if (IsPno() && !IsNull())
		{
			return ((SEUPNO_HDR*)pF)->size - VP_FRAME_CRC_SIZE;
		}
		else
			return 0;
	}
};

class CVPcmd : public SEUCMD_HDR
{
public:
	CVPcmd( uint32_t pre = VIPER_CMD_PREAMBLE ) : ppay(0), szpay(0)
	{
		Init(pre);
	}

	CVPcmd(const CVPcmd & rv)
	{
		*this = rv;
	}

	void *       operator () (CVPcmd & rv)       { return (void*)&rv.preamble; }
	const void * operator () (const CVPcmd & rv) { return (const void*)&rv.preamble; }

	CVPcmd & operator = (const CVPcmd & rv)
	{
		memcpy(this, (const void *)&rv.preamble, sizeof(SEUCMD_HDR));
		return *this;
	}

	operator SEUCMD_HDR * () { return (SEUCMD_HDR *)this; }

	void Init( uint32_t pre=VIPER_CMD_PREAMBLE )
	{
		ppay = 0; szpay = 0;
		preamble = pre; // VIPER_CMD_PREAMBLE;
		size = sizeof(SEUCMD) + sizeof(VPCRC);
	}

	void Fill(uint32_t id, uint32_t cmd, uint32_t act, uint32_t a1 = 0, uint32_t a2 = 0, void *pp=0, uint32_t szp=0)
	{
		seucmd.seuid = id;
		seucmd.cmd = cmd;
		seucmd.action = act;
		seucmd.arg1 = a1;
		seucmd.arg2 = a2;
		if (act == CMD_ACTION_SET)
		{
			ppay = pp;
			szpay = szp;
			size += szp;
		}
		else
		{
			ppay = 0;
			szpay = 0;
		}
	}

	uint32_t & Seu() {
		return seucmd.seuid;
	}
	uint32_t & Cmd() {
		return seucmd.cmd;
	}
	uint32_t & Act() {
		return seucmd.action;
	}
	uint32_t & SnsArg() {
		return Arg1();
	}
	uint32_t & Arg1() {
		return seucmd.arg1;
	}
	uint32_t & Arg2() {
		return seucmd.arg2;
	}

	eViperCmds  ECmd() { return (eViperCmds)seucmd.cmd; }

	eCmdActions  EAct() { return (eCmdActions)seucmd.action; }

	bool IsGet() { return EAct() == CMD_ACTION_GET; }
	bool IsSet() { return EAct() == CMD_ACTION_SET; }
	bool IsReset() { return EAct() == CMD_ACTION_RESET; }
	bool IsAck() { return EAct() == CMD_ACTION_ACK; }
	bool IsNak() { return EAct() == CMD_ACTION_NAK; }
	bool IsNakWarning() { return EAct() == CMD_ACTION_NAK_WARNING; }

	void Prepare( uint8_t buf[], int & txbytes)
	{
		CVPcmd *ptx = (CVPcmd*)buf;

		//*ptx = *this;
		uint32_t crc_count = sizeof(SEUCMD_HDR);
		memcpy(ptx, (const void *)this, sizeof(SEUCMD_HDR));
		if (ppay)
		{
			memcpy(&buf[sizeof(SEUCMD_HDR)], ppay, szpay);
			crc_count += szpay;
		}

		uint32_t *pcrc = (uint32_t*)&buf[crc_count];
		*pcrc = CalcCRC_Bytes(buf, crc_count);

		txbytes = crc_count + sizeof(uint32_t);
	}

	static size_t paysize(eViperCmds cmd);
	static size_t framesize(eViperCmds cmd);
	static const char *cmdstr(uint32_t cmd);
	static const char *actstr(uint32_t act);
	static const char *posunitsstr(uint32_t posu, bool bpub=false, bool bbrief=false);
	static const char *oriunitsstr(uint32_t oriu, bool bpub = false, bool bbrief = false);
	static const char *frameratestr(uint32_t frate);
	static const char *filterlevstr(uint32_t flev);
	static const char *filtertarstr(uint32_t ftar);
	static const char *fttstr(uint32_t ftt);
	static const char *hemispherestr(uint32_t hemval);
	static const char *vphemispherestr(uint32_t hemval);
	static const char *baudstr(uint32_t b);
	static const char *paritystr(uint32_t p);
	static const char *baudvalstr(uint32_t b); // valstr() fcns return human parsable text strings
	static const char *parityvalstr(uint32_t p);
	static const char * bitcodestr(uint32_t p);
	static const char * bitcodevalstr(uint32_t p);
	static const char *stylusmodestr(uint32_t p);
	static const char *snsorigstr(uint32_t p);
	static const HEMISPHERE_CONFIG *hemispherecfg(uint32_t hemval);

	static const int baudbps(uint32_t b);
	static const eBaud bpsbaud(int32_t bps);

	static void InitCmdInfo();
	static void InitActInfo();
	static void InitUnitsInfo();
	static void InitFRateInfo();
	static void InitFilterLevInfo();
	static void InitFilterTarInfo();
	static void InitFTTInfo();
	static void InitHemisphereInfo(); //VPif-only enum for CHemisphereCfg
	static void InitViperHemisphereInfo();
	static void InitBaudInfo();
	static void InitParityInfo();
	static void InitBITcodeInfo();
	static void InitStylusInfo();
	static void InitSnsOrigInfo();

	static bool ValidateLeader(uint8_t * pbuf, uint32_t cmdPre = VIPER_CMD_PREAMBLE, uint32_t pnoPre = VIPER_PNO_PREAMBLE);

	static bool ValidateCRC(uint8_t *pbuf)
	{
		uint32_t size = (pbuf == 0) ? 0 : ((VP_FRAME_HDR*)pbuf)->size;

		if (size == 0)
			return false;

		uint32_t count = 0;
		uint32_t *pCrc = 0;
		uint32_t crcCalc = 0;
		uint32_t crcFrame = 0;

		count = VP_FRAME_LEADER_SIZE + size - VP_FRAME_CRC_SIZE;
		pCrc = (uint32_t*)(pbuf + VP_FRAME_LEADER_SIZE + size - VP_FRAME_CRC_SIZE);

		crcFrame = *pCrc;
		crcCalc = CalcCRC_Bytes(pbuf, count);

		return (crcFrame == crcCalc);
	}

	static bool ValidateAct(eViperCmds, eCmdActions);
	
	static void crc16(uint32_t * crc, uint32_t data)
	{
		static const char op[16] = { 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0 };
		data = (data ^ (*crc)) & 0xff;
		*crc >>= 8;

		if (op[data & 0xf] ^ op[data >> 4])
			*crc ^= 0xc001;

		data <<= 6;
		*crc ^= data;
		data <<= 1;
		*crc ^= data;

		return;
	}

	static uint32_t CalcCRC_Bytes(uint8_t *data, uint32_t count)
	{
		uint32_t crc;
		uint32_t n;

		crc = 0;
		for (n = 0; n<count; n++)
			crc16(&crc, data[n]);

		return crc;
	}
	// For Q15, the denominator is 2^15, or 32768
	static float FractToFloat(int16_t fract, float factor)
	{
		return (float)(fract) / 32768 * factor;
	}

	static eViperCmds tag2vpcmd(uint32_t); //place holder
	static uint32_t   vp2tagcmd(eViperCmds); //place holder
	void   vp2tag();
	static uint32_t TrkCmdPre(uint32_t pid);
	static uint32_t TrkPnoPre(uint32_t pid);

	void * ppay;
	uint32_t szpay;

};

typedef struct _VP_PNO
{
	VP_FRAME_HDR hdr;
	SEUPNO seupno;
	SENFRAMEDATA sarr[SENSORS_PER_SEU];
}VP_PNO;

typedef struct _SNSFRMALL
{
	SENFRAMEDATA f;
	float accel[4];
}SNSFRMALL;
class CVPSnsFrameA : public SNSFRMALL
{
public:
	//CVPSnsFrameA() 
	//{
	//	Init();
	//}
	//CVPSnsFrameA(SENFRAMEDATA *pF=0)
	//{
	//	Fill(pF);
	//}
	CVPSnsFrameA(SENFRAMEDATA_A *pFA=0)
	{
		Fill(pFA);
	}
	operator SENFRAMEDATA * () { return &f; }

	void Init()
	{
		memset(&f, 0, sizeof(SNSFRMALL));
	}
	bool Qtrn()
	{
		return f.SFinfo.bfOriUnits == ORI_QUATERNION;
	}
	int NumTerms()
	{
		return Qtrn() ? 4 : 3;
	}
	float OriFactor()
	{
		return OriFractFactor[f.SFinfo.bfOriUnits];
	}
	void ConvertOri(int16_t *pFracts, int count, float factor)
	{
		for (int i = 0; i < count; i++)
		{
			f.pno.ori[i] = CVPcmd::FractToFloat(pFracts[i], factor);
		}
	}
	void ConvertAccel(int16_t *pFracts, int count=4, float factor=FFACTOR_ACCEL)
	{
		for (int i = 0; i < count; i++)
		{
			accel[i] = CVPcmd::FractToFloat(pFracts[i], factor);
		}
	}
	void Fill(SENFRAMEDATA_A *pFA=0)
	{
		if (!pFA)
		{
			Init();
			return;
		}
		f.SFinfo = pFA->SFinfo;
		memcpy(f.pno.pos, pFA->pno.pos, sizeof(f.pno.pos));
		ConvertOri(pFA->pno.ori, Qtrn() ? 4 : 3, OriFractFactor[f.SFinfo.bfOriUnits]);
		ConvertAccel(pFA->pno.acc);

	}
	void Fill(SENFRAMEDATA *pF = 0)
	{
		if (!pF)
		{
			Init(); 
			return;
		}
		memcpy(&f, pF, sizeof(SENFRAMEDATA));
	}

	static float OriFractFactor[ORI_MAX];

};


class CVPSeuPno : public VP_PNO
{
public:
	CVPSeuPno()
	{
		Init();
	}

	CVPSeuPno(const CVPSeuPno & rv)
	{
		memcpy(&hdr, &rv.hdr, sizeof(VP_PNO));
	}

	CVPSeuPno(const VP_PNO * prv)
	{
		Init();
		memcpy(&hdr, prv, sizeof(VP_PNO));
	}

	CVPSeuPno(uint8_t *p)
	{
		Init();
		memcpy(&hdr, p, sizeof(VP_PNO));
	}

	operator VP_PNO * () { return (VP_PNO *)this; }

	CVPSeuPno & operator= (const CVPSeuPno & rv)
	{
		memcpy(&hdr, &rv.hdr, sizeof(VP_PNO));
		return *this;
	}

	CVPSeuPno & operator= (const VP_PNO * prv)
	{
		memcpy(&hdr, prv, sizeof(VP_PNO));
		return *this;
	}

	CVPSeuPno & operator= (uint8_t *p)
	{
		memcpy(&hdr, p, sizeof(VP_PNO));
		return *this;
	}

	uint32_t Extractraw(uint8_t *p)
	{
		if (!p)
			return 0;

		VP_FRAME_HDR *ph = (VP_FRAME_HDR*)p;
		if (ph->preamble != VIPER_PNO_PREAMBLE)
			return 0;

		Init();
		uint32_t index = 0;
		memcpy(&hdr, ph, sizeof(VP_FRAME_HDR)); index += sizeof(VP_FRAME_HDR);
		memcpy(&seupno, &p[index], sizeof(SEUPNO)); index += sizeof(SEUPNO);
		for (uint32_t i=0; i < seupno.sensorCount; i++)
		{
			memcpy(&sarr[i], &p[index], sizeof(SENFRAMEDATA)); index += sizeof(SENFRAMEDATA);
		}

		index += sizeof(VPCRC);
		return index;
	}

	uint32_t Extractseupno(uint8_t *p)
	{
		if (!p)
			return 0;

		//VP_FRAME_HDR *ph = (VP_FRAME_HDR*)p;
		//if (ph->preamble != VIPER_PNO_PREAMBLE)
		//	return 0;

		Init();
		uint32_t index = 0;
		//memcpy(&hdr, ph, sizeof(VP_FRAME_HDR)); index += sizeof(VP_FRAME_HDR);
		memcpy(&seupno, &p[index], sizeof(SEUPNO)); index += sizeof(SEUPNO);
		for (uint32_t i = 0; i < seupno.sensorCount; i++)
		{
			memcpy(&sarr[i], &p[index], sizeof(SENFRAMEDATA)); index += sizeof(SENFRAMEDATA);
		}

		index += sizeof(VPCRC);
		return index;
	}

	uint32_t SensorCount() const { return seupno.sensorCount; }

	uint32_t SensorMap() const
	{
		uint32_t map = 0;

		for (uint32_t i = 0; i < seupno.sensorCount; i++)
		{
			const SENFRAMEDATA*  pSD = &(sarr[i]);

			map |= (1 << pSD->SFinfo.bfSnum);
		}
		return map;
	}

	SENFRAMEDATA * SensFrame(int i)
	{
		if (i < (int)SensorCount())
			return &(sarr[i]);
		else
			return 0;
	}

	SENFRAMEDATA_A * SensFrameA(int i)
	{
		if (i < (int)SensorCount())
			return (SENFRAMEDATA_A*)(&(sarr[i]));
		else
			return 0;
	}

	uint32_t Mode()
	{
		return seupno.HPinfo.bfPnoMode;
	}

	uint32_t BITerr()
	{
		return seupno.HPinfo.bfBITerr;
	}

	void Init()
	{
		memset(&hdr, 0, sizeof(VP_PNO));
	}

	void MakePrintable()
	{

	}




};

//typedef struct WHOAMI_STRUCT
//{
//	char device_name[NAME_SIZE];
//	char hw_ser_no[SERNUM_SIZE];
//	char ioproc_pn[PN_SIZE];
//	char dsp_bt_fw_pn[PN_SIZE];
//	char dsp_app_fw_pn[PN_SIZE];
//}WHOAMI_STRUCT;
class CWhoAmI : public WHOAMI_STRUCT
{
public:
	CWhoAmI()
	{
		Init();
	}

	CWhoAmI( const CWhoAmI & rv)
	{
		memcpy(device_name, rv.device_name, sizeof(WHOAMI_STRUCT));
	}

	CWhoAmI(const WHOAMI_STRUCT * prv)
	{
		memcpy(device_name, prv, sizeof(WHOAMI_STRUCT));
	}

	operator WHOAMI_STRUCT * () { return (WHOAMI_STRUCT *)this; }

	void Init()
	{
		memset(device_name, 0, sizeof(WHOAMI_STRUCT));
	}

	size_t Size() { return sizeof(WHOAMI_STRUCT); }

	void Fill(const char* name = 0, const char * hwser = 0, const char* iop = 0, const char * dspbt = 0, const char * dspapp = 0)
	{
		if (name)
			memcpy(device_name, name, sizeof(device_name));

		if (hwser)
			memcpy(hw_ser_no, hwser, sizeof(hw_ser_no));

		if (iop)
			memcpy(ioproc_pn, iop, sizeof(ioproc_pn));

		if (dspbt)
			memcpy(dsp_bt_fw_pn, dspbt, sizeof(dsp_bt_fw_pn));

		if (dspapp)
			memcpy(dsp_app_fw_pn, dspapp, sizeof(dsp_app_fw_pn));
	}

	void MakePrintable()
	{
		device_name[NAME_SIZE - 1] = 0;
		hw_ser_no[SERNUM_SIZE - 1] = 0;
		ioproc_pn[PN_SIZE - 1] = 0;
		dsp_bt_fw_pn[PN_SIZE - 1] = 0;
		dsp_app_fw_pn[PN_SIZE - 1] = 0;
	}


};

//typedef struct _STATION_MAP_CONFIG
//{
//	uint32_t station_map;	// read-only.  Ignored by tracker in set action
//	uint32_t enabled_map;
//}STATION_MAP_CONFIG;


//typedef struct _STATION_MAP
//{
//	union
//	{
//		uint32_t stamap;
//		struct _bf {
//			uint32_t sensor_map : 16;
//			uint32_t reserved1 : 8;
//			uint32_t source_map : 4;
//			uint32_t reserved2 : 4;
//		} bf;
//	};
//}STATION_MAP;
class CStationMap : public STATION_MAP
{
public:
	uint32_t sns_detected_count;  //<! Detected sensor count
	uint32_t src_detected_count;  //<! Detected source count
	uint32_t en_count;        //<! Enabled sensor count (enabled & detected)
	uint32_t en_map;          //<! Enabled sensor map.  For host use and maintenance only!

	CStationMap()
	{
		Init();
	}

	CStationMap(const CStationMap & rv)
	{
		//memcpy(&sensor_map, &rv.sensor_map, sizeof(STATION_MAP));
		stamap = rv.stamap;
		en_count = rv.en_count;
		en_map = rv.en_map;
		CountDetected();

	}

	CStationMap(const STATION_MAP * prv)
	{
		//memcpy(&station_map, prv, sizeof(STATION_MAP_CONFIG));
		stamap = prv->stamap;
		CountDetected();
		en_count = sns_detected_count;
		en_map = bf.sensor_map;
	}

	CStationMap(uint32_t map)
	{
		stamap = map;
		CountDetected();
		en_count = sns_detected_count;
		en_map = bf.sensor_map;
	}

	operator STATION_MAP * () { return (STATION_MAP *)this; }
	operator void * () { return (void *)((STATION_MAP *)this); }
	bool operator == (const STATION_MAP *prv) { return (prv->stamap == stamap) ; }

	void Init()
	{
		memset(&stamap, 0, sizeof(STATION_MAP));
		CountDetected();
		InitEnabled();
	}

	size_t Size() { return sizeof(STATION_MAP); }

	void CountDetected()
	{
		sns_detected_count = 0;
		for (int i = 0; i < SENSORS_PER_SEU; i++)
		{
			if ((1 << i) & bf.sensor_map)
				sns_detected_count++;
		}
		src_detected_count = 0;
		for (int i = 0; i < SOURCES_PER_SEU; i++)
		{
			if ((1 << i) & bf.source_map)
				src_detected_count++;
		}

	}
	void CountEnabled()
	{
		CountDetected();
		en_count = 0;
		for (int i = 0; i < SENSORS_PER_SEU; i++)
		{
			if ((1 << i) & (en_map))
				en_count++;
		}
	}
	void InitEnabled()
	{
		en_map = bf.sensor_map;
		CountEnabled();
	}
	void InitDefault()
	{
		memcpy(this, &Default, sizeof(STATION_MAP));
		CountDetected();
		InitEnabled();
	}
	uint32_t SensorMap() { return bf.sensor_map; }
	uint32_t SourceMap() { return bf.source_map; }

	uint32_t & EnabledMap()  { return en_map; }

	//bool IsEnabled(int32_t sns)  { return ((1 << sns) & enabled_map) != 0; }
	bool IsDetected(int32_t sns) { return ((1 << sns) & bf.sensor_map) != 0; }
	bool IsEnabled(int32_t sns)   { return ((1 << sns) & (en_map & bf.sensor_map)) != 0; }
	bool IsSrcDetected(int32_t src) { return ((1 << src) & bf.source_map) != 0; }
	uint32_t SnsDetectedCount() { CountDetected(); return sns_detected_count; }
	void SetEnabled(uint32_t s) 
	{	
		en_map &= (1 << s); 
		CountEnabled(); 
	}
	//void SetProcEnMap(int p, uint8_t map)
	//{
	//	uint32_t mask = ~(0xf << p);
	//	enabled_map &= mask; // clear the map for this p
	//	enabled_map |= (map << p);
	//	CountActive();
	//}

	//void Fill(uint32_t stamap = 0, uint32_t enmap = 0) // don't need to Fill.  It is read-only
	//{
	//	station_map = stamap;
	//	enabled_map = enmap;
	//	CountActive();
	//}

	void MakePrintable()
	{

	}

	//void Report(std::string & s, bool bTitle = false, bool bFlat = false)
	//{
	//	RPT_szCRLF = bFlat ? "" : RPT_CRLF;
	//	RPT_szINDENT = bTitle ? "   " : "";
	//
	//	MakePrintable();
	//	char sz[200];
	//	sprintf_s(sz, 200,
	//		"%s" "%s"
	//		"%s" "sens procs : % 6d" "%s"
	//		"%s" "station_map: 0x%03x" "%s"
	//		"%s" "enabled_map: 0x%04x\r\n",
	//		bTitle ? "STATION_MAP_CONFIG: " : "", bTitle ? RPT_szCRLF : "",
	//		RPT_szINDENT, snsprocs, RPT_szCRLF,
	//		RPT_szINDENT, sensormap, RPT_szCRLF,
	//		RPT_szINDENT, enabled_map);
	//	s += sz;
	//}



	static STATION_MAP Default;
};

//typedef struct _UNITS_CONFIG
//{
//	uint32_t pos_units;		// valid values in enum eViperPosUnits
//	uint32_t ori_units;		// valid values in enum eViperOriUnits
//}UNITS_CONFIG;
class CUnitsCfg : public UNITS_CONFIG
{
public:
	CUnitsCfg()
	{
		Init();
	}

	CUnitsCfg(const CUnitsCfg & rv)
	{
		memcpy(&pos_units, &rv.pos_units, sizeof(UNITS_CONFIG));
	}

	CUnitsCfg(const UNITS_CONFIG * prv)
	{
		memcpy(&pos_units, prv, sizeof(UNITS_CONFIG));
	}
	CUnitsCfg(eViperPosUnits posu, eViperOriUnits oriu)
	{
		pos_units = (uint32_t)posu;
		ori_units = (uint32_t)oriu;
	}

	operator UNITS_CONFIG * () { return (UNITS_CONFIG *)this; }
	operator void * () { return (void *)((UNITS_CONFIG *)this); }
	bool operator == (const UNITS_CONFIG *prv) const { return (prv->pos_units == pos_units) && (prv->ori_units == ori_units); }

	void Init()
	{
		InitDefault();
	}
	void InitDefault()
	{
		memcpy(this, &Default, sizeof(UNITS_CONFIG));
	}

	void Fill(uint32_t pos = POS_INCH, uint32_t ori = ORI_QUATERNION)
	{
		pos_units = pos;
		ori_units = ori;
	}

	bool IsEuler() { return ori_units != ORI_QUATERNION; }
	eViperPosUnits ePos() { return (eViperPosUnits)pos_units; }
	eViperOriUnits eOri() { return (eViperOriUnits)ori_units; }
	const char *posunitsstr(bool bpub = false) { return CVPcmd::posunitsstr(pos_units, bpub); }
	const char *oriunitsstr(bool bpub = false) { return CVPcmd::oriunitsstr(ori_units, bpub); }

	static UNITS_CONFIG Default;
};

class CEnumCfg// : public ENUM_CONFIG_STRUCT
{
public:
ENUM_CONFIG val;

CEnumCfg()
{
Init();
}

CEnumCfg(ENUM_CONFIG e)
{
val = e;
}

CEnumCfg(const CEnumCfg & rv)
{
val = rv.val;// memcpy(&val, &rv.val, sizeof(ENUM_CONFIG));
}

CEnumCfg(const ENUM_CONFIG * prv)
{
memcpy(&val, prv, sizeof(ENUM_CONFIG));
}

operator ENUM_CONFIG * () { return &val; }
operator ENUM_CONFIG () { return val; }
operator void * () { return (void *)((ENUM_CONFIG *)&val); }
bool operator == (const ENUM_CONFIG *prv) const { return *prv == val; }

CEnumCfg & operator = (const ENUM_CONFIG & rv)
{
memcpy(&val, &rv, sizeof(ENUM_CONFIG));

return *this;
}

void Init()
{
memset(&val, 0, sizeof(ENUM_CONFIG));
}
void InitDefault()
{
memcpy(&val, &Default, sizeof(ENUM_CONFIG));
}

void Fill(int32_t m = 0)
{
val = m;
}

void MakePrintable()
{
}

/*void Report(std::string & s, bool bTitle = false, bool bFlat = false)
{
RPT_szCRLF = bFlat ? "" : RPT_CRLF;
RPT_szINDENT = bTitle ? "   " : "";
MakePrintable();
char sz[200];
sprintf_s(sz, 200,
"%s" "%s"
"%s" "VAL: %d : %d\r\n",
bTitle ? "ENUM_CONFIG :" : "", bTitle ? RPT_szCRLF : "",
RPT_szINDENT, val, val
);
s += sz;
}*/

static ENUM_CONFIG Default;
};

class CFTTCfg : public CEnumCfg
{
public:
CFTTCfg() : CEnumCfg() {};
CFTTCfg(eFTTMode e) : CEnumCfg((ENUM_CONFIG)e) {};

virtual void InitDefault()
{
memcpy(&val, &Default, sizeof(ENUM_CONFIG));
}

bool Enabled() { return val != 0; }

/*virtual void Report(std::string & s, bool bTitle = false, bool bFlat = false)
{
RPT_szCRLF = bFlat ? "" : RPT_CRLF;
RPT_szINDENT = bTitle ? "   " : "";
MakePrintable();
char sz[200];
sprintf_s(sz, 200,
"%s" "%s"
"%s" "VAL: %d : %s\r\n",
bTitle ? "FTT_CONFIG :" : "", bTitle ? RPT_szCRLF : "",
RPT_szINDENT, val, CVPcmd::fttstr(val)
);
s += sz;
}*/
static FTTMODE_CONFIG Default;
};

//typedef struct _SRCROT_CONFIG
//{
//	uint32_t srcindex;
//	float rot[4];
//}SRCROT_CONFIG;

//typedef struct _BLOCK_CONFIG
//{
//		FRAMERATE_CONFIG	frame_rate;					/**  6   set/get/reset,	SEU		FRAMERATE_CONFIG	p		cfg							*/
//		UNITS_CONFIG		units;						/**  7   set/get/reset,	SEU		UNITS_CONFIG		p		cfg							*/
//		SRC_CONFIG			src_cfg[SOURCES_PER_SEU];	/**  8   set/get/reset,	SEU		SRC_CONFIG			p		cfg							*/
//		BINARY_CONFIG		sync_mode;					/**  9   set/get/reset,	SEU		BINARY_CONFIG		p		cfg-binary					*/
//		STATION_MAP			sta_map;					/**  10  set/get/reset,	SEU		STATION_MAP					read-only					*/
//		STYLUS_CONFIG		stylus_mode;				/**  11  set/get/reset,	sensor	STYLUS_CONFIG(ENUM)	p		cfg							*/
//		VIPER_SEUID			seuid;						/**  12  set/get/reset,	SEU		VIPER_SEUID			p		cfg							*/
//		BINARY_CONFIG		dualout_mode;				/**  13  set/get/reset,	SEU		BINARY_CONFIG		p		cfg-binary					*/
//		SERIAL_CONFIG		ser_cfg;					/**  14  set/get/reset,	SEU		SERIAL_CONFIG		p		cfg							*/
//		PF_CONFIG_EXT		pf_cfg;						/**  28  set/get/reset,	SEU		PF_CONFIG			p		cfg							*/
//		SENSOR_CONFIG		sensor_config[SENSORS_PER_SEU];
//}BLOCK_CONFIG;
//typedef struct _VIRTUAL_SNS_CONFIG {
//	uint32_t uInput1;
//	uint32_t reserved0;
//	uint32_t reserved1;
//	uint32_t reserved2;
//}VIRTUAL_SNS_CONFIG;

class CSnsVirtCfg : public VIRTUAL_SNS_CONFIG
{
public:
	uint32_t m_uVirtSns;
	uint32_t & VirtSns() { return m_uVirtSns; }
	void * TOP() const { return (void*)this; }
	size_t SZCFG() { return sizeof(VIRTUAL_SNS_CONFIG); }
	CSnsVirtCfg()
	{
		Init(); //InitDefault
	}

	CSnsVirtCfg(const CSnsVirtCfg & rv)
	{
		memcpy(TOP(), rv.TOP(), SZCFG());
	}

	CSnsVirtCfg(const VIRTUAL_SNS_CONFIG * prv)
	{
		memcpy(TOP(), prv, SZCFG());
	}

	operator VIRTUAL_SNS_CONFIG * () { return (VIRTUAL_SNS_CONFIG *)this; }
	operator uint32_t * () { return (uint32_t*)TOP(); }
	operator void * () { return (void *)TOP(); }
	bool operator == (const uint32_t rv) const { return (rv == uInput1); }

	CSnsVirtCfg & operator = (const uint32_t & rv)
	{
		uInput1 = rv; return *this;
	}

	void Init()
	{
		memset(TOP(), 0, SZCFG());
		m_uVirtSns = 0;
	}
	void InitDefault()
	{
		memcpy(this, &Default, SZCFG());
		m_uVirtSns = 0;
	}

	void Fill(uint32_t in1, uint32_t in2=0, uint32_t op1=0, uint32_t op2=0)
	{
		Init();
		uInput1 = in1;
		reserved0 = in2;
		reserved1 = op1;
		reserved2 = op2;
	}

	/*void MakePrintable()
	{
	}
	void Report(std::string & s, bool bTitle = false, bool bFlat = false)
	{
		RPT_szCRLF = bFlat ? "" : RPT_CRLF;
		RPT_szINDENT = bTitle ? "   " : "";
		MakePrintable();
		char sz[200];
		sprintf_s(sz, 200,
			"%s" "%s"
			"%s" "Input1: % 2d\r\n",
			bTitle ? "VIRTUAL_SNS_CONFIG: " : "", bTitle ? RPT_szCRLF : "",
			RPT_szINDENT, uInput1);
		s += sz;
	}*/

static VIRTUAL_SNS_CONFIG Default;

};

//typedef struct _HEMISPHERE_CONFIG
//{
//	struct {
//		uint32_t track_en : 1;
//		uint32_t auto_en : 1;
//		uint32_t res : 30;
//	}bf;
//	float params[3];
//}HEMISPHERE_CONFIG;
class CHemisphereCfg : public HEMISPHERE_CONFIG
{
public:
	enum eHemPresetVal {
		POS_X, POS_Y, POS_Z, NEG_X, NEG_Y, NEG_Z, CUSTOM,/* AUTO_HEM, TRACK_EN,*/ HEMPRESETVAL_MAX
	};

	CHemisphereCfg()
	{
		Init();
	}

	CHemisphereCfg(const CHemisphereCfg & rv)
	{
		memcpy(&bf, &rv.bf, sizeof(HEMISPHERE_CONFIG));
		eVal = rv.eVal;
	}

	CHemisphereCfg(const HEMISPHERE_CONFIG * prv)
	{
		Init();
		memcpy(&bf, prv, sizeof(HEMISPHERE_CONFIG));
		SetPresetEnum();
	}

	operator HEMISPHERE_CONFIG * () { return (HEMISPHERE_CONFIG *)this; }
	operator void * () { return (void *)((HEMISPHERE_CONFIG *)this); }

	CHemisphereCfg & operator= (const CHemisphereCfg & rv)
	{
		memcpy(&bf, &rv.bf, sizeof(HEMISPHERE_CONFIG));
		SetPresetEnum();
		return *this;
	}

	CHemisphereCfg & operator= (const HEMISPHERE_CONFIG * prv)
	{
		memcpy(params, prv, sizeof(HEMISPHERE_CONFIG));
		SetPresetEnum();
		return *this;
	}
	bool operator ==  (const HEMISPHERE_CONFIG * prv) const
	{
		return (prv->bf.track_en == bf.track_en) && (prv->bf.auto_en == bf.auto_en) && (FP_EQ(params[0], prv->params[0]) && FP_EQ(params[1], prv->params[1]) && FP_EQ(params[2], prv->params[2]) && FP_EQ(params[3], prv->params[3]));
	}

	void Init()
	{
		Fill(POS_X, false);
		//memset(&track_en, 0, sizeof(HEMISPHERE_CONFIG));
		//params[0] = 1.0f;
		//eVal = POS_X;
	}
	void InitDefault()
	{
		memcpy(this, &Default, sizeof(HEMISPHERE_CONFIG));
	}

	size_t Size() { return sizeof(HEMISPHERE_CONFIG); }

	void Fill(float x = 1.0, float y = 0, float z = 0, bool bAutoHem = false, bool bHTrack = false)
	{
		params[0] = x;
		params[1] = y;
		params[2] = z;
		
		bf.auto_en = bAutoHem ? 1 : 0;
		bf.track_en = (bAutoHem || bHTrack) ? 1 : 0;
		SetPresetEnum();
	}

	void Fill(float *p, bool bAutoHem = false, bool bHTrack=false)
	{
		memcpy(params, p, 3 * sizeof(float));
		
		bf.auto_en = bAutoHem ? 1 : 0;
		bf.track_en = (bAutoHem || bHTrack) ? 1 : 0;

		SetPresetEnum();
	}

	void Fill(enum eHemPresetVal e, bool bAutoHem = false, bool bHTrack=false)
	{
		const HEMISPHERE_CONFIG * phc = CVPcmd::hemispherecfg(e);
		memcpy(this, phc, sizeof(HEMISPHERE_CONFIG));
		bf.auto_en = bAutoHem ? 1 : 0;
		bf.track_en = (bAutoHem || bHTrack) ? 1 : 0;
		eVal = e;
	}

	void SetTrackEn(bool b)
	{
		bf.track_en = b ? 1 : 0;
	}
	void SetAutoEn(bool b)
	{
		bf.auto_en = b ? 1 : 0;
	}
	bool TrackEn() { return bf.track_en ? true : false; }
	bool AutoEn() { return bf.auto_en ? true : false; }

	void SetPresetEnum()
	{
		//bool bSet = true;
		float xvec = params[0];
		float yvec = params[1];
		float zvec = params[2];

		if ((xvec != 0) && (yvec == 0.0f) && (zvec == 0.0f))
		{
			eVal = (xvec > 0) ? POS_X : NEG_X;
		}
		else if ((xvec == 0) && (yvec != 0.0f) && (zvec == 0.0f))
		{
			eVal = (yvec > 0) ? POS_Y : NEG_Y;
		}
		else if ((xvec == 0) && (yvec == 0.0f) && (zvec != 0.0f))
		{
			eVal = (zvec > 0) ? POS_Z : NEG_Z;
		}
		//else if ((xvec == 0) && (yvec == 0.0f) && (zvec == 0.0f))
		//{
		//	if (bf.auto_en)
		//		eVal = AUTO_HEM;
		//	else
		//		eVal = TRACK_EN;
		//	bf.track_en = 1;
		//}
		else
		{
			eVal = CUSTOM;

			//if (abs(xvec) > abs(yvec))
			//{
			//	if (abs(xvec) > abs(zvec))
			//		eVal = (xvec > 0) ? CHemisphereCfg::POS_X : CHemisphereCfg::NEG_X;
			//	else
			//		eVal = (zvec > 0) ? CHemisphereCfg::POS_Z : CHemisphereCfg::NEG_Z;
			//}
			//else if (abs(yvec) > abs(xvec))
			//{
			//	if (abs(yvec) > abs(zvec))
			//		eVal = (yvec > 0) ? CHemisphereCfg::POS_Y : CHemisphereCfg::NEG_Y;
			//	else if (abs(zvec) > abs(yvec))
			//		eVal = (zvec > 0) ? CHemisphereCfg::POS_Z : CHemisphereCfg::NEG_Z;
			//
			//}
			//// x == y.  Now what about z
			//else if (abs(zvec) > abs(xvec))
			//{
			//	eVal = (zvec > 0) ? CHemisphereCfg::POS_Z : CHemisphereCfg::NEG_Z;
			//}
			//// x = y and z is < than both.  Don't know what hemisphere that is. Just call it POS_X.
			//else if (abs(zvec) < abs(xvec))
			//{
			//	eVal = (xvec > 0) ? CHemisphereCfg::POS_X : CHemisphereCfg::NEG_X;
			//}
			//// x = y = z.  Are they zero?
			//else if (zvec == 0.0f)
			//{
			//	eVal = CHemisphereCfg::TRACK_EN;
			//	cH.track_en = TRUE;
			//}
			//// if not zero, afu
			//else
			//	eVal = (xvec > 0) ? CHemisphereCfg::POS_X : CHemisphereCfg::NEG_X;
		}

		//if (bf.auto_en)
		//	eVal = AUTO_HEM;
		//else
		//		eVal = TRACK_EN;
		//bf.track_en = 1;

	}

	void MakePrintable()
	{

	}

	/*void Report(std::string & s, bool bTitle = false, bool bFlat = false)
	{
		RPT_szCRLF = bFlat ? "" : RPT_CRLF;
		RPT_szINDENT = bTitle ? "   " : "";
		MakePrintable();
		char sz[200];

		sprintf_s(sz, 200,
			"%s" "%s"
			"%s" "%s : %d, %d, %f, %f, %f\r\n",
			bTitle ? "HEMISPHERE_CONFIG: " : "", bTitle ? RPT_szCRLF : "",
			RPT_szINDENT, CVPcmd::hemispherestr(eVal), bf.track_en, bf.auto_en, params[0], params[1], params[2]);

		s += sz;
	}*/

	uint32_t eVal;

	//bool TrackingEnabledParams()
	//{
	//	return ((params[0] == 0.0f) && (params[1] == 0.0f) && (params[2] == 0.0f));
	//}

	static HEMISPHERE_CONFIG Default;

};