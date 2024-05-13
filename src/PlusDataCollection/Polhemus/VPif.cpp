// VSS $Header: /Viper/VPCmdIF/VPCmdIF/VPif.cpp 41    3/20/20 6:35p Suzanne $

#include "VPif.h"
#include <vector>
//#include "VPtypesUT.h"

//#ifndef CLAMP
//#define CLAMP(val, min, max) ((val) < (min) ? (min) : ((val) > (max) ? (max) : (val)))
//#endif

#ifndef MIN
#define MIN(a, b)	((a) < (b) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a, b)	((a) > (b) ? (a) : (b))
#endif



typedef enum
{
	CMDTYPE_CFG
	, CMDTYPE_BINCFG
	, CMDTYPE_EXEC
	, CMDTYPE_PNO
	, CMDTYPE_RO

	, CMDTYPE_MAX
}eCmdType;
typedef enum
{
	SCOPE_SENSOR
	,SCOPE_SEU

	,SCOPE_MAX
}eCmdScope;
typedef enum
{
	CFG_RUNTIME
	,CFG_PERSIST
	,CFG_NA

	,CFG_MAX
}eCfgPersistence;

#define SGR_MAP(s,g,r)  ((s << CMD_ACTION_SET) | (g << CMD_ACTION_GET) | (r << CMD_ACTION_RESET))

#define SET_GET			SGR_MAP(1,1,0)
#define SET_GET_RESET   SGR_MAP(1,1,1)
#define SET_RESET		SGR_MAP(1,0,1)
#define GET_RESET		SGR_MAP(0,1,1)
#define SETONLY			SGR_MAP(1,0,0)
#define GETONLY			SGR_MAP(0,1,0)
#define RESETONLY		SGR_MAP(0,0,1)

#define ACT_MAP(a)(1 << a)

#define NO_TAG_EQUIV  ((uint32_t)-1)

typedef struct _hemisphereinfo
{
	uint32_t et;
	char * szt;
	HEMISPHERE_CONFIG cfg;
}hemisphereinfo;
hemisphereinfo g_heminfo[CHemisphereCfg::HEMPRESETVAL_MAX + 1] =
{ CHemisphereCfg::HEMPRESETVAL_MAX, "HEMPRESETVAL_MAX", {0, 0, 0, 0.0f, 0.0f, 0.0f} };

typedef struct _vpheminfo
{
	eViperHemisphere ehem;
	char *   szhem;
}vpheminfo;
vpheminfo g_vpheminfo[E_VP_HEM_MAX + 1] =
{ E_VP_HEM_MAX, "E_VP_HEM_MAX" };

void CVPcmd::InitHemisphereInfo()
{
	if (g_heminfo[0].et != CHemisphereCfg::HEMPRESETVAL_MAX)
		return;

	for (int i = 0; i <= CHemisphereCfg::HEMPRESETVAL_MAX; i++)
	{
		switch (i)
		{
		default:
			g_heminfo[i] = { (uint32_t)CHemisphereCfg::HEMPRESETVAL_MAX, "HEMPRESETVAL_MAX", {0, 0, 0, 0.0f, 0.0f, 0.0f} };
			break;
		case CHemisphereCfg::POS_X: // = 0 /**< Target position and orientation */
			g_heminfo[i] = { CHemisphereCfg::POS_X, "POS_X", {0, 0, 0, 1.0f, 0.0f, 0.0f} };
			break;
		case CHemisphereCfg::POS_Y: // = 0 /**< Target position and orientation */
			g_heminfo[i] = { CHemisphereCfg::POS_Y, "POS_Y",{ 0, 0, 0, 0.0f, 1.0f, 0.0f } };
			break;
		case CHemisphereCfg::POS_Z: // = 0 /**< Target position and orientation */
			g_heminfo[i] = { CHemisphereCfg::POS_Z, "POS_Z",{ 0, 0, 0, 0.0f, 0.0f, 1.0f } };
			break;
		case CHemisphereCfg::NEG_X: //   /**< Target orientation */
			g_heminfo[i] = { CHemisphereCfg::NEG_X, "NEG_X",{ 0, 0, 0, -1.0f, 0.0f, 0.0f } };
			break;
		case CHemisphereCfg::NEG_Y: //   /**< Target orientation */
			g_heminfo[i] = { CHemisphereCfg::NEG_Y, "NEG_Y",{ 0, 0, 0, 0.0f, -1.0f, 0.0f } };
			break;
		case CHemisphereCfg::NEG_Z: //   /**< Target orientation */
			g_heminfo[i] = { CHemisphereCfg::NEG_Z, "NEG_Z",{ 0, 0, 0, 0.0f, 0.0f, -1.0f } };
			break;
		//case CHemisphereCfg::AUTO_HEM: //   /**< Target orientation */
		//	g_heminfo[i] = { CHemisphereCfg::AUTO_HEM, "Auto",{ 1, 1, 0, 0.0f, 0.0f, 0.0f } };
		//	break;
		case CHemisphereCfg::CUSTOM: //   /**< Target orientation */
			g_heminfo[i] = { CHemisphereCfg::CUSTOM, "CUSTOM",{ 0, 0, 0, 0.0f, 0.0f, 0.0f } };
			break;
		//case CHemisphereCfg::TRACK_EN: //   /**< Target orientation */
		//	g_heminfo[i] = { CHemisphereCfg::TRACK_EN, "HEM_TRACKING_ENABLED",{ 1, 0, 0, 0.0f, 0.0f, 0.0f } };
			break;
		}
	}
}

void CVPcmd::InitViperHemisphereInfo()
{
	if (g_vpheminfo[0].ehem != E_VP_HEM_MAX)
		return;

	for (int i = 0; i <= E_VP_HEM_MAX; i++)
	{
		switch (i)
		{
		default:
			g_vpheminfo[i] = { E_VP_HEM_MAX, "E_VP_HEM_MAX" };
			break;
		case E_VP_HEM_POS_X: // = 0 /**< +X */
			g_vpheminfo[i] = { E_VP_HEM_POS_X, "POS_X" };
			break;
		case E_VP_HEM_POS_Y: // = 0 /**< +Y */
			g_vpheminfo[i] = { E_VP_HEM_POS_Y, "POS_Y" };
			break;
		case E_VP_HEM_POS_Z: // = 0 /**< +Z */
			g_vpheminfo[i] = { E_VP_HEM_POS_Z, "POS_Z" };
			break;
		case E_VP_HEM_NEG_X: //   /**< -X */
			g_vpheminfo[i] = { E_VP_HEM_NEG_X, "NEG_X" };
			break;
		case E_VP_HEM_NEG_Y: //   /**< -Y */
			g_vpheminfo[i] = { E_VP_HEM_NEG_Y, "NEG_Y" };
			break;
		case E_VP_HEM_NEG_Z: //   /**< -Z */
			g_vpheminfo[i] = { E_VP_HEM_NEG_Z, "NEG_Z" };
			break;
		case E_VP_HEM_AUTO: //   /**< Auto-Hemisphere */
			g_vpheminfo[i] = { E_VP_HEM_AUTO, "Auto" };
			break;
		}
	}
}

/*inline*/ const HEMISPHERE_CONFIG *CVPcmd::hemispherecfg(uint32_t hemval)
{
	if (g_heminfo[0].et == CHemisphereCfg::HEMPRESETVAL_MAX)
		InitHemisphereInfo();

	return &(g_heminfo[CLAMP(hemval, 0, CHemisphereCfg::HEMPRESETVAL_MAX)].cfg);
}

/*inline*/ const char * CVPcmd::vphemispherestr(uint32_t hemval)
{
	if (g_vpheminfo[0].ehem == E_VP_HEM_MAX)
		InitViperHemisphereInfo();

	return g_vpheminfo[CLAMP(hemval, 0, E_VP_HEM_MAX)].szhem;
}
UNITS_CONFIG CUnitsCfg::Default = { POS_INCH, ORI_EULER_DEGREE };



