#ifndef __TUNER_R820T_H
#define __TUNER_R820T_H

#include "rtl2832.h"

namespace RTL2832_NAMESPACE
{
namespace TUNERS_NAMESPACE
{

class r820t : public RTL2832_NAMESPACE::tuner_skeleton
{
IMPLEMENT_INLINE_TUNER_FACTORY(r820t)
public:
	r820t(demod* p);
public:
	inline virtual const char* name() const
	{ return "Rafael Micro R820T tuner"; }
public:
	int initialise(tuner::PPARAMS params = NULL);
	int set_frequency(double freq);
	int set_bandwidth(double bw);
	int set_gain(double gain);
};

}
}

#define	BORDER_FREQ	2600000	//2.6GHz : The border frequency which determines whether Low VCO or High VCO is used
#define USE_EXT_CLK	0	//0 : Use internal XTAL Oscillator / 1 : Use External Clock input
#define OFS_RSSI 57

#define R820T_I2C_ADDR          0x34
#define R820T_CHECK_ADDR        0x00
#define R820T_CHECK_VAL         0x69

#define R820T_IF_FREQ           3570000

typedef enum {
	R820T_UHF_BAND,
	R820T_L_BAND,
	R820T_VHF_BAND,
	R820T_NO_BAND
} r820t_band_type;

typedef int r820t_fci_result_type;

#define R820T_FCI_FAIL		0
#define R820T_FCI_SUCCESS	1

enum R820T_FUNCTION_STATUS
{
	R820T_FUNCTION_SUCCESS,
	R820T_FUNCTION_ERROR,
};

extern void r820t_wait_msec(RTL2832_NAMESPACE::tuner* pTuner, int a);
/*
r820t_fci_result_type r820t_i2c_write(RTL2832_NAMESPACE::tuner* pTuner, unsigned char reg, unsigned char val);
r820t_fci_result_type r820t_i2c_read(RTL2832_NAMESPACE::tuner* pTuner, unsigned char reg, unsigned char *read_data);
*/
/*==============================================================================
       r820t initial setting

  This function is a generic function which gets called to initialize

  r820t in DVB-H mode or L-Band TDMB mode

  <input parameter>

  ifagc_mode
    type : integer
	1 : Internal AGC
	2 : Voltage Control Mode

==============================================================================*/
r820t_fci_result_type r820t_set_init(RTL2832_NAMESPACE::tuner* pTuner, int ifagc_mode, unsigned int freq_xtal );

/*==============================================================================
       r820t frequency setting

  This function is a generic function which gets called to change LO Frequency

  of r820t in DVB-H mode or L-Band TDMB mode

  <input parameter>

  f_lo
	Value of target LO Frequency in 'kHz' unit
	ex) 2.6GHz = 2600000

==============================================================================*/
r820t_fci_result_type r820t_set_freq(RTL2832_NAMESPACE::tuner* pTuner, unsigned int f_lo, unsigned int freq_xtal );


// The following context is R820T tuner API source code
// Definitions

// Bandwidth mode
enum R820T_BANDWIDTH_MODE
{
	R820T_BANDWIDTH_1530000HZ = 1,
	R820T_BANDWIDTH_6000000HZ = 6,
	R820T_BANDWIDTH_7000000HZ = 7,
	R820T_BANDWIDTH_8000000HZ = 8,
};

// Manipulaing functions
int
r820t_Initialize(
	RTL2832_NAMESPACE::tuner* pTuner
	);

int
r820t_SetRfFreqHz(
	RTL2832_NAMESPACE::tuner* pTuner,
	unsigned long RfFreqHz
	);

// Extra manipulaing functions
int
r820t_SetBandwidthMode(
	RTL2832_NAMESPACE::tuner* pTuner,
	int BandwidthMode
	);

// Tuner LNA
enum R820T_LNA_GAIN_VALUE
{
        R820T_LNA_GAIN_LOW     = 0x00, // -6.3dB
        R820T_LNA_GAIN_MIDDLE  = 0x08, //  7.1dB
        R820T_LNA_GAIN_HIGH_17 = 0x11, // 19.1dB
        R820T_LNA_GAIN_HIGH_19 = 0x10, // 19.7dB
};



//***************************************************************
//*                       INCLUDES.H
//***************************************************************
#define USE_16M_XTAL		FALSE
#define R828_Xtal		28800

#define USE_DIPLEXER		FALSE
#define TUNER_CLK_OUT		TRUE

#ifndef _UINT_X_
#define _UINT_X_ 1
typedef unsigned char  UINT8;
typedef unsigned short UINT16;
typedef unsigned int   UINT32;
#endif

#define VER_NUM  49

#define TRUE	1
#define FALSE	0

#define FUNCTION_SUCCESS	0
#define FUNCTION_ERROR		-1

typedef enum _R828_ErrCode
{
	RT_Success,
	RT_Fail
}R828_ErrCode;

typedef enum _Rafael_Chip_Type  //Don't modify chip list
{
	R828 = 0,
	R828D,
	R828S,
	R820T,
	R820C,
	R620D,
	R620S
}Rafael_Chip_Type;
//----------------------------------------------------------//
//                   R828 Parameter                        //
//----------------------------------------------------------//

extern UINT8 R828_ADDRESS;

#define DIP_FREQ  	  320000
#define IMR_TRIAL    9
#define VCO_pwr_ref   0x02

extern UINT32 R828_IF_khz;
extern UINT32 R828_CAL_LO_khz;
extern UINT8  R828_IMR_point_num;
extern UINT8  R828_IMR_done_flag;
extern UINT8  Rafael_Chip;

typedef enum _R828_Standard_Type  //Don't remove standand list!!
{
	NTSC_MN = 0,
	PAL_I,
	PAL_DK,
	PAL_B_7M,       //no use
	PAL_BGH_8M,     //for PAL B/G, PAL G/H
	SECAM_L,
	SECAM_L1_INV,   //for SECAM L'
	SECAM_L1,       //no use
	ATV_SIZE,
	DVB_T_6M = ATV_SIZE,
	DVB_T_7M,
	DVB_T_7M_2,
	DVB_T_8M,
	DVB_T2_6M,
	DVB_T2_7M,
	DVB_T2_7M_2,
	DVB_T2_8M,
	DVB_T2_1_7M,
	DVB_T2_10M,
	DVB_C_8M,
	DVB_C_6M,
	ISDB_T,
	DTMB,
	R828_ATSC,
	FM,
	STD_SIZE
}R828_Standard_Type;

extern UINT8  R828_Fil_Cal_flag[STD_SIZE];

typedef enum _R828_SetFreq_Type
{
	FAST_MODE = TRUE,
	NORMAL_MODE = FALSE
}R828_SetFreq_Type;

typedef enum _R828_LoopThrough_Type
{
	LOOP_THROUGH = TRUE,
	SIGLE_IN     = FALSE
}R828_LoopThrough_Type;


typedef enum _R828_InputMode_Type
{
	AIR_IN = 0,
	CABLE_IN_1,
	CABLE_IN_2
}R828_InputMode_Type;

typedef enum _R828_IfAgc_Type
{
	IF_AGC1 = 0,
	IF_AGC2
}R828_IfAgc_Type;

typedef enum _R828_GPIO_Type
{
	HI_SIG = TRUE,
	LO_SIG = FALSE
}R828_GPIO_Type;

typedef struct _R828_Set_Info
{
	UINT32        RF_Hz;
	UINT32        RF_KHz;
	R828_Standard_Type R828_Standard;
	R828_LoopThrough_Type RT_Input;
	R828_InputMode_Type   RT_InputMode;
	R828_IfAgc_Type R828_IfAgc_Select; 
}R828_Set_Info;

typedef struct _R828_RF_Gain_Info
{
	UINT8   RF_gain1;
	UINT8   RF_gain2;
	UINT8   RF_gain_comb;
}R828_RF_Gain_Info;

typedef enum _R828_RF_Gain_TYPE
{
	RF_AUTO = 0,
	RF_MANUAL
}R828_RF_Gain_TYPE;

typedef struct _R828_I2C_LEN_TYPE
{
	UINT8 RegAddr;
	UINT8 Data[50];
	UINT8 Len;
}R828_I2C_LEN_TYPE;

typedef struct _R828_I2C_TYPE
{
	UINT8 RegAddr;
	UINT8 Data;
}R828_I2C_TYPE;
//----------------------------------------------------------//
//                   R828 Function                         //
//----------------------------------------------------------//
r820t_fci_result_type R828_Init(RTL2832_NAMESPACE::tuner* pTuner);
r820t_fci_result_type R828_Init(RTL2832_NAMESPACE::tuner* pTuner);
r820t_fci_result_type R828_Standby(RTL2832_NAMESPACE::tuner* pTuner, R828_LoopThrough_Type R828_LoopSwitch);
r820t_fci_result_type R828_GPIO(RTL2832_NAMESPACE::tuner* pTuner, R828_GPIO_Type R828_GPIO_Conrl);
r820t_fci_result_type R828_SetStandard(RTL2832_NAMESPACE::tuner* pTuner, R828_Standard_Type RT_Standard);
r820t_fci_result_type R828_SetFrequency(RTL2832_NAMESPACE::tuner* pTuner, R828_Set_Info R828_INFO, R828_SetFreq_Type R828_SetFreqMode);
r820t_fci_result_type R828_GetRfGain(RTL2832_NAMESPACE::tuner* pTuner, R828_RF_Gain_Info *pR828_rf_gain);
r820t_fci_result_type R828_SetRfGain(RTL2832_NAMESPACE::tuner* pTuner, int gain);
r820t_fci_result_type R828_RfGainMode(RTL2832_NAMESPACE::tuner* pTuner, int manual);

int
r820t_SetRfFreqHz(
	RTL2832_NAMESPACE::tuner* pTuner,
	unsigned long RfFreqHz
	);

int
r820t_SetStandardMode(
	RTL2832_NAMESPACE::tuner* pTuner,
	int StandardMode
	);

int
r820t_SetStandby(
	RTL2832_NAMESPACE::tuner* pTuner,
	int LoopThroughType
	);

#endif /* _R820T_TUNER_H */
