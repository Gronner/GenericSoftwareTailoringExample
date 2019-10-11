#ifndef DSPSIGNAL_H
#define DSPSIGNAL_H
#include "tl_basetypes.h"

Float32 Out_accY_g_dsp_value;
Float32 Out_accY_g_dsp_noiseVariance;
Float32 Out_accY_g_dsp_processVariance;
Bool Out_accY_g_dsp_valid;

Float32 Out_angularRateX_RADs_dsp_value;
Float32 Out_angularRateX_RADs_dsp_noiseVariance;
Float32 Out_angularRateX_RADs_dsp_processVariance;
Bool Out_angularRateX_RADs_dsp_valid;

Float32 Out_angularRateY_RADs_dsp_value;
Float32 Out_angularRateY_RADs_dsp_noiseVariance;
Float32 Out_angularRateY_RADs_dsp_processVariance;
Bool Out_angularRateY_RADs_dsp_valid;

Float32 Out_accX_g_dsp_value;
Float32 Out_accX_g_dsp_noiseVariance;
Float32 Out_accX_g_dsp_processVariance;
Bool Out_accX_g_dsp_valid;

// Signals for AttitudeController
Float32 Out_angularRateZ_RADs_dsp;

// Signals for Height Observer
Float32 Out_Altitude_m_dsp_value;
Float32 Out_Altitude_m_dsp_processVariance; 
Bool Out_Altitude_m_dsp_valid;

// Signals for Altitude Observer
Float32 Out_Altitude_m_dsp_value;
Float32 Out_Altitude_m_dsp_noiseVariance;
Float32 Out_Altitude_m_dsp_processVariance;
Bool Out_Altitude_m_dsp_valid;

Float32 Out_AccZ_g_dsp_value;
Float32 Out_AccZ_g_dsp_noiseVariance;
Float32 Out_AccZ_g_dsp_processVariance;
Bool Out_AccZ_g_dsp_valid;
// Signals from remote
Float32 Out_VelocityZ_Ms_rem;
Float32 Out_angleX_RAD_rem;
Float32 Out_angleY_RAD_rem;
Float32 Out_angularRateZ_RADs_rem;
#endif // DSPSIGNAL_H
