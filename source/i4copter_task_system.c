#include "osek.h"
#include "AttitudeController.h"
#include "AltitudeController.h"
#include "AltitudeObserver.h"
#include "AttitudeObserver.h"
#include "HeightObserver.h"
#include "EngineController.h"
#include "dsp_signals.h"

/* Task Declarations */
DeclareTask(T1_Controllers);
DeclareTask(T2_EngineController);
DeclareTask(T3_AttitudeObserver);
DeclareTask(T4_HeightObserver);
DeclareTask(T5_AltitudeObserver);
DeclareTask(T6_DSP);
/* Idle Tasks */
DeclareTask(IdleTask_OsCore_Core0);
DeclareTask(IdleTask_OsCore_Core1);
DeclareTask(IdleTask_OsCore_Core2);

// Global Remote variables
static float Out_VelocityZ_Ms_rem;
static float Out_angleX_RAD_rem;
static float Out_angleY_RAD_rem;
static float Out_angularRateZ_RADs_rem;

// Global DSP variables

static float random_value(float min, float max) {
    static int i = -1;
    i *= -1;
    return i * 5.f; // Dummy Random
}

TASK(T1_Controllers) {
    static Float32 DiscreteIntegrator_wrapper = 0.F;
    
    // Inputs for AltitudeController
    In_altCtr_Altitude_m_rem = DiscreteIntegrator_wrapper;
    DiscreteIntegrator_wrapper += 0.009F * In_altCtr_VelocityZ_Ms_rem;
    In_altCtr_VelocityZ_Ms_rem = Out_VelocityZ_Ms_rem;
    In_altCtr_AccZ_g_rem = 0.F;
    In_altCtr_Altitude_m_altObs_value = Out_Altitude_m_altObs_value;
    In_altCtr_Altitude_m_altObs_noiseVariance = Out_Altitude_m_altObs_noiseVariance;
    In_altCtr_Altitude_m_altObs_processVariance = Out_Altitude_m_altObs_processVariance;
    In_altCtr_Altitude_m_altObs_valid = Out_Altitude_m_altObs_valid;
    In_altCtr_VelocityZ_Ms_altObs_value = Out_VelocityZ_Ms_altObs_value;
    In_altCtr_VelocityZ_Ms_altObs_noiseVariance = Out_VelocityZ_Ms_altObs_noiseVariance;
    In_altCtr_VelocityZ_Ms_altObs_processVariance = Out_VelocityZ_Ms_altObs_processVariance;
    In_altCtr_VelocityZ_Ms_altObs_valid = Out_VelocityZ_Ms_altObs_valid;
    In_altCtr_AccZ_g_altObs_value = Out_AccZ_Mss_altObs_noiseVariance;
    In_altCtr_AccZ_g_altObs_noiseVariance = Out_AccZ_Mss_altObs_processVariance;
    In_altCtr_AccZ_g_altObs_processVariance = Out_AccZ_Mss_altObs_value;
    In_altCtr_AccZ_g_altObs_valid = Out_AccZ_Mss_altObs_valid;
    // Call Altitude Controller
    STEP_AltitudeController();

    // Inputs for AttitudeController
    In_attCtr_thrustZ_g_rem = 1.F - Out_ThrustZ_g_altCon;
    In_attCtr_angleX_RAD_rem = Out_angleX_RAD_rem;
    In_attCtr_angleY_RAD_rem = Out_angleY_RAD_rem;
    In_attCtr_angularRateZ_RADs_rem = Out_angularRateZ_RADs_rem;
    In_attCtr_statesY_attObs_angularState1 = Out_angularStatesY_attObs_angularState1;
    In_attCtr_statesY_attObs_angularState2 = Out_angularStatesY_attObs_angularState2;
    In_attCtr_statesY_attObs_angularRate = Out_angularStatesY_attObs_angularRate;
    In_attCtr_statesY_attObs_angle = Out_angularStatesY_attObs_angle;
    In_attCtr_statesX_attObs_angularState1 = Out_angularStatesX_attObs_angularState1;
    In_attCtr_statesX_attObs_angularState2 = Out_angularStatesX_attObs_angularState2;
    In_attCtr_statesX_attObs_angularRate = Out_angularStatesX_attObs_angularRate;
    In_attCtr_statesX_attObs_angle = Out_angularStatesX_attObs_angle;
    In_attCtr_angularRateZ_RADs_dsp = Out_angularRateZ_RADs_dsp;
    In_attCtr_takeOff_engCtr = Out_takeOff_engCtr;
    // Call AttitudeController
    STEP_AttitudeController();

    TerminateTask();
}

TASK(T2_EngineController) {
    // Inputs for EngineController
    In_engCtr_thrustZ_N_attCtr = Out_thrustZ_N_attCtr;
    In_engCtr_torqueX_NM_attCtr = Out_torqueX_NM_attCtr;
    In_engCtr_torqueY_NM_attCtr = Out_torqueY_NM_attCtr;
    In_engCtr_torqueZ_NM_attCtr = Out_torqueZ_NM_attCtr;
    In_engCtr_takeoff_g_sw = 1.F;
    // Call EngineController
    STEP_EngineController();

    TerminateTask();
}

TASK(T3_AttitudeObserver) {
    // Set Inputs for AltitudeObserver
    In_attObs_takeOff_engCtr = Out_takeOff_engCtr;
    In_attObs_torqueY_NM_engCtr = Out_torqueY_NM_engCtr;
    In_attObs_torqueX_NM_engCtr = Out_torqueX_NM_engCtr;
    In_attObs_accY_g_dsp_value = Out_accY_g_dsp_value;
    In_attObs_accY_g_dsp_noiseVariance = Out_accY_g_dsp_noiseVariance;
    In_attObs_accY_g_dsp_processVariance = Out_accY_g_dsp_processVariance;
    In_attObs_accY_g_dsp_valid = Out_accY_g_dsp_valid;
    In_attObs_angularRateY_RADs_dsp_value = Out_angularRateY_RADs_dsp_value;
    In_attObs_angularRateY_RADs_dsp_noiseVariance = Out_angularRateY_RADs_dsp_noiseVariance;
    In_attObs_angularRateY_RADs_dsp_processVariance = Out_angularRateY_RADs_dsp_processVariance;
    In_attObs_angularRateY_RADs_dsp_valid = Out_angularRateY_RADs_dsp_valid;
    In_attObs_accX_g_dsp_value = Out_accX_g_dsp_value;
    In_attObs_accX_g_dsp_noiseVariance = Out_accX_g_dsp_noiseVariance;
    In_attObs_accX_g_dsp_processVariance = Out_accX_g_dsp_processVariance;

    // Call AttitudeObserver
    STEP_AttitudeObserver();

    TerminateTask();
}

TASK(T4_HeightObserver) {
    // Set Inputs for HeightObserver
    In_heiObs_Altitude_m_dsp_value = Out_Altitude_m_dsp_value;
    In_heiObs_AngleX_RAD_attObs = Out_angleX_RAD_attObs;
    In_heiObs_AngleY_RAD_attObs = Out_angleY_RAD_attObs;
    In_heiObs_ThrustZ_N_engCtr = Out_thrustZ_N_engCtr;
    In_heiObs_TakeOff_engCtr = Out_takeOff_engCtr;
    // Call HeightObserver
    STEP_HeightObserver();

    TerminateTask();
}

TASK(T5_AltitudeObserver) {
    // Set Inputs for AltitudeObserver
    In_altObs_AngleX_RAD_attObs = Out_angleX_RAD_attObs;
    In_altObs_AngleY_RAD_attObs = Out_angleY_RAD_attObs;
    In_altObs_ThrustZ_N_engCtr = Out_thrustZ_N_engCtr;
    In_altObs_TakeOff_engCtr = Out_takeOff_engCtr;
    In_altObs_Altitude_m_dsp_value = Out_Altitude_m_dsp_value;
    In_altObs_Altitude_m_dsp_noiseVariance = Out_Altitude_m_dsp_noiseVariance;
    In_altObs_Altitude_m_dsp_processVariance = Out_Altitude_m_dsp_processVariance;
    In_altObs_Altitude_m_dsp_valid = Out_Altitude_m_dsp_valid;
    In_altObs_AccZ_g_dsp_value = Out_AccZ_g_dsp_value;
    In_altObs_AccZ_g_dsp_noiseVariance = Out_AccZ_g_dsp_noiseVariance;
    In_altObs_AccZ_g_dsp_processVariance = Out_AccZ_g_dsp_processVariance;
    In_altObs_AccZ_g_dsp_valid = Out_AccZ_g_dsp_valid;

    // Call AltitudeObserver
    STEP_AltitudeObserver();

    TerminateTask();
}

TASK(T6_DSP) {
    // Signals for Attitude Observer
    Out_accY_g_dsp_value = random_value(0.f, 100.f);
    Out_accY_g_dsp_noiseVariance = random_value(0.f, 100.f);
    Out_accY_g_dsp_processVariance = random_value(0.f, 100.f);
    Out_accY_g_dsp_valid = 1;
    
    Out_angularRateX_RADs_dsp_value = random_value(0.f, 100.f);
    Out_angularRateX_RADs_dsp_noiseVariance= random_value(0.f, 100.f);
    Out_angularRateX_RADs_dsp_processVariance = random_value(0.f, 100.f);
    Out_angularRateX_RADs_dsp_valid = 1;

    Out_angularRateY_RADs_dsp_value = random_value(0.f, 100.f);
    Out_angularRateY_RADs_dsp_noiseVariance = random_value(0.f, 100.f);
    Out_angularRateY_RADs_dsp_processVariance = random_value(0.f, 100.f);
    Out_angularRateY_RADs_dsp_valid = 1;

    Out_accX_g_dsp_value = random_value(0.f, 100.f);
    Out_accX_g_dsp_noiseVariance = random_value(0.f, 100.f);
    Out_accX_g_dsp_processVariance = random_value(0.f, 100.f);
    Out_accX_g_dsp_valid = 1;

    // Signals for AttitudeController
    Out_angularRateZ_RADs_dsp = random_value(0.f, 100.f);

    // Signals for Height Observer
    Out_Altitude_m_dsp_value = random_value(0.f, 100.f);
    Out_Altitude_m_dsp_processVariance = random_value(0.f, 100.f);
    Out_Altitude_m_dsp_valid = 1;

    // Signals for Altitude Observer
    Out_Altitude_m_dsp_value = random_value(0.f, 100.f);
    Out_Altitude_m_dsp_noiseVariance = random_value(0.f, 100.f);
    Out_Altitude_m_dsp_processVariance = random_value(0.f, 100.f);
    Out_Altitude_m_dsp_valid = 1;

    Out_AccZ_g_dsp_value = random_value(0.f, 100.f);
    Out_AccZ_g_dsp_noiseVariance = random_value(0.f, 100.f);
    Out_AccZ_g_dsp_processVariance = random_value(0.f, 100.f);
    Out_AccZ_g_dsp_valid = 1;
    // Signals from remote
    Out_VelocityZ_Ms_rem = random_value(0.f, 100.f);
    Out_angleX_RAD_rem = random_value(0.f, 100.f);
    Out_angleY_RAD_rem = random_value(0.f, 100.f);
    Out_angularRateZ_RADs_rem = random_value(0.f, 100.f);
    
    TerminateTask();
}

TASK(IdleTask_OsCore_Core0) {
    TerminateTask();
}

TASK(IdleTask_OsCore_Core1) {
    TerminateTask();
}

TASK(IdleTask_OsCore_Core2) {
    TerminateTask();
}

ISR (XSignalIsr_OsCore_Core0) {
}

ISR (XSignalIsr_OsCore_Core1) {
}

ISR (XSignalIsr_OsCore_Core2) {
}

ISR (CounterIsr_SystemTimer) {
}

ISR (CounterIsr_SystemTimer_0) {
}

ISR (CounterIsr_SystemTimer_1) {
}
