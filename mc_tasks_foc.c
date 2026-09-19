/**
  ******************************************************************************
  * @file    mc_tasks_foc.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file implements tasks definition
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
//cstat -MISRAC2012-Rule-21.1
#include "main.h"
//cstat +MISRAC2012-Rule-21.1
#include "mc_type.h"
#include "mc_math.h"
#include "motorcontrol.h"
#include "regular_conversion_manager.h"
#include "mc_interface.h"
#include "digital_output.h"
#include "pwm_common.h"
#include "mc_tasks.h"
#include "parameters_conversion.h"
#include "mcp_config.h"
#include "mc_app_hooks.h"

/* USER CODE BEGIN Includes */
#include "ann_1layer.h"
/* USER CODE END Includes */

/* USER CODE BEGIN Private define */
/* Private define ------------------------------------------------------------*/

/* USER CODE END Private define */

/* Private variables----------------------------------------------------------*/

static volatile uint16_t hBootCapDelayCounterM1 = ((uint16_t)0);
static volatile uint16_t hStopPermanencyCounterM1 = ((uint16_t)0);

#define M1_CHARGE_BOOT_CAP_TICKS          (((uint16_t)SYS_TICK_FREQUENCY * (uint16_t)10) / 1000U)
#define M1_CHARGE_BOOT_CAP_DUTY_CYCLES (uint32_t)(0.000 * ((uint32_t)PWM_PERIOD_CYCLES / 2U))
#define M2_CHARGE_BOOT_CAP_TICKS         (((uint16_t)SYS_TICK_FREQUENCY * (uint16_t)10) / 1000U)
#define M2_CHARGE_BOOT_CAP_DUTY_CYCLES (uint32_t)(0 * ((uint32_t)PWM_PERIOD_CYCLES2 / 2U))

/* USER CODE BEGIN Private Variables */

/* ===================== ANN full-replacement config ======================== */
/* Master switch: comment out to fall back to the stock PI Iq controller.     */
#define USE_ANN_CONTROLLER

/* Vq scale: converts ANN output (Volts) -> MCSDK internal s16 voltage units. */
#define VOLTAGE_SCALE_F      1171.0f

/* Max change of Vqd.q allowed per high-frequency tick (internal s16 units).  */
#define ANN_VQ_SLEW_MAX      50.0f

/*
 * CURRENT_CONV_FACTOR_INV converts MCSDK internal s16A current units -> Amps.
 * It is already defined in parameters_conversion.h (included via main.h), so
 * we use that real value directly - do NOT redefine it here. The ANN was
 * trained on Amps, and MotorPilot logs use this same scaling, so the value
 * from parameters_conversion.h is exactly what we want.
 */

/* Cached ANN output, computed at medium frequency, consumed at high frequency */
static volatile float   g_vq_cached_full     = 0.0f;
static volatile uint8_t g_ann_full_active    = 0U;
static float            g_vq_output_prev_full = 0.0f;

/* Debug globals - readable from Live Expressions / breakpoints */
static volatile float   g_ann_dbg_in[4]      = {0.0f, 0.0f, 0.0f, 0.0f};
static volatile float   g_ann_dbg_vq_pred_raw = 0.0f;

/* USER CODE END Private Variables */

/* Private functions ---------------------------------------------------------*/
void TSK_MediumFrequencyTaskM1(void);
void FOC_InitAdditionalMethods(uint8_t bMotor);
void FOC_CalcCurrRef(uint8_t bMotor);
void TSK_MF_StopProcessing(uint8_t motor);

MCI_Handle_t *GetMCI(uint8_t bMotor);
static uint16_t FOC_CurrControllerM1(void);

void TSK_SafetyTask_PWMOFF(uint8_t motor);

/* USER CODE BEGIN Private Functions */
#ifdef USE_ANN_CONTROLLER
static void ANN_UpdateVqPrediction_Full(uint8_t bMotor);
#endif
/* USER CODE END Private Functions */
/**
  * @brief  It initializes the whole MC core according to user defined
  *         parameters.
  */
__weak void FOC_Init(void)
{

  /* USER CODE BEGIN MCboot 0 */

  /* USER CODE END MCboot 0 */

    /**********************************************************/
    /*    PWM and current sensing component initialization    */
    /**********************************************************/
    pwmcHandle[M1] = &PWM_Handle_M1._Super;
    R3_2_Init(&PWM_Handle_M1);

    /* USER CODE BEGIN MCboot 1 */

    /* USER CODE END MCboot 1 */

    /******************************************************/
    /*   PID component initialization: speed regulation   */
    /******************************************************/
    PID_HandleInit(&PIDSpeedHandle_M1);

    /******************************************************/
    /*   Main speed sensor component initialization       */
    /******************************************************/
    ENC_Init (&ENCODER_M1);

    /******************************************************/
    /*   Main encoder alignment component initialization  */
    /******************************************************/
    EAC_Init(&EncAlignCtrlM1,pSTC[M1],&VirtualSpeedSensorM1,&ENCODER_M1);
    pEAC[M1] = &EncAlignCtrlM1;

    /******************************************************/
    /*   Speed & torque component initialization          */
    /******************************************************/
    STC_Init(pSTC[M1],&PIDSpeedHandle_M1, &ENCODER_M1._Super);

    /********************************************************/
    /*   PID component initialization: current regulation   */
    /********************************************************/
    PID_HandleInit(&PIDIqHandle_M1);
    PID_HandleInit(&PIDIdHandle_M1);

    /*************************************************/
    /*   Power measurement component initialization  */
    /*************************************************/
    pMPM[M1]->pVBS = &(BusVoltageSensor_M1._Super);
    pMPM[M1]->pFOCVars = &FOCVars[M1];

    /*******************************************************/
    /*   Flux weakening component initialization           */
    /*******************************************************/
    PID_HandleInit(&PIDFluxWeakeningHandle_M1);
    FW_Init(pFW[M1],&PIDSpeedHandle_M1,&PIDFluxWeakeningHandle_M1);

    pREMNG[M1] = &RampExtMngrHFParamsM1;
    REMNG_Init(pREMNG[M1]);

    FOC_Clear(M1);
    STC_Clear(pSTC[M1]);
    FOCVars[M1].bDriveInput = EXTERNAL;
    FOCVars[M1].Iqdref = STC_GetDefaultIqdref(pSTC[M1]);
    FOCVars[M1].UserIdref = STC_GetDefaultIqdref(pSTC[M1]).d;

    MCI_ExecSpeedRamp(&Mci[M1],
    STC_GetMecSpeedRefUnitDefault(pSTC[M1]),0); /* First command to STC */

    /* USER CODE BEGIN MCboot 2 */

    /* USER CODE END MCboot 2 */
}

/**
 * @brief Performs stop process and update the state machine.This function
 *        shall be called only during medium frequency task.
 */
void TSK_MF_StopProcessing(uint8_t motor)
{
  R3_2_SwitchOffPWM(pwmcHandle[motor]);

  FOC_Clear(motor);
  STC_Clear(pSTC[motor]);

  TSK_SetStopPermanencyTimeM1(STOPPERMANENCY_TICKS);
  Mci[motor].State = STOP;
}

/**
  * @brief Executes medium frequency periodic Motor Control tasks
  *
  * This function performs some of the control duties on Motor 1 according to the
  * present state of its state machine. In particular, duties requiring a periodic
  * execution at a medium frequency rate (such as the speed controller for instance)
  * are executed here.
  */
__weak void TSK_MediumFrequencyTaskM1(void)
{
  /* USER CODE BEGIN MediumFrequencyTask M1 0 */

  /* USER CODE END MediumFrequencyTask M1 0 */

  int16_t wAux = 0;
  (void)ENC_CalcAvrgMecSpeedUnit(&ENCODER_M1, &wAux);
  PQD_CalcElMotorPower(pMPM[M1]);

  if (MCI_GetCurrentFaults(&Mci[M1]) == MC_NO_FAULTS)
  {
    if (MCI_GetOccurredFaults(&Mci[M1]) == MC_NO_FAULTS)
    {
      switch (Mci[M1].State)
      {

        case IDLE:
        {
          if ((MCI_START == Mci[M1].DirectCommand) || (MCI_MEASURE_OFFSETS == Mci[M1].DirectCommand))
          {
            if (pwmcHandle[M1]->offsetCalibStatus == false)
            {
              (void)PWMC_CurrentReadingCalibr(pwmcHandle[M1], CRC_START);
              Mci[M1].State = OFFSET_CALIB;
            }
            else
            {
              /* Calibration already done. Enables only TIM channels */
              pwmcHandle[M1]->OffCalibrWaitTimeCounter = 1u;
              (void)PWMC_CurrentReadingCalibr(pwmcHandle[M1], CRC_EXEC);

              R3_2_TurnOnLowSides(pwmcHandle[M1],M1_CHARGE_BOOT_CAP_DUTY_CYCLES);
              TSK_SetChargeBootCapDelayM1(M1_CHARGE_BOOT_CAP_TICKS);
              Mci[M1].State = CHARGE_BOOT_CAP;
            }
          }
          else
          {
            /* Nothing to be done, FW stays in IDLE state */
          }
          break;
        }

        case OFFSET_CALIB:
        {
          if (MCI_STOP == Mci[M1].DirectCommand)
          {
            TSK_MF_StopProcessing(M1);
          }
          else
          {
            if (PWMC_CurrentReadingCalibr(pwmcHandle[M1], CRC_EXEC))
            {
              if (MCI_MEASURE_OFFSETS == Mci[M1].DirectCommand)
              {
                FOC_Clear(M1);
                STC_Clear(pSTC[M1]);
                Mci[M1].DirectCommand = MCI_NO_COMMAND;
                Mci[M1].State = IDLE;
              }
              else
              {
                R3_2_TurnOnLowSides(pwmcHandle[M1],M1_CHARGE_BOOT_CAP_DUTY_CYCLES);
                TSK_SetChargeBootCapDelayM1(M1_CHARGE_BOOT_CAP_TICKS);
                Mci[M1].State = CHARGE_BOOT_CAP;
              }
            }
            else
            {
              /* Nothing to be done, FW waits for offset calibration to finish */
            }
          }
          break;
        }

        case CHARGE_BOOT_CAP:
        {
          if (MCI_STOP == Mci[M1].DirectCommand)
          {
            TSK_MF_StopProcessing(M1);
          }
          else
          {
            if (TSK_ChargeBootCapDelayHasElapsedM1())
            {
              R3_2_SwitchOffPWM(pwmcHandle[M1]);
              FOCVars[M1].bDriveInput = EXTERNAL;
              STC_SetSpeedSensor( pSTC[M1], &VirtualSpeedSensorM1._Super );

              ENC_Clear(&ENCODER_M1);

              FOC_Clear( M1 );

              if (EAC_IsAligned(&EncAlignCtrlM1) == false)
              {
                EAC_StartAlignment(&EncAlignCtrlM1);
                Mci[M1].State = ALIGNMENT;
              }
              else
              {
                STC_SetControlMode(pSTC[M1], MCM_SPEED_MODE);
                STC_SetSpeedSensor(pSTC[M1], &ENCODER_M1._Super);
                FOC_InitAdditionalMethods(M1);
                FOC_CalcCurrRef(M1);
                STC_ForceSpeedReferenceToCurrentSpeed(pSTC[M1]); /* Init the reference speed to current speed */
                MCI_ExecBufferedCommands(&Mci[M1]); /* Exec the speed ramp after changing of the speed sensor */
                Mci[M1].State = RUN;
              }
              PWMC_SwitchOnPWM(pwmcHandle[M1]);
            }
            else
            {
              /* Nothing to be done, FW waits for bootstrap capacitor to charge */
            }
          }
          break;
        }

        case ALIGNMENT:
        {
          if (MCI_STOP == Mci[M1].DirectCommand)
          {
            TSK_MF_StopProcessing(M1);
          }
          else
          {
            bool isAligned = EAC_IsAligned(&EncAlignCtrlM1);
            bool EACDone = EAC_Exec(&EncAlignCtrlM1);
            if ((isAligned == false)  && (EACDone == false))
            {
              qd_t IqdRef;
              IqdRef.q = 0;
              IqdRef.d = STC_CalcTorqueReference(pSTC[M1]);
              FOCVars[M1].Iqdref = IqdRef;
            }
            else
            {
              R3_2_SwitchOffPWM( pwmcHandle[M1] );
              STC_Clear(pSTC[M1]);
              STC_SetControlMode(pSTC[M1], MCM_SPEED_MODE);
              STC_SetSpeedSensor(pSTC[M1], &ENCODER_M1._Super);
              FOC_Clear(M1);
              R3_2_TurnOnLowSides(pwmcHandle[M1],M1_CHARGE_BOOT_CAP_DUTY_CYCLES);
              TSK_SetStopPermanencyTimeM1(STOPPERMANENCY_TICKS);
              Mci[M1].State = WAIT_STOP_MOTOR;
              /* USER CODE BEGIN MediumFrequencyTask M1 EndOfEncAlignment */

              /* USER CODE END MediumFrequencyTask M1 EndOfEncAlignment */
            }
          }
          break;
        }

        case RUN:
        {
          if (MCI_STOP == Mci[M1].DirectCommand)
          {
            TSK_MF_StopProcessing(M1);
          }
          else
          {
            /* USER CODE BEGIN MediumFrequencyTask M1 2 */

            /* USER CODE END MediumFrequencyTask M1 2 */

            MCI_ExecBufferedCommands(&Mci[M1]);

              FOC_CalcCurrRef(M1);

            /* USER CODE BEGIN MediumFrequencyTask M1 RUN ANN */
#ifdef USE_ANN_CONTROLLER
            /* Run the ANN here (1 kHz, ~1000us budget) rather than inside the
               16 kHz high-frequency ISR - the tansig expf() calls are far too
               slow for the 62.5us HF budget and would trip MC_DURATION. */
            ANN_UpdateVqPrediction_Full(M1);
#endif
            /* USER CODE END MediumFrequencyTask M1 RUN ANN */
          }
          break;
        }

        case STOP:
        {
          if (TSK_StopPermanencyTimeHasElapsedM1())
          {

            /* USER CODE BEGIN MediumFrequencyTask M1 5 */

            /* USER CODE END MediumFrequencyTask M1 5 */
            Mci[M1].DirectCommand = MCI_NO_COMMAND;
            Mci[M1].State = IDLE;
          }
          else
          {
            /* Nothing to do, FW waits for to stop */
          }
          break;
        }

        case FAULT_OVER:
        {
          if (MCI_ACK_FAULTS == Mci[M1].DirectCommand)
          {
            Mci[M1].DirectCommand = MCI_NO_COMMAND;
            Mci[M1].State = IDLE;
          }
          else
          {
            /* Nothing to do, FW stays in FAULT_OVER state until acknowledgement */
          }
          break;
        }

        case FAULT_NOW:
        {
          Mci[M1].State = FAULT_OVER;
          break;
        }

        case WAIT_STOP_MOTOR:
        {
          if (MCI_STOP == Mci[M1].DirectCommand)
          {
            TSK_MF_StopProcessing(M1);
          }
          else
          {
            if (TSK_StopPermanencyTimeHasElapsedM1())
            {
              ENC_Clear(&ENCODER_M1);
              R3_2_SwitchOnPWM(pwmcHandle[M1]);
              FOC_InitAdditionalMethods(M1);
              STC_ForceSpeedReferenceToCurrentSpeed(pSTC[M1]); /* Init the reference speed to current speed */
              MCI_ExecBufferedCommands(&Mci[M1]); /* Exec the speed ramp after changing of the speed sensor */
              FOC_CalcCurrRef(M1);
              Mci[M1].State = RUN;
            }
            else
            {
              /* Nothing to do */
            }
          }
          break;
        }

        default:
          break;
       }
    }
    else
    {
      Mci[M1].State = FAULT_OVER;
    }
  }
  else
  {
    Mci[M1].State = FAULT_NOW;
  }
  /* USER CODE BEGIN MediumFrequencyTask M1 6 */

  /* USER CODE END MediumFrequencyTask M1 6 */
}

/**
  * @brief  It re-initializes the current and voltage variables. Moreover
  *         it clears qd currents PI controllers, voltage sensor and SpeednTorque
  *         controller. It must be called before each motor restart.
  *         It does not clear speed sensor.
  * @param  bMotor related motor it can be M1 or M2.
  */
__weak void FOC_Clear(uint8_t bMotor)
{
  /* USER CODE BEGIN FOC_Clear 0 */

  /* USER CODE END FOC_Clear 0 */

  ab_t NULL_ab = {((int16_t)0), ((int16_t)0)};
  qd_t NULL_qd = {((int16_t)0), ((int16_t)0)};
  alphabeta_t NULL_alphabeta = {((int16_t)0), ((int16_t)0)};

  FOCVars[bMotor].Iab = NULL_ab;
  FOCVars[bMotor].Ialphabeta = NULL_alphabeta;
  FOCVars[bMotor].Iqd = NULL_qd;
    FOCVars[bMotor].Iqdref = NULL_qd;
  FOCVars[bMotor].hTeref = (int16_t)0;
  FOCVars[bMotor].Vqd = NULL_qd;
  FOCVars[bMotor].Valphabeta = NULL_alphabeta;
  FOCVars[bMotor].hElAngle = (int16_t)0;

  PID_SetIntegralTerm(pPIDIq[bMotor], ((int32_t)0));
  PID_SetIntegralTerm(pPIDId[bMotor], ((int32_t)0));

  PWMC_SwitchOffPWM(pwmcHandle[bMotor]);

  if (NULL == pFW[bMotor])
  {
    /* Nothing to do */
  }
  else
  {
    FW_Clear(pFW[bMotor]);
  }

  MC_Perf_Clear(&PerfTraces,bMotor);
  /* USER CODE BEGIN FOC_Clear 1 */
#ifdef USE_ANN_CONTROLLER
  /* Reset the ANN pipeline too, otherwise a stale Vq from the previous run
     would be applied on the first HF tick after restart. */
  g_vq_cached_full      = 0.0f;
  g_vq_output_prev_full = 0.0f;
  g_ann_full_active     = 0U;
#endif
  /* USER CODE END FOC_Clear 1 */
}

/**
  * @brief  Use this method to initialize additional methods (if any) in
  *         START_TO_RUN state.
  * @param  bMotor related motor it can be M1 or M2.
  */
__weak void FOC_InitAdditionalMethods(uint8_t bMotor) //cstat !RED-func-no-effect
{
    if (M_NONE == bMotor)
    {
      /* Nothing to do */
    }
    else
    {
  /* USER CODE BEGIN FOC_InitAdditionalMethods 0 */

  /* USER CODE END FOC_InitAdditionalMethods 0 */
    }
}

/**
  * @brief  It computes the new values of Iqdref (current references on qd
  *         reference frame) based on the required electrical torque information
  *         provided by oTSC object (internally clocked).
  *         If implemented in the derived class it executes flux weakening and/or
  *         MTPA algorithm(s). It must be called with the periodicity specified
  *         in oTSC parameters.
  * @param  bMotor related motor it can be M1 or M2.
  */
__weak void FOC_CalcCurrRef(uint8_t bMotor)
{
  qd_t IqdTmp;

  /* Enter critical section */
  /* Disable interrupts to avoid any interruption during Iqd reference latching */
  /* to avoid MF task writing them while HF task reading them */
  __disable_irq();
  IqdTmp = FOCVars[bMotor].Iqdref;

  /* Exit critical section */
  __enable_irq();

  /* USER CODE BEGIN FOC_CalcCurrRef 0 */

  /* USER CODE END FOC_CalcCurrRef 0 */
  if (INTERNAL == FOCVars[bMotor].bDriveInput)
  {
    FOCVars[bMotor].hTeref = STC_CalcTorqueReference(pSTC[bMotor]);
    IqdTmp.q = FOCVars[bMotor].hTeref;

    if (NULL == pFW[bMotor])
    {
      /* Nothing to do */
    }
    else
    {
      IqdTmp.d = FOCVars[bMotor].UserIdref;
      IqdTmp = FW_CalcCurrRef(pFW[bMotor], IqdTmp);
    }
  }
  else
  {
    /* Nothing to do */
  }

  /* Enter critical section */
  /* Disable interrupts to avoid any interruption during Iqd reference restoring */
  __disable_irq();
  FOCVars[bMotor].Iqdref = IqdTmp;

  /* Exit critical section */
  __enable_irq();
  /* USER CODE BEGIN FOC_CalcCurrRef 1 */

  /* USER CODE END FOC_CalcCurrRef 1 */
}

/* USER CODE BEGIN mc_task ANN_Full */
#ifdef USE_ANN_CONTROLLER
/**
  * @brief  Runs the ANN that replaces the Iq PI current controller.
  *
  *         Called from TSK_MediumFrequencyTaskM1() at 1 kHz. The predicted Vq
  *         is cached in g_vq_cached_full and consumed by FOC_CurrControllerM1()
  *         at 16 kHz, which only clamps and slew-limits it.
  *
  * @param  bMotor related motor, M1 here.
  */
static void ANN_UpdateVqPrediction_Full(uint8_t bMotor)
{
  qd_t IqdSnapshot;
  qd_t IqdrefSnapshot;
  int16_t speedSnapshot;
  float i_d_meas;
  float i_q_meas;
  float i_q_ref;
  float speed;
  float vq_pred;

  /* Critical section: Iqd is written at the end of the HF ISR, so read both
     Iqd and Iqdref atomically to keep the ANN inputs mutually consistent. */
  __disable_irq();
  IqdSnapshot    = FOCVars[bMotor].Iqd;
  IqdrefSnapshot = FOCVars[bMotor].Iqdref;
  __enable_irq();

  /* Read the already-computed average speed instead of calling
     ENC_CalcAvrgMecSpeedUnit() again - a second call in the same MF cycle
     would advance the encoder's internal averaging buffer. */
  speedSnapshot = SPD_GetAvrgMecSpeedUnit(&ENCODER_M1._Super);

  i_d_meas = (float)IqdSnapshot.d    * CURRENT_CONV_FACTOR_INV;
  i_q_meas = (float)IqdSnapshot.q    * CURRENT_CONV_FACTOR_INV;
  i_q_ref  = (float)IqdrefSnapshot.q * CURRENT_CONV_FACTOR_INV;
  speed    = (float)speedSnapshot;

  /* Input order must match INPUT_NAMES in the MATLAB training script:
     {I_D_MEAS, I_Q_MEAS, I_Q_REF, ENCODER_SPEED} */
  vq_pred = ANN_1L_Predict(i_d_meas, i_q_meas, i_q_ref, speed);

  /* Debug capture - inspect these at a breakpoint to tell a bad prediction
     apart from bad/stale input data. */
  g_ann_dbg_in[0] = i_d_meas;
  g_ann_dbg_in[1] = i_q_meas;
  g_ann_dbg_in[2] = i_q_ref;
  g_ann_dbg_in[3] = speed;
  g_ann_dbg_vq_pred_raw = vq_pred;

  if (ANN_1L_IsLastPredictionValid() != 0U)
  {
    g_vq_cached_full  = vq_pred * VOLTAGE_SCALE_F;
    g_ann_full_active = 1U;
  }
  else
  {
    /* NaN/Inf caught - let the HF loop fall back to the PI controller. */
    g_ann_full_active = 0U;
  }
}
#endif /* USE_ANN_CONTROLLER */
/* USER CODE END mc_task ANN_Full */

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif

/**
  * @brief  Executes the Motor Control duties that require a high frequency rate and a precise timing.
  *
  *  This is mainly the FOC current control loop. It is executed depending on the state of the Motor Control
  * subsystem (see the state machine(s)).
  * @param bMotorNbr Motor reference number defined
  * @retval Number of the  motor instance which FOC loop was executed.
  */
__weak uint8_t FOC_HighFrequencyTask(uint8_t bMotorNbr)
{
  uint16_t hFOCreturn;
  /* USER CODE BEGIN HighFrequencyTask 0 */

  /* USER CODE END HighFrequencyTask 0 */

  RCM_ReadOngoingConv();
  RCM_ExecNextConv();

  (void)ENC_CalcAngle(&ENCODER_M1);   /* If not sensorless then 2nd parameter is MC_NULL */

  /* USER CODE BEGIN HighFrequencyTask SINGLEDRIVE_1 */

  /* USER CODE END HighFrequencyTask SINGLEDRIVE_1 */
  hFOCreturn = FOC_CurrControllerM1();
  /* USER CODE BEGIN HighFrequencyTask SINGLEDRIVE_2 */

  /* USER CODE END HighFrequencyTask SINGLEDRIVE_2 */
  if(hFOCreturn == MC_DURATION)
  {
    MCI_FaultProcessing(&Mci[M1], MC_DURATION, 0);
  }
  else
  {
    /* USER CODE BEGIN HighFrequencyTask SINGLEDRIVE_3 */

    /* USER CODE END HighFrequencyTask SINGLEDRIVE_3 */
  }

  return (bMotorNbr);

}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
/**
  * @brief It executes the core of FOC drive that is the controllers for Iqd
  *        currents regulation. Reference frame transformations are carried out
  *        accordingly to the active speed sensor. It must be called periodically
  *        when new motor currents have been converted
  * @param this related object of class CFOC.
  * @retval int16_t It returns MC_NO_FAULTS if the FOC has been ended before
  *         next PWM Update event, MC_DURATION otherwise
  */
inline uint16_t FOC_CurrControllerM1(void)
{
  qd_t Iqd, Vqd;
  ab_t Iab;
  alphabeta_t Ialphabeta, Valphabeta;
  int16_t hElAngle;
  uint16_t hCodeError = MC_NO_FAULTS;
  SpeednPosFdbk_Handle_t *speedHandle;
  speedHandle = STC_GetSpeedSensor(pSTC[M1]);
  hElAngle = SPD_GetElAngle(speedHandle);
  PWMC_GetPhaseCurrents(pwmcHandle[M1], &Iab);
  Ialphabeta = MCM_Clarke(Iab);
  Iqd = MCM_Park(Ialphabeta, hElAngle);
  if (PWMC_GetPWMState(pwmcHandle[M1]) == true)
  {
#ifdef USE_ANN_CONTROLLER
    if (g_ann_full_active != 0U)
    {
      float vq_target = g_vq_cached_full;
      float delta;
      float upperLimit = (float)pPIDIq[M1]->hUpperOutputLimit;
      float lowerLimit = (float)pPIDIq[M1]->hLowerOutputLimit;
      /* Clamp to the same output range the PI controller was bounded by. */
      if (vq_target > upperLimit)
      {
        vq_target = upperLimit;
      }
      if (vq_target < lowerLimit)
      {
        vq_target = lowerLimit;
      }

      /* Slew-rate limit: the ANN only updates at 1 kHz, so without this the
         cached value would step-change once every 16 HF ticks. */
      delta = vq_target - g_vq_output_prev_full;
      if (delta > ANN_VQ_SLEW_MAX)
      {
        delta = ANN_VQ_SLEW_MAX;
      }
      if (delta < -ANN_VQ_SLEW_MAX)
      {
        delta = -ANN_VQ_SLEW_MAX;
      }
      g_vq_output_prev_full += delta;

      Vqd.q = (int16_t)g_vq_output_prev_full;

      /* Bumpless transfer: keep the PI integral term tracking the ANN output
         so a fallback to PI does not produce a torque step. */
      PID_SetIntegralTerm(pPIDIq[M1], ((int32_t)Vqd.q) << 16);
    }
    else
    {
      /* ANN produced NaN/Inf, or has not run yet - use the stock PI. */
      Vqd.q = PI_Controller(pPIDIq[M1], (int32_t)(FOCVars[M1].Iqdref.q) - Iqd.q);
    }
#else
    Vqd.q = PI_Controller(pPIDIq[M1], (int32_t)(FOCVars[M1].Iqdref.q) - Iqd.q);
#endif /* USE_ANN_CONTROLLER */

    /* The d-axis loop stays PI-controlled - only Iq is replaced by the ANN. */
    Vqd.d = PI_Controller(pPIDId[M1], (int32_t)(FOCVars[M1].Iqdref.d) - Iqd.d);
  }
  else
  {
    Vqd.q = 0;
    Vqd.d = 0;
  }
  Vqd = Circle_Limitation(&CircleLimitationM1, Vqd);
  Valphabeta = MCM_Rev_Park(Vqd, hElAngle);

  if (PWMC_GetPWMState(pwmcHandle[M1]) == true)
  {
    hCodeError = PWMC_SetPhaseVoltage(pwmcHandle[M1], Valphabeta);
  }
  else
  {
    /* Nothing to do. No PWM setting to prevent possible ChargeBootCap conflict */

  }

  FOCVars[M1].Vqd = Vqd;
  FOCVars[M1].Iab = Iab;
  FOCVars[M1].Ialphabeta = Ialphabeta;
  FOCVars[M1].Iqd = Iqd;
  FOCVars[M1].Valphabeta = Valphabeta;
  FOCVars[M1].hElAngle = hElAngle;

  FW_DataProcess(pFW[M1], Vqd);
  return (hCodeError);
}

/* USER CODE BEGIN mc_task 0 */

/* USER CODE END mc_task 0 */

/******************* (C) COPYRIGHT 2025 STMicroelectronics *****END OF FILE****/
