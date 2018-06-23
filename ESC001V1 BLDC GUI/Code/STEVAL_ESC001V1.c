/**
  ******************************************************************************
  * @file    STEVAL_ESC001V1.c
  * @author  MikrocontrollerProjekte
  * 				 YouTube:	https://youtu.be/WawNPzLC5oc
  * 				 GitHub:	https://github.com/MikrocontollerProjekte/YouTubeProjects/tree/master/ESC001V1 BLDC GUI
  * @brief   STM32F7 BLDC CAN Speed Controller ESC001V1 GUI
  ******************************************************************************
  * @attention: - add this content inside the File STEVAL_ESC001V1.c in the ESC001V1 Demonstration Project
  *
*/

/* Includes ------------------------------------------------------------------*/
#include "stm32f30x_can.h"
#include "stm32f30x_rcc.h"
#include "UserInterfaceClass.h"

/* Define --------------------------------------------------------------------*/
#define SM_POSITIVE_RUN_INIT 0x04

/* Private variables ---------------------------------------------------------*/
CanTxMsg TxMessageCAN1;
CanRxMsg RxMessageInterruptCAN1;
volatile uint8_t test123;
volatile uint8_t can_arming_cmd_state = 0;
uint16_t motorSpeedCAN = 0;
uint16_t motorSpeedCANlast = 0;

/**
  * @brief  Boot function to initialize the ESC board.
  * @retval none.
  */
void ESCboot(void)
{ 
  /* Enable CAN 1*/
  CAN_InitTypeDef       CAN_InitStructure;
  CAN_FilterInitTypeDef CAN_FilterInitStructure;
  GPIO_InitTypeDef  	GPIO_InitStructure;
  NVIC_InitTypeDef  	NVIC_InitStructure;
  
  /* Enable GPIO clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, (FunctionalState)ENABLE);

  /* Enable CAN clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, (FunctionalState)ENABLE); 

  /* Connect CAN pins to AF */
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_9);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_9);

  /* Configure CAN RX and TX pins */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* NVIC configuration *******************************************************/
  //NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);
  NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;        //= 20,     /*!< USB Device Low Priority or CAN1 RX0 Interrupts                    */  
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelCmd = (FunctionalState)ENABLE;
  NVIC_Init(&NVIC_InitStructure);
 
  /* CAN1 register deinit */
  CAN_DeInit(CAN1);
  CAN_StructInit(&CAN_InitStructure);
  
  /* CAN cell init */
  CAN_InitStructure.CAN_TTCM = (FunctionalState)DISABLE;
  CAN_InitStructure.CAN_ABOM = (FunctionalState)DISABLE;
  CAN_InitStructure.CAN_AWUM = (FunctionalState)DISABLE;
  CAN_InitStructure.CAN_NART = (FunctionalState)DISABLE;
  CAN_InitStructure.CAN_RFLM = (FunctionalState)DISABLE;
  CAN_InitStructure.CAN_TXFP = (FunctionalState)DISABLE;
  CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
  CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
  
  /* CAN Baudrate = 500 SP 72 (CAN clocked at 36 MHz)    */
  CAN_InitStructure.CAN_BS1 = CAN_BS1_12tq;
  CAN_InitStructure.CAN_BS2 = CAN_BS2_5tq;
  CAN_InitStructure.CAN_Prescaler = 4;
  CAN_Init(CAN1, &CAN_InitStructure);
  
  /* CAN filter init "FIFO0" */
  CAN_FilterInitStructure.CAN_FilterNumber = 0;
  CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
  CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
  CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
  CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
  CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;
  CAN_FilterInitStructure.CAN_FilterActivation = (FunctionalState)ENABLE;
  CAN_FilterInit(&CAN_FilterInitStructure);
  
  /* Enable FIFO 0 message pending Interrupt */
  CAN_ITConfig(CAN1, CAN_IT_FMP0, (FunctionalState)ENABLE);
}




/*This is the main function to use in the main.c in order to start the current example */
void pwm_start()
{ 
 if(TB_UserTimebaseHasElapsed())
 { 
  /* User defined code */
  switch (User_State)
  {
   case SM_ARMING:
    {
      if(   (can_arming_cmd_state == 1) 
         && (motorSpeedCAN > speed_min_valueRPM )
         && (motorSpeedCAN < speed_max_valueRPM )
        )
      {
        ARMING_counter++;
        if(ARMING_counter > ARMING_TIME)    
         {
           User_State = SM_ARMED;   
           ARMING_counter  = 0;
           PWM_TURNOFF_counter = 0;                   
           PWM_fail_counter = 0;
           PWM_fail_counter_prev = 0;   
           buffer_completed = FALSE;
         }
      }
      else 
       {
         User_State = SM_ARMING;          
         ARMING_counter  = 0;
       }
      }
    break;  
   case SM_ARMED:
    {
      MCI_ExecSpeedRamp(oMCI, motorSpeedCAN/6, 0);
      cmd_status = MCI_StartMotor(oMCI);

      if(cmd_status==FALSE)    
       {
        User_State = SM_ARMING;                       // Command NOT executed
       }
      else User_State = SM_POSITIVE_RUN_INIT;              // Command executed

      UserCnt = 0;
          
      if(  (motorSpeedCAN == 0)
        ||(can_arming_cmd_state != 1)
       )
      {
        User_State = SM_STOP;
      }
    }
    break;  
   case SM_POSITIVE_RUN_INIT:
   {  
     if(MCI_GetSTMState(oMCI)== RUN)       
     {
       MCI_ExecSpeedRamp(oMCI, motorSpeedCAN/6, 0);
       motorSpeedCANlast = motorSpeedCAN;
       User_State = SM_POSITIVE_RUN;
     }
     
     if(  (motorSpeedCAN == 0)
        ||(can_arming_cmd_state != 1)
       )
     {
       User_State = SM_STOP;
     }
      break;
   }
   case SM_POSITIVE_RUN:
    {     
     if( motorSpeedCAN != motorSpeedCANlast)
     {
       if(MCI_GetSTMState(oMCI)== RUN)       
       {
         MCI_ExecSpeedRamp(oMCI, motorSpeedCAN/6, 0);
         motorSpeedCANlast = motorSpeedCAN;
         User_State = SM_POSITIVE_RUN;
       }
     }
 
     if(  (motorSpeedCAN == 0)
        ||(can_arming_cmd_state != 1)
        )
     {
       User_State = SM_STOP;
     }
    }
    break;
   case SM_STOP:
    {
       MCI_StopMotor(oMCI);

       if (UserCnt >= STOP_DURATION)
          {
            /* Next state */ 
            User_State = SM_ARMING;
            UserCnt = 0;   
            Ton_value  = 0;
            ARMING_counter = 0;
            Ton_value_previous = 0;
            buffer_completed = FALSE;
          }
          else
          {
            UserCnt++;
          }
    }
    break;  
  }
  TB_SetUserTimebaseTime(USER_TIMEBASE_OCCURENCE_TICKS);
 }
}

/**
  * @brief  This function handles PWM control from input signal
  * @param  None
  * @retval None
  */
void PWM_FC_control()
{
#ifdef PWM_ESC  
  if(MCI_GetSTMState(oMCI)== FAULT_NOW)
  {
    if(MCI_GetSTMState(oMCI)== FAULT_NOW)
    {
      uint16_t currentFault = MCI_GetCurrentFaults(oMCI);
      uint16_t occuredFault = MCI_GetOccurredFaults(oMCI);
      
      if(currentFault == MC_FOC_DURATION)
      {
        currentFault = MCI_GetCurrentFaults(oMCI);
      }
      else if(currentFault == MC_OVER_VOLT)
      {
         currentFault = MCI_GetCurrentFaults(oMCI);
      }
      else if(currentFault == MC_UNDER_VOLT)
      {
         currentFault = MCI_GetCurrentFaults(oMCI);
      }
      else if(currentFault == MC_OVER_TEMP)
      {
         currentFault = MCI_GetCurrentFaults(oMCI);
      }
      else if(currentFault == MC_START_UP)
      {
         currentFault = MCI_GetCurrentFaults(oMCI);
      }
      else if(currentFault == MC_SPEED_FDBK)
      {
         currentFault = MCI_GetCurrentFaults(oMCI);
      }
      else if(currentFault == MC_BREAK_IN)
      {
         currentFault = MCI_GetCurrentFaults(oMCI);
      }
      else if(currentFault == MC_SW_ERROR)
      {
         currentFault = MCI_GetCurrentFaults(oMCI);
      }
    }  
  }
#endif
}


/**
  * @brief  This function handles CAN1 RX0 request.
* @param  None
* @retval None
*/                                             
void USB_LP_CAN1_RX0_IRQHandler(void)                                         
{
  uint32_t can_id = 0;
  uint32_t temp = 0;

  // read CAN message
  CAN_Receive(CAN1, CAN_FIFO0, &RxMessageInterruptCAN1);
  
  // read CAN message ID
  can_id = RxMessageInterruptCAN1.ExtId;

  switch(can_id){
  case 0x700:   // ARMING / DISARMING per CAN
    if(RxMessageInterruptCAN1.Data[0] == 0xAA)
    {
      can_arming_cmd_state = 1;
    }
    else
    {
      can_arming_cmd_state = 0;
      Ton_value = 0;
    }

    break;
  case 0x701:   // send Motor Control Information
    temp = (RxMessageInterruptCAN1.Data[0])
          |(RxMessageInterruptCAN1.Data[1] << 8)
          |(RxMessageInterruptCAN1.Data[2] << 16)
          |(RxMessageInterruptCAN1.Data[3] << 24);
    if(can_arming_cmd_state == 1)
    {
      motorSpeedCAN = (uint16_t)temp;
      Ton_value = temp;
    }
    break;
  case 0x702:   // Send all Parameters via CAN
    CAN_Transmit_Message(0x600);
    CAN_Transmit_Message(0x601);
    break;
  default:
    // unknown CAN ID
    break;
  }
}


void CAN_Transmit_Message(uint32_t msg)
{
  int16_t  	temp_s_16 = 0;
  uint16_t	temp_u_16 = 0;
  int32_t 	temp_s_32 = 0;
  uint8_t 	send = 1;
  
  oMCI = GetMCI(M1);
  CMCT oMCT = GetMCT(M1);
  CTSNS oTSNS;
  CMPM oMPM;
  CVBS oVBS;

  TxMessageCAN1.StdId = 0;
  TxMessageCAN1.RTR = CAN_RTR_DATA;
  TxMessageCAN1.IDE = CAN_ID_EXT;
  TxMessageCAN1.DLC = 8;

  switch(msg){
  case 0x200:
    TxMessageCAN1.ExtId = 0x200;
    TxMessageCAN1.Data[7] = ((1 & 0xFF000000) >> 24);
    TxMessageCAN1.Data[6] = ((2 & 0x00FF0000) >> 16);
    TxMessageCAN1.Data[5] = ((3 & 0x0000FF00) >> 8);
    TxMessageCAN1.Data[4] = ((4 & 0x000000FF));
    TxMessageCAN1.Data[3] = ((5 & 0xFF000000) >> 24);
    TxMessageCAN1.Data[2] = ((6 & 0x00FF0000) >> 16);
    TxMessageCAN1.Data[1] = ((7 & 0x0000FF00) >> 8);
    TxMessageCAN1.Data[0] = ((8 & 0x000000FF));
    break;
  case 0x600:
    TxMessageCAN1.ExtId = 0x600;  
    // MC_PROTOCOL_REG_SPEED_MEAS
    // retval int16_t rotor average mechanical speed (01Hz)
    temp_s_16 = (MCI_GetAvrgMecSpeed01Hz(oMCI) * 6);
    TxMessageCAN1.Data[7] = ((temp_s_16 & 0x0000FF00) >> 8);
    TxMessageCAN1.Data[6] = ((temp_s_16 & 0x000000FF));
    
    // MC_PROTOCOL_REG_MOTOR_POWER
    // @retval int16_t The average measured motor power expressed in watt.
    oMPM = MCT_GetMotorPowerMeasurement(oMCT);
    temp_s_16 = MPM_GetAvrgElMotorPowerW(oMPM);
    TxMessageCAN1.Data[5] = ((temp_s_16 & 0x0000FF00) >> 8);
    TxMessageCAN1.Data[4] = ((temp_s_16 & 0x000000FF));
    
    // MC_PROTOCOL_REG_BUS_VOLTAGE
    // @retval uint16_t Latest averaged Vbus measurement in Volts
    oVBS = MCT_GetBusVoltageSensor(oMCT);
    temp_u_16 = VBS_GetAvBusVoltage_V(oVBS);
    TxMessageCAN1.Data[3] = ((temp_u_16 & 0x0000FF00) >> 8);
    TxMessageCAN1.Data[2] = ((temp_u_16 & 0x000000FF));
    
    // MC_PROTOCOL_REG_HEATS_TEMP
    // @retval int16_t Latest averaged temperature measurement in Celsius degrees
    oTSNS = MCT_GetTemperatureSensor(oMCT);
    temp_s_16 = TSNS_GetAvTemp_C(oTSNS);
    TxMessageCAN1.Data[1] = ((temp_s_16 & 0x0000FF00) >> 8);
    TxMessageCAN1.Data[0] = ((temp_s_16 & 0x000000FF));
    break; 
  case 0x601:
    TxMessageCAN1.ExtId = 0x601;  
    // MC_PROTOCOL_REG_TORQUE_MEAS  MC_PROTOCOL_REG_I_Q
    // @retval Curr_Components Stator current Iqd
    temp_s_32 = MCI_GetIqd(oMCI).qI_Component1;
    TxMessageCAN1.Data[7] = 0;
    TxMessageCAN1.Data[6] = 0;
    TxMessageCAN1.Data[5] = 0;
    TxMessageCAN1.Data[4] = 0;
    TxMessageCAN1.Data[3] = ((temp_s_32 & 0xFF000000) >> 24);
    TxMessageCAN1.Data[2] = ((temp_s_32 & 0x00FF0000) >> 16);
    TxMessageCAN1.Data[1] = ((temp_s_32 & 0x0000FF00) >> 8);
    TxMessageCAN1.Data[0] = ((temp_s_32 & 0x000000FF));
  break;  
  default:
    // unknown ID
    send = 0;
    break;
  }

  // send message if ID is known
  if(send != 0)
  {
    CAN_Transmit(CAN1, &TxMessageCAN1);
  }
}

