/**
  ******************************************************************************
  * @file    STEVAL_ESC001V1.h
  * @author  MikrocontrollerProjekte
  * 				 YouTube:	https://youtu.be/WawNPzLC5oc
  * 				 GitHub:	https://github.com/MikrocontollerProjekte/YouTubeProjects/tree/master/ESC001V1 BLDC GUI
  * @brief   STM32F7 BLDC CAN Speed Controller ESC001V1 GUI
  ******************************************************************************
  * @attention: - add this content inside the File STEVAL_ESC001V1.h in the ESC001V1 Demonstration Project from the ST Website
  *
*/

/* Exported define -----------------------------------------------------------*/
#define CAN1_PreemptionPriority	    3
#define CAN1_CLK                    RCC_APB1Periph_CAN1
#define CAN1_RX_PIN                 GPIO_Pin_8
#define CAN1_TX_PIN                 GPIO_Pin_9
#define CAN1_GPIO_PORT              GPIOB
#define CAN1_GPIO_CLK               RCC_AHBPeriph_GPIOB
#define CAN1_AF_PORT                GPIO_AF_9
#define CAN1_RX_SOURCE              GPIO_PinSource8
#define CAN1_TX_SOURCE              GPIO_PinSource9

/* Exported functions ------------------------------------------------------- */
void CAN_Transmit_Message(uint32_t msg);

