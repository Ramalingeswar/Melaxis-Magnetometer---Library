/**
  ******************************************************************************
  * @file     SPI_Interface with MLX90363(mlx90363lib.h)
  * @author   P.Ramalingeswara rao
  * @version  V2.0
  * @date     16-MAY-2019
  * @brief    This file contains the headers of mlx90636lib.c 
  ******************************************************************************
  */ 
#ifndef _MLX90363LIB_
#define _MLX90363LIB_

/* Includes ------------------------------------------------------------------*/
#include "stm8s.h"					/* To include code STM8 controller Configuration */
#include <string.h>					/* Standard library */
#include <stdlib.h>    				        /* Standard library */
#include "mlx_debug.h"			                /* To use Serial communication for DEBUG */ 

/* Private define ------------------------------------------------------------*/
#define DEBUG 0 					/* Define when Serial debug Needed */

#define CS1 1						/* Chipselect 1 Indication */
#define CS2 2						/* Chipselect 2 Indication */

#define MLX_SUCCESS			0x01	
#define MLX_ERROR			0x00

/********************************Values definations***********************************/

/* Define it to Read Alpha Value only, remaining defines should be ZERO */
#define ALPHA_VALUES		0

/* Define it to Read Alpha & Beta Values, remaining defines should be ZERO */
#define ALPHA_BETA_VALUES	0

/* Define it to Read XYZ Values, remaining defines should be ZERO */
#define XYZ_COMP_VALUES		1

/********************************Values Definations END********************************/

/**************MARKER + OPCODE Definations from Datasheet******************/
#if ALPHA_VALUES

#define GET1_M_OPCODE	0x13
#define GET2_M_OPCODE	0x14

#endif

#if ALPHA_BETA_VALUES
	
#define GET1_M_OPCODE	0x53
#define GET2_M_OPCODE	0x54

#endif

#if XYZ_COMP_VALUES		

#define GET1_M_OPCODE	0x93
#define GET2_M_OPCODE	0x94	

#endif
	
#define NOP_M_OPCODE		0xD0
#define REBOOT_M_OPCODE		0xEF
#define DIAG_M_OPCODE		0xD6
#define EEREAD_M_OPCODE		0xC1
#define EEWRITE_M_OPCODE	0xC3
#define EE_RD_CHL_M_OPCODE	0xCF
#define RD_CHL_ANS_M_OPCODE	0xC5
#define ERROR_M_OPCODE		0xFD
#define NTT_M_OPCODE		0xFE

#define NULL_DATA			0x00
#define NOP_KEY				0xAA
#define GET_TIME_OUT		0xFF

/* Response opcode's for Every command to Check error */

#define EEREAD_RES_OPCODE 			0x02
#define EEWRITE_RES_OPCODE 			0x11	
#define EEREAD_CH_RES_OPCODE		        0x04	
#define EEREAD_CH_RES_ANS_OPCODE	        0x28
#define EEWRITE_STAT_OPCODE			0x3F
#define REBOOT_RES_OPCODE			0x11
#define DIAG_ANS_M_OPCODE			0xD7

/*to send Sync pulse*/
#define Sync_Pulse()    \
						  GPIO_WriteHigh((GPIO_TypeDef*)GPIOC, (GPIO_Pin_TypeDef)GPIO_PIN_2);        \
                          GPIO_WriteLow((GPIO_TypeDef*)GPIOC, (GPIO_Pin_TypeDef)GPIO_PIN_1);         \
                          for(int i=0;i<1;i++)             \
                          GPIO_WriteHigh((GPIO_TypeDef*)GPIOC, (GPIO_Pin_TypeDef)GPIO_PIN_1);        \
                          GPIO_WriteLow((GPIO_TypeDef*)GPIOC, (GPIO_Pin_TypeDef)GPIO_PIN_2);         \
                          for(int i=0;i<1;i++)             \
                          GPIO_WriteHigh((GPIO_TypeDef*)GPIOC, (GPIO_Pin_TypeDef)GPIO_PIN_2);        \

/* Private typedef -----------------------------------------------------------*/

/******************Structure to store Magnetometer Data**********************/
typedef volatile struct 
{
	#if ALPHA_VALUES
		float f32_alpha_angle_degrees;
		uint16_t u16_alpha_angle_lsb;
		uint8_t u8_virtualgain_dec;
	#endif
		
	#if ALPHA_BETA_VALUES
		float f32_alpha_angle_degrees,f32_beta_angle_degrees;
		uint16_t u16_alpha_angle_lsb,u16_beta_angle_lsb;
		uint8_t u8_virtualgain_dec;
	#endif

	#if XYZ_COMP_VALUES
		uint16_t u16_X_component,u16_Y_component,u16_Z_component;
	#endif

		uint8_t u8_error_lsb,u8_rollcnt_dec,u8_crc_dec;
	
}MAGNETOMETER_DATA;

/*function prototypes -----------------------------------------------*/

/**
    * @brief  To generate Delay in milli seconds.
    * @param  Number of milli seconds
    * @retval None
    */
void Delay(uint16_t ms);

/**
 * @brief  GET2 SS Rising Edge to Sync Pulse (RE).
 * @param  None
 * @retval None
 */
void tRESync_Delay(void);

/**
 * @brief  Sync Pulse (RE) to GET2 Commnad Falling Edge
 * @param  None
 * @retval None
 */
void tSyncFE_Delay(void);

/**
  * @brief  It will print type of diagnosis error in serial terminal
  * @param  Diagnosis Buffer
  * @retval None
  */
void Diag_magnetometer(uint8_t *D_BUFF);

/**
    * @brief  ERROR defination.
    * @param  Error Data byte
    * @retval None
    */
void Call_error_function(uint8_t ERR_DATA);

/**
    * @brief  Status of E1 E0 Diagnosis
    * @param  Combined DATA of E1 & E0 bits
    * @retval None
    */
void Call_E1_E0_diag(uint8_t E1_E0_DATA);

/**
    * @brief  To Check errors on received buffer & Expected OP_CODE .
    * @param  Received Buffer, OP_CODE value
    * @retval MLX_SUCCESS (or) MLX_ERROR
    */
uint8_t Error_Check(uint8_t* RxBuff,uint8_t OP_C);

/**
 * @brief  To pull down the corresponding CS pin .
 * @param Chip select Number
 * @retval None
 */
void CS_pull_down(uint8_t CS);

/**
 * @brief  To pull up the corresponding CS pins .
 * @param Chip select Number
 * @retval None
 */
void CS_pull_up(uint8_t CS);

/**
 * @brief  To send Data to slave .
 * @param  array buffer, Chip select number(1 (or) 2)
 * @retval None
 */
 void Send_SPI_data(uint8_t TX_BUFFER[],uint8_t X);

/* @brief  Function ComputeCRC .
 * @param  array buffer
 * @retval 1 byte corresponding to CRC code
 */
char ComputeCRC(uint8_t Data[]);

/**
  * @brief  UART,SPI & GPIO Initializations.
  * @param  None
  * @retval None
  */
void Peripheral_init(void);

/**
  * @brief  To read perticulour addres from Magnetometer.
  * @param  uint16_t Address1,Address2 & Chipselect number
  * @retval 4 bytes of data as array at ADDR1 & ADDR2 simultaneously
  */
uint8_t* MAG_EEREAD(uint16_t ADDR1,uint16_t ADDR2,uint8_t CS);

/**
  * @brief  To Write perticular address Magnetometer EEPROM.
  * @param  uint16_t Address, uint16_t DATA, uint8_t chip select number
  * @retval Write status of EEPROM
  */
uint8_t MAG_EEWRITE(uint16_t ADDR,uint16_t DATA,uint8_t CS);

/**
  * @brief  To RUN GET2 command to read values from magnetometer.
  * @param  none
  * @retval MLX_SUCCESS or ERROR
  */
uint8_t RUN_GET2_CMD(void);

/**
  * @brief  To RUN GET2 command with NOP to read values from magnetometer.
  * @param  none
  * @retval MLX_SUCCESS or ERROR
  */
uint8_t RUN_GET2_W_NOP(void);

/**
  * @brief  To RUN GET1 with NOP command to read values from magnetometer.
  * @param  none
  * @retval MLX_SUCCESS or ERROR
  */
uint8_t RUN_GET1_W_NOP(void);

/**
  * @brief  To send NOP command to the magneto meter
  * @param  uint8_t Chipselect DATA
  * @retval MLX_SUCCESS or ERROR
  */
uint8_t SEND_NOP_CMD(uint8_t CS);

/**
  * @brief  To send REBOOT command to the magneto meter
  * @param  uint8_t Chipselect DATA
  * @retval MLX_SUCCESS or ERROR
  */
uint8_t MAG_REBOOT_CMD(uint8_t CS);

/**
  * @brief  To Run Diagnosis command on magnetometer
  * @param  void
  * @retval MLX_SUCCESS or ERROR
  */
uint8_t DIAG_CMD(void);

/**
  * @brief  To program magnetometer to Linear motion
  * @param  No value
  * @retval MLX_SUCCESS or ERROR
  */
uint8_t PRG_LINEAR_MOTION(void);

/**
  * @brief  To program magnetometer to Joystick motion
  * @param  No value
  * @retval MLX_SUCCESS or ERROR
  */
uint8_t PRG_JOYSTICK_MOTION(void);

/**
 * @brief  To process values from Received buffer.
 * @param  uint8_t* received buffer, MAGNETOMETER_DATA* type structure
 * @retval None
 */
void Process_Data(uint8_t* R_Buffer,MAGNETOMETER_DATA* Data);

/**
 * @brief  To print Magnetometer Data
 * @param  MAGNETOMETER_DATA* type structure
 * @retval None
 */
void print_data(MAGNETOMETER_DATA* Data);

#endif                      /*           _MLX90363LIB_         */
