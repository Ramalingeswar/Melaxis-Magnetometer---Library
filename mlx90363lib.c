  /**
  ******************************************************************************
  * @file     SPI_Interface with MLX90363(mlx90363lib.c)
  * @author   P.Ramalingeswara rao
  * @version  V2.0
  * @date     16-MAY-2019
  * @brief    This file contains the functions for SPI Interface with MLX90363 
  ******************************************************************************
  */ 
  
/* Includes ------------------------------------------------------------------*/
#include "mlx90363lib.h"					/* mlx90363lib header file */
  
/* Private variables ---------------------------------------------------------*/

MAGNETOMETER_DATA *CS1_DATA,*CS2_DATA;	/* Creating pointer to store Magnetometer DATA from CS1 & CS2 of magnetometer */

uint8_t 	ch_sm,   					/* To Check run time check sum */ 
                op_code; 					/* To Check run time Op code */

uint8_t RxBuffer[8];    				/* SPI Receiving Buffer */ 

#if (ALPHA_VALUES || ALPHA_BETA_VALUES)

const float f32_lsb_to_dec_degrees = 0.02197;	/*To convert LBS values to degree Values*/

#endif

/* Keys to write data into Magnetometer EEPROM for perticulour ADDR, keys from DATASHEET */

uint16_t EE_WRITE_KEY[4][8]={17485,31053,57190,57724,7899,53543,26763,12528,
                            38105,51302,16209,24847,13134,52339,14530,18350,
                            55636,64477,40905,45498,24411,36677,4213,48843,
                            6368,5907,31384,63325,3562,19816,6995,3147}; 		

/* One byte of CRC for 7 bytes of SPI DATA */
 
char CRCArray[] = {
      0x00, 0x2F, 0x5E, 0x71, 0xBC, 0x93, 0xE2, 0xCD, 0x57, 0x78, 0x09, 0x26,
      0xEB, 0xC4, 0xB5, 0x9A, 0xAE, 0x81, 0xF0, 0xDF, 0x12, 0x3D, 0x4C, 0x63,
      0xF9, 0xD6, 0xA7, 0x88, 0x45, 0x6A, 0x1B, 0x34, 0x73, 0x5C, 0x2D, 0x02,
      0xCF, 0xE0, 0x91, 0xBE, 0x24, 0x0B, 0x7A, 0x55, 0x98, 0xB7, 0xC6, 0xE9,
      0xDD, 0xF2, 0x83, 0xAC, 0x61, 0x4E, 0x3F, 0x10, 0x8A, 0xA5, 0xD4, 0xFB,
      0x36, 0x19, 0x68, 0x47, 0xE6, 0xC9, 0xB8, 0x97, 0x5A, 0x75, 0x04, 0x2B,
      0xB1, 0x9E, 0xEF, 0xC0, 0x0D, 0x22, 0x53, 0x7C, 0x48, 0x67, 0x16, 0x39,
      0xF4, 0xDB, 0xAA, 0x85, 0x1F, 0x30, 0x41, 0x6E, 0xA3, 0x8C, 0xFD, 0xD2,
      0x95, 0xBA, 0xCB, 0xE4, 0x29, 0x06, 0x77, 0x58, 0xC2, 0xED, 0x9C, 0xB3,
      0x7E, 0x51, 0x20, 0x0F, 0x3B, 0x14, 0x65, 0x4A, 0x87, 0xA8, 0xD9, 0xF6,
      0x6C, 0x43, 0x32, 0x1D, 0xD0, 0xFF, 0x8E, 0xA1, 0xE3, 0xCC, 0xBD, 0x92,
      0x5F, 0x70, 0x01, 0x2E, 0xB4, 0x9B, 0xEA, 0xC5, 0x08, 0x27, 0x56, 0x79,
      0x4D, 0x62, 0x13, 0x3C, 0xF1, 0xDE, 0xAF, 0x80, 0x1A, 0x35, 0x44, 0x6B,
      0xA6, 0x89, 0xF8, 0xD7, 0x90, 0xBF, 0xCE, 0xE1, 0x2C, 0x03, 0x72, 0x5D,
      0xC7, 0xE8, 0x99, 0xB6, 0x7B, 0x54, 0x25, 0x0A, 0x3E, 0x11, 0x60, 0x4F,
      0x82, 0xAD, 0xDC, 0xF3, 0x69, 0x46, 0x37, 0x18, 0xD5, 0xFA, 0x8B, 0xA4,
      0x05, 0x2A, 0x5B, 0x74, 0xB9, 0x96, 0xE7, 0xC8, 0x52, 0x7D, 0x0C, 0x23,
      0xEE, 0xC1, 0xB0, 0x9F, 0xAB, 0x84, 0xF5, 0xDA, 0x17, 0x38, 0x49, 0x66,
      0xFC, 0xD3, 0xA2, 0x8D, 0x40, 0x6F, 0x1E, 0x31, 0x76, 0x59, 0x28, 0x07,
      0xCA, 0xE5, 0x94, 0xBB, 0x21, 0x0E, 0x7F, 0x50, 0x9D, 0xB2, 0xC3, 0xEC,
      0xD8, 0xF7, 0x86, 0xA9, 0x64, 0x4B, 0x3A, 0x15, 0x8F, 0xA0, 0xD1, 0xFE,
      0x33, 0x1C, 0x6D, 0x42 };

/**
  * @addtogroup SPI_FullDuplex Communication with 64-bit frame
  * @{
  */
/*function Definations ----------------------------------------------------------*/

/**
    * @brief  Delay.
    * @param  nCount
    * @retval None
    */
void Delay(uint16_t ms)
{
  uint16_t nCount;		
  
  for(int i=0;i<ms;i++)
  {
    nCount=0xC30;			/* Caluculated to 16MHz clock speed ST controller for 1 ms*/
     while (nCount != 0)
      {
          nCount--;
      }
  }
}

/**
 * @brief  Get2 SS Rising Edge to Sync Pulse (RE).
 * @param  None
 * @retval None
 */
void tRESync_Delay(void)
{
  uint16_t nCount=0x109; /*for 84.8 micro seconds*/
  
    while (nCount)
    {
            nCount--;
    }
}

/**
 * @brief  Sync Pulse (RE) to Get2 Falling Edge
 * @param  None
 * @retval None
 */
void tSyncFE_Delay(void)
{
  uint16_t nCount=0xC90; /*for 1007.7 micro seconds delay*/
  
    while (nCount)
    {
            nCount--;
    }
}

#ifdef USE_FULL_ASSERT

  /**
    * @brief  Reports the name of the source file and the source line number
    *   where the assert_param error has occurred.
    * @param file: pointer to the source file name
    * @param line: assert_param error line source number
    * @retval None
    */
  void assert_failed(uint8_t* file, uint32_t line)
  { 
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

    /* Infinite loop */
    while (1)
    {
    }
  }
#endif

/**
  * @brief  It will print type of diagnosis error in serial terminal
  * @param  Diagnosis Buffer
  * @retval None
  */
void Diag_magnetometer(uint8_t *D_BUFF)
{
  for(int j=0;j<=2;j++)
  {
    for(int i=0;(i<=7);i++)
    {
      if((D_BUFF[j]>>i)&1)
      {
        switch(i+(j*8))
        {
          case 0: print_str("\n\rD0: RAM March C-10N Test\n\r");break;
          case 1: print_str("\n\rD1: Watchdog BIST\n\r");break;
          case 2: print_str("\n\rD2: ROM 16 bit Checksum\n\r");break;
          case 3: print_str("\n\rD3: RAM Test (continuous)\n\r");break;
          case 4: print_str("\n\rD4: CPU Register Functional Test\n\r");break;
          case 5: print_str("\n\rD5: EEPROM Calibration parameters (8 bit CRC)\n\r");break;    
          case 6: print_str("\n\rD6: EEPROM Hamming Code DED (Dual Error Detection)\n\r");break;    
          case 7: print_str("\n\rD7: EEPROM RAM Cache Error\n\r");break;
          case 8: print_str("\n\rD8: ADC Block\n\r");break;
          case 16: print_str("\n\rD16: Temperature > 190 Deg.C (± 20 Deg.C)\n\rTemperature < -80 Deg.C (± 20 Deg.C)\t External failure\n\r");break;
          case 17: print_str("\n\rD17: Field magnitude too high ( Norm > 99% ADC Span )\t External Failure\n\r");break;
          case 18: print_str("\n\rD18: Field magnitude too low ( Norm < 20% ADC Span )\t External failure\n\r");break;
          case 19: print_str("\n\rD19: ADC clipping (X, Y, Z, two phases each)\t External failure\n\r");break;
          case 20: print_str("\n\rD20:Supply voltage monitor (VDD) and Regulator monitor (VDEC)\t External failure\n\r");break;
          default:print_str("\n\rDefault Diagnosis error\n\r");break;
        }
      }
    }
  }
  
  print_str("\nRoot-cause of entry in fail-safe mode (FSMERC): ");
  
  switch(D_BUFF[3]>>6)
  {
    case 0:print_str("The chip is not in fail-safe mode\n\r");break;
    case 1:print_str("BIST error happened and the chip is in fail-safe mode\n\r");break;
    case 2:print_str("Digital diagnostic error happened and the chip is in fail-safe mode\n\r");break;
    case 3:
      print_str("\nAnalog-class diagnostics (ANADIAGCNT): ");
                       switch(D_BUFF[3])
                       {
                          case 193:print_str("Protection error interruption happened\n\r");break;
                          case 194:print_str("Invalid address error interruption happened\n\r");break;
                          case 195:print_str("Program error interruption happened\n\r");break;
                          case 196:print_str("Exchange error interruption happened\n\r");break;
                          case 197:print_str("Not connected error interruption happened\n\r");break;
                          case 198:print_str("Stack Interrupt\n\r");break;    
                          case 199:print_str("Flow Control Error\n\r");break;
                       default:print_str("Default Diagnosis error\n\r");break;
                       }
                                         break;
  }
}

/**
    * @brief  ERROR defination.
    * @param  Error Data byte
    * @retval None
    */
void Call_error_function(uint8_t ERR_DATA)
{
	print_str("\tError type :");
	switch(ERR_DATA)
	{
		case 1: print_str("INCORRECT BIT COUNT\n\r");break;
		case 2: print_str("INCORRECT CRC\n\r");break;
		case 3: print_str("ANSWER TIME OUT OR ANSWER NOT READY\n\r");break;
		case 4: print_str("OPCODE NOT VALID FROM MOSI\n\r");break;
	}
}

/**
    * @brief  Status of E1 E0 Diagnosis
    * @param  Combined DATA of E1 & E0 bits
    * @retval None
    */
void Call_E1_E0_diag(uint8_t E1_E0_DATA)
{
	print_str("\t * =>Diagnosis type :");
	switch(E1_E0_DATA)
	{
		case 0: print_str("First Diagnostics Seuence Not Yet Finished");break;
		case 1: print_str("Diagnostic Fail");break;
		case 2: print_str("Diagnostic Pass (Previous cycle)");break;
		case 3: print_str("Diagnostic Pass – New Cycle Completed");break;
	}
}

/**
    * @brief  To Check errors on received buffer & Expected OP_CODE .
    * @param  Received Buffer, OP_CODE value
    * @retval MLX_SUCCESS (or) MLX_ERROR
    */
	
uint8_t Error_Check(uint8_t* RxBuff,uint8_t OP_C)
{
	uint8_t ch_sm,   					/* To Check run time check sum */ 
			op_code; 					/* To Check run time Op code */

	ch_sm=ComputeCRC(RxBuffer);		/*Computing CRC*/
	
        op_code=RxBuffer[6]&(0x3F);		/* Extracting OP_CODE*/

	if(ch_sm!=RxBuffer[7])
	{
          print_str("\n\rCheck sum failed:\n\r");
#if DEBUG
			print_str("\n\r Error occured running while loop\n\r");
			while(1);
#endif		
          return MLX_ERROR;
	}
        
        if((RxBuffer[6]==ERROR_M_OPCODE)&&(RxBuffer[1]==0)&&(RxBuffer[2]==0)&&(RxBuffer[3]==0)&&(RxBuffer[4]==0)&&(RxBuffer[5]==0))
	{
          print_str("\n\rRESULT ERROR:\n\r");
          
          Call_error_function(RxBuffer[0]);		/* To  find which type of error */
          
          print_SPI_data();		                /* To print RECEIVED DATA */
#if DEBUG
			print_str("\n\r Error occured running while loop\n\r");
			while(1);
#endif
          return MLX_ERROR;
	}
        
        if((RxBuffer[6]==NTT_M_OPCODE)&&(RxBuffer[0]==0x11)&&(RxBuffer[1]==0x22)&&(RxBuffer[2]==0x33)&&(RxBuffer[3]==0x44)&&(RxBuffer[4]==0x55)&&(RxBuffer[5]==0x66))
	{
          print_str("\n\rNOTHIG TO TRANSMIT (NTT):\n\r");
          
          print_SPI_data();		                /* To print RECEIVED DATA */
#if DEBUG
			print_str("\n\r Error occured running while loop\n\r");
			while(1);
#endif
          return MLX_ERROR;
	}

	if(OP_C!=NULL_DATA)                     /* Checking for Expected OP-Code*/
	{
		if(op_code!=OP_C)
		{
                  print_str("\n\rNot Expected opcode from MISO:\n\r");
                  print_SPI_data();		/* To print RECEIVED DATA */
#if DEBUG
			print_str("\n\r Error occured running while loop\n\r");
			while(1);
#endif			
                  return MLX_ERROR;
		}
	}
	return MLX_SUCCESS;
}

/**
 * @brief  To pull down the corresponding CS pin .
 * @param Chip select Number
 * @retval None
 */
void CS_pull_down(uint8_t CS)
{
  if(CS==CS1)
    GPIO_WriteLow(GPIOC, (GPIO_Pin_TypeDef)GPIO_PIN_1);	/* Pulling CS1 LOW*/
  else
    GPIO_WriteLow(GPIOC, (GPIO_Pin_TypeDef)GPIO_PIN_2);	/* Pulling CS2 LOW*/
}

/**
 * @brief  To pull up the corresponding CS pins .
 * @param Chip select Number
 * @retval None
 */
void CS_pull_up(uint8_t CS)
{
    GPIO_WriteHigh(GPIOC, (GPIO_Pin_TypeDef)GPIO_PIN_1);	/* Pulling CS1 HIGH*/
    GPIO_WriteHigh(GPIOC, (GPIO_Pin_TypeDef)GPIO_PIN_2);	/* Pulling CS2 HIGH*/
}

/**
 * @brief  To send Data to slave .
 * @param  array buffer, Chip select number
 * @retval None
 */
void Send_SPI_data(uint8_t TX_BUFFER[],uint8_t X)
{
    uint8_t TxCounter=0,RxCounter=0;
    
    CS_pull_down(X);                            /* Pulling-Down	Corresponding Chipselect pin */ 
    
    for(int i=0;i<8;i++)
    {
            /* Wait until end of transmit */

            while (SPI_GetFlagStatus(SPI_FLAG_TXE)== RESET);

            /* Write one byte in the SPI Transmit Data Register */

            SPI_SendData(TX_BUFFER[TxCounter++]);

            /* Wait the byte is entirely received by SPI */

            while (SPI_GetFlagStatus(SPI_FLAG_RXNE) == RESET);

            /* Store the received byte in the RxBuffer2 */

            RxBuffer[RxCounter++] = SPI_ReceiveData();
    }
    
    CS_pull_up(X);                              /* Pulling-Up	Corresponding Chipselect pin */
}

 /* @brief  Function ComputeCRC .
 * @param  array buffer
 * @retval 1 byte corresponding to CRC code
 */
char ComputeCRC(uint8_t Data[])
  {
    char CRC = 0xFF;
    CRC = CRCArray[(CRC ^ Data[0])];
    CRC = CRCArray[(CRC ^ Data[1])];
    CRC = CRCArray[(CRC ^ Data[2])];
    CRC = CRCArray[(CRC ^ Data[3])];
    CRC = CRCArray[(CRC ^ Data[4])];
    CRC = CRCArray[(CRC ^ Data[5])];
    CRC = CRCArray[(CRC ^ Data[6])];
    CRC = ~CRC;
    
	return CRC;
  }

/**
  * @brief  UART,SPI & GPIO Initializations.
  * @param  None
  * @retval None
  */
void Peripheral_init(void)
  {
    /*High speed internal clock prescaler: 1*/
    CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1);

    /* UART1 configuration -------------------------------------------------------*/
    /* UART1 configured as follow:
    - Word Length = 8 Bits
    - 1 Stop Bit
    - No parity
    - BaudRate = 115200 baud
    - UART1 Clock enabled
    - Polarity Low
    - Phase Middle
    - Last Bit enabled
    - Receive and transmit enabled
    */
    UART1_DeInit();

    UART1_Init((uint32_t)115200, UART1_WORDLENGTH_8D, UART1_STOPBITS_1, UART1_PARITY_NO, 
    (UART1_SyncMode_TypeDef)(UART1_SYNCMODE_CLOCK_ENABLE | UART1_SYNCMODE_CPOL_LOW |UART1_SYNCMODE_CPHA_MIDDLE |UART1_SYNCMODE_LASTBIT_ENABLE),
    UART1_MODE_TXRX_ENABLE);
    UART1_Cmd(DISABLE);

    /* SPI configuration */
    SPI_DeInit();

    /* Initialize SPI in Master mode  */
    SPI_Init(SPI_FIRSTBIT_MSB, SPI_BAUDRATEPRESCALER_8, SPI_MODE_MASTER, SPI_CLOCKPOLARITY_LOW,
    SPI_CLOCKPHASE_2EDGE, SPI_DATADIRECTION_2LINES_FULLDUPLEX, SPI_NSS_SOFT,(uint8_t)0xC2);

    /* Enable the UART1*/
    UART1_Cmd(ENABLE);

    Delay(30);

    /* Enable the SPI*/
    SPI_Cmd(ENABLE);

    /* Initialize I/Os in Output Mode to select CS pin */
    GPIO_Init(GPIOC, (GPIO_Pin_TypeDef)GPIO_PIN_2, GPIO_MODE_OUT_OD_LOW_FAST/*GPIO_MODE_OUT_PP_LOW_FAST*/);
    GPIO_Init(GPIOC, (GPIO_Pin_TypeDef)GPIO_PIN_1, GPIO_MODE_OUT_OD_LOW_FAST/*GPIO_MODE_OUT_PP_LOW_FAST*/);
    
	/* Pulling up Both chipselect pins to high Initally */
    GPIO_WriteHigh((GPIO_TypeDef*)GPIOC, (GPIO_Pin_TypeDef)GPIO_PIN_1);
    GPIO_WriteHigh((GPIO_TypeDef*)GPIOC, (GPIO_Pin_TypeDef)GPIO_PIN_2);
    
	/* Allocating memory to two DIES for Alpha (or) Alpha-Beta (or) XYZ values DATA */

    CS1_DATA=(MAGNETOMETER_DATA *)calloc(1,sizeof(MAGNETOMETER_DATA));
    CS2_DATA=(MAGNETOMETER_DATA *)calloc(1,sizeof(MAGNETOMETER_DATA));
  }

/**
  * @brief  To read perticulour addres from Magnetometer.
  * @param  uint16_t Address1,Address2 & Chipselect number
  * @retval 4 bytes of data as array at ADDR1 & ADDR2 simultaneously
  */
uint8_t* MAG_EEREAD(uint16_t ADDR1,uint16_t ADDR2,uint8_t CS)
{
  /* Reading Data at required Locations we are using 2 Address for 64 bit data (valid address ADDR1 & ADDR2)*/
  
  uint8_t EE_READ[8],CH_ERROR;
  
  EE_READ[0]=ADDR1;						/* LSByte of ADDR1 */   
  EE_READ[1]=(ADDR1>>8);				/* MSByte of ADDR1 */      
  EE_READ[2]=ADDR2;						/* LSByte of ADDR2 */   
  EE_READ[3]=(ADDR2>>8);				/* MSByte of ADDR2 */
  EE_READ[4]=EE_READ[5]=NULL_DATA;		/* NULL DATA For remainig bytes */
  EE_READ[6]=EEREAD_M_OPCODE;			/* MARKER + OPCODE Byte */
  EE_READ[7]=ComputeCRC(EE_READ);		/* Computing CRC */

#if DEBUG
	print_str("\n\rEEREAD from Chip select ");print_int(CS);print_str(":\n\r");
#endif
	
  for(uint8_t i=0;i<=1;i++)
   {
      Send_SPI_data(EE_READ,CS);		/* Sending EE_READ Command to the Magnetometer */ 
      Delay(10);
   }
   
   CH_ERROR=Error_Check(RxBuffer,EEREAD_RES_OPCODE);		/* Checking for ERROR if any & expected OPCODE */
   
#if DEBUG
	if(CH_ERROR==MLX_SUCCESS)
		print_str("\n\rEEREAD OK FOR CHIP_SELECT ");print_int(CS);print_str(".\n\r");
#endif
   
   CH_ERROR=SEND_NOP_CMD(CS);			/* Sending NOP command to RESET Magnetometer*/
   
#if DEBUG
	if(CH_ERROR==MLX_SUCCESS)
		print_str("\rEEREAD NOP OK FOR CHIP_SELECT ");print_int(CS);print_str(".\n\r");
#endif

   return RxBuffer;
}

/**
  * @brief  To Write perticular address Magnetometer EEPROM.
  * @param  uint16_t Address, uint16_t DATA, uint8_t chip select number
  * @retval Write status of EEPROM
  */
uint8_t MAG_EEWRITE(uint16_t ADDR,uint16_t DATA,uint8_t CS)
{
  /* Writing Data to required Locations(valid ADDR) */
  
  uint8_t EE_WRITE[8],CHALLENGE[8],CHALLENGE_ANS[8],CH_ERROR;
  
  EE_WRITE[0]=NULL_DATA;						/* NULL DATA */
  EE_WRITE[1]=(ADDR&0x003F);      					/* Extracting 6 bits from 16 bit ADDR */
  EE_WRITE[2]=EE_WRITE_KEY[(ADDR>>4)&(0x03)][(ADDR>>1)&(0x07)];   	/* LSByte of EEWRITE Key */
  EE_WRITE[3]=EE_WRITE_KEY[(ADDR>>4)&(0x03)][(ADDR>>1)&(0x07)]>>8;	/* MSByte of EEWRITE Key */
  EE_WRITE[4]=DATA;							/* LSByte of 16 Bit DATA */
  EE_WRITE[5]=DATA>>8;							/* MSByte of 16 Bit DATA */
  EE_WRITE[6]=EEWRITE_M_OPCODE;						/* EEWRITE Marker + OPCODE */	
  EE_WRITE[7]=ComputeCRC(EE_WRITE);					/* Computing CRC*/

#if DEBUG
		print_str("\n\rEEWRITE to Magnetometer Chip Select ");print_int(CS);print_str(":\n\r");
#endif

  Send_SPI_data(EE_WRITE,CS);						/* Sending EE_READ Command to the Magnetometer*/ 
  Delay(10);
  
  CH_ERROR=Error_Check(RxBuffer,EEWRITE_RES_OPCODE);	                /* Checking for ERROR if any & expected OPCODE */
  
#if DEBUG
	if(CH_ERROR==MLX_SUCCESS)
		print_str("\n\rChallenge from Magnetometer Chip Select ");print_int(CS);print_str(" Received.\n\r");
#endif  
  
  CHALLENGE[0]=CHALLENGE[1]=CHALLENGE[2]=CHALLENGE[3]=CHALLENGE[4]=CHALLENGE[5]=NULL_DATA;	/* NULL DATA */ 
  CHALLENGE[6]=EE_RD_CHL_M_OPCODE;															/* READ Challenge MARKER +OPCODE */
  CHALLENGE[7]=ComputeCRC(CHALLENGE);														/* Computing CRC*/
    
  Send_SPI_data(CHALLENGE,CS);							/* Sending EE_READ_CHALLENGE Command to the Magnetometer*/ 
  Delay(10);
  
  CH_ERROR=Error_Check(RxBuffer,EEREAD_CH_RES_OPCODE);	/* Checking for ERROR if any & expected OPCODE */
  
#if DEBUG
	if(CH_ERROR==MLX_SUCCESS)
		print_str("\n\rWrite Challenge received from Magnetometer Chip Select ");print_int(CS);print_str(".\n\r");
#endif
   
  CHALLENGE_ANS[0]=CHALLENGE_ANS[1]=NULL_DATA;	/* NULL DATA */
  CHALLENGE_ANS[2]=RxBuffer[2]^0x34;			/* EXOR LSByte with 0x34 to answer ( from DATASHEET ) */ 
  CHALLENGE_ANS[3]=RxBuffer[3]^0x12;			/* EXOR MSByte with 0x12 to answer ( from DATASHEET ) */
  CHALLENGE_ANS[4]=(~(RxBuffer[2]^0x34));		/* Complementing LSByte ( from DATASHEET ) */
  CHALLENGE_ANS[5]=(~(RxBuffer[3]^0x12));		/* Complementing MSByte ( from DATASHEET ) */
  CHALLENGE_ANS[6]=RD_CHL_ANS_M_OPCODE;			/* READ Challenge ANS MARKER +OPCODE */
  CHALLENGE_ANS[7]=ComputeCRC(CHALLENGE_ANS);	/* Computing CRC */
  
  Send_SPI_data(CHALLENGE_ANS,CS);				/* Sending Challenge Answer to Magnetometer */
  Delay(40);
   
  CH_ERROR=Error_Check(RxBuffer,EEREAD_CH_RES_ANS_OPCODE);	/* Checking for ERROR if any & expected OPCODE */

#if DEBUG
	if(CH_ERROR==MLX_SUCCESS)
		print_str("\n\rChallenge Answer OK for Chip Select ");print_int(CS);print_str(".\n\r");
#endif
  
  CH_ERROR=SEND_NOP_CMD(CS);	/*Sending NOP command for checking EEwrite Status */
  Delay(10);
  
#if DEBUG
	if(CH_ERROR==MLX_SUCCESS)
		print_str("\n\rEEWRITE Status OK for Chip Select ");print_int(CS);print_str(".\n\r");
#endif

  return RxBuffer[0];			/* Returns Write Status of EEPROM */
}

/**
  * @brief  To RUN GET2 command to read values from magnetometer.
  * @param  none
  * @retval MLX_SUCCESS or ERROR
  */
uint8_t RUN_GET2_CMD(void)
{
  /* Sending GET2 Command to read Alhpa (or) Alpha-Beta (or) X-Y-Z values from Magnetometer */
  
  uint8_t GET2_CMD[8],CH_ERROR;
  
  GET2_CMD[0]=GET2_CMD[1]=GET2_CMD[4]=GET2_CMD[5]=NULL_DATA;	/* NULL DATA */
  GET2_CMD[2]=GET2_CMD[3]=GET_TIME_OUT;							/* Setting Time out value (from DATASHEET) */
  GET2_CMD[6]=GET2_M_OPCODE;									/* GET2 Command MARKER + OPCODE */
  GET2_CMD[7]=ComputeCRC(GET2_CMD);								/* Computing CRC*/

#if DEBUG
		print_str("\n\rRunning GET2 command ");print_str(":\n\r");
#endif

  Send_SPI_data(GET2_CMD,CS1);					/* Sending GET2 Command to the Magnetometer CS1*/ 
 
  CH_ERROR=Error_Check(RxBuffer,NULL_DATA);		/* Checking for ERROR if any & expected OPCODE */

#if DEBUG
	if(CH_ERROR==MLX_SUCCESS)
		print_str("\n\r GET2 CMD OK for CS ");print_int(CS1);print_str(".\n\r");
#endif  
  
  Process_Data(RxBuffer,CS1_DATA);				/*Processing Data from CS1 received buffer*/
  
  Send_SPI_data(GET2_CMD,CS2);					/* Sending GET2 Command to the Magnetometer CS2 */ 
 
  CH_ERROR=Error_Check(RxBuffer,NULL_DATA);		/* Checking for ERROR if any & expected OPCODE */

#if DEBUG
	if(CH_ERROR==MLX_SUCCESS)
		print_str("\n\r GET2 CMD OK for CS ");print_int(CS2);print_str(".\n\r");
#endif
  
  Process_Data(RxBuffer,CS2_DATA);				/*Processing Data from CS2 received buffer*/
  
  tRESync_Delay();								/*NEED of delay to send sync pulse (From DATASHEET)*/

  Sync_Pulse();									/*Sync pulse for both slaves (From DATASHEET)*/

  tSyncFE_Delay();								/*NEED of delay to process Alpha Beta values (From DATASHEET)*/
  
  return CH_ERROR;
}

/**
  * @brief  To RUN GET2 command with NOP to read values from magnetometer.
  * @param  none
  * @retval MLX_SUCCESS or ERROR
  */
uint8_t RUN_GET2_W_NOP(void)
{
  /* Sending GET2 Command with NOP to read Alhpa (or) Alpha-Beta (or) X-Y-Z values from Magnetometer */
  
  uint8_t GET2_CMD[8],CH_ERROR;
  
  GET2_CMD[0]=GET2_CMD[1]=GET2_CMD[4]=GET2_CMD[5]=NULL_DATA;	/* NULL DATA */
  GET2_CMD[2]=GET2_CMD[3]=GET_TIME_OUT;							/* Setting Time out value (from DATASHEET) */
  GET2_CMD[6]=GET2_M_OPCODE;									/* GET2 Command MARKER + OPCODE */
  GET2_CMD[7]=ComputeCRC(GET2_CMD);								/* Computing CRC */

#if DEBUG
		print_str("\n\rRunning GET2 command with NOP");print_str(":\n\r");
#endif

  Send_SPI_data(GET2_CMD,CS1);					/* Sending GET2 Command to the Magnetometer CS1 */ 
 
  CH_ERROR=Error_Check(RxBuffer,NULL_DATA);		/* Checking for ERROR if any & expected OPCODE */
  
#if DEBUG
	if(CH_ERROR==MLX_SUCCESS)
		print_str("\n\r GET2 CMD OK for CS ");print_int(CS1);print_str(".\n\r");
#endif
  
  Send_SPI_data(GET2_CMD,CS2);					/* Sending GET2 Command to the Magnetometer CS2 */ 
 
  CH_ERROR=Error_Check(RxBuffer,NULL_DATA);		/* Checking for ERROR if any & expected OPCODE */

#if DEBUG
	if(CH_ERROR==MLX_SUCCESS)
		print_str("\n\r GET2 CMD OK for CS ");print_int(CS2);print_str(".\n\r");
#endif
  
  tRESync_Delay();								/* NEED of delay to send sync pulse (From DATASHEET) */

  Sync_Pulse();									/* Sync pulse for both slaves (From DATASHEET) */

  tSyncFE_Delay();								/* NEED of delay to process Alpha Beta values (From DATASHEET) */
  
  CH_ERROR=SEND_NOP_CMD(CS1);					/* Sending NOP to read values from CS1 */

#if DEBUG
	if(CH_ERROR==MLX_SUCCESS)
		print_str("\n\r GET2 NOP OK for CS");print_int(CS1);print_str(".\n\r");
#endif

  Process_Data(RxBuffer,CS1_DATA);				/* Processing Data from CS1 received buffer */
  
  CH_ERROR=SEND_NOP_CMD(CS2);					/* Sending NOP to read values from CS2 */

#if DEBUG
	if(CH_ERROR==MLX_SUCCESS)
		print_str("\n\r GET2 NOP OK for CS");print_int(CS2);print_str(".\n\r");
#endif

  Process_Data(RxBuffer,CS2_DATA);				/* Processing Data from CS2 received buffer */
  
  return CH_ERROR;
}

/**
  * @brief  To RUN GET1 with NOP command to read values from magnetometer.
  * @param  none
  * @retval MLX_SUCCESS or ERROR
  */
uint8_t RUN_GET1_W_NOP(void)
{
  /* Sending GET1 Command with NOP to read Alhpa (or) Alpha-Beta (or) X-Y-Z values from Magnetometer */
  
  uint8_t GET1_CMD[8],CH_ERROR;
  
  GET1_CMD[0]=GET1_CMD[1]=GET1_CMD[4]=GET1_CMD[5]=NULL_DATA;	/* NULL DATA */
  GET1_CMD[2]=GET1_CMD[3]=GET_TIME_OUT;							/* Setting Time out value (from DATASHEET) */
  GET1_CMD[6]=GET1_M_OPCODE;									/* GET1 Command MARKER + OPCODE */
  GET1_CMD[7]=ComputeCRC(GET1_CMD);								/* Computing CRC */

#if DEBUG
		print_str("\n\rRunning GET1 command with NOP");print_str(":\n\r");
#endif

  Send_SPI_data(GET1_CMD,CS1);					/* Sending GET1 Command to the Magnetometer CS1 */ 
 
  CH_ERROR=Error_Check(RxBuffer,NULL_DATA);		/* Checking for ERROR if any & expected OPCODE */

#if DEBUG
	if(CH_ERROR==MLX_SUCCESS)
		print_str("\n\r GET1 CMD OK for CS ");print_int(CS1);print_str(".\n\r");
#endif
  
  Send_SPI_data(GET1_CMD,CS2);					/* Sending GET1 Command to the Magnetometer CS2 */ 
 
  CH_ERROR=Error_Check(RxBuffer,NULL_DATA);		/* Checking for ERROR if any & expected OPCODE */

#if DEBUG
	if(CH_ERROR==MLX_SUCCESS)
		print_str("\n\r GET1 CMD OK for CS ");print_int(CS2);print_str(".\n\r");
#endif

  CH_ERROR=SEND_NOP_CMD(CS1);					/* Sending NOP to read values from CS1 */

#if DEBUG
	if(CH_ERROR==MLX_SUCCESS)
		print_str("\n\r GET1 NOP OK for CS");print_int(CS1);print_str(".\n\r");
#endif

  Process_Data(RxBuffer,CS1_DATA);				/* Processing Data from CS1 received buffer */
  
  CH_ERROR=SEND_NOP_CMD(CS2);					/* Sending NOP to read values from CS2 */
  
#if DEBUG
	if(CH_ERROR==MLX_SUCCESS)
		print_str("\n\r GET1 NOP OK for CS");print_int(CS1);print_str(".\n\r");
#endif

  Process_Data(RxBuffer,CS2_DATA);				/* Processing Data from CS2 received buffer */
  
  return CH_ERROR;
}

/**
  * @brief  To send NOP command to the magneto meter
  * @param  uint8_t Chipselect DATA
  * @retval MLX_SUCCESS or ERROR
  */
uint8_t SEND_NOP_CMD(uint8_t CS)
{
  uint8_t NOP[8],CH_ERROR;
  
  NOP[0]=NOP[1]=NOP[4]=NOP[5]=NULL_DATA;	/* NULL DATA */
  NOP[2]=NOP[3]=NOP_KEY;					/* NOP KEY as 0xAAAA*/
  NOP[6]=NOP_M_OPCODE;						/* NOP Command MARKER + OPCODE */
  NOP[7]=ComputeCRC(NOP);					/* Computing CRC */
  
  Send_SPI_data(NOP,CS);					/* Sending NOP commmad to Magnetometer */
  
  Delay(10);
  
  CH_ERROR=Error_Check(RxBuffer,NULL_DATA);			/* Checking for ERROR if any & expected OPCODE */
  
  return CH_ERROR;
}

/**
  * @brief  To send REBOOT command to the magneto meter
  * @param  uint8_t Chipselect DATA
  * @retval MLX_SUCCESS or ERROR
  */
uint8_t MAG_REBOOT_CMD(uint8_t CS)
{
  uint8_t REBOOT[8],CH_ERROR;
  
  REBOOT[0]=REBOOT[1]=REBOOT[2]=REBOOT[3]=REBOOT[4]=REBOOT[5]=NULL_DATA;	/* NULL DATA */
  REBOOT[6]=REBOOT_M_OPCODE;												/* REBOOT Command MARKER + OPCODE */
  REBOOT[7]=ComputeCRC(REBOOT);												/* Computing CRC */
  
  Send_SPI_data(REBOOT,CS);													/* Sending REBOOT commmad to Magnetometer */
  
  Delay(30);
  
  CH_ERROR=Error_Check(RxBuffer,NULL_DATA);									/* Checking for ERROR if any & expected OPCODE */
  
  return CH_ERROR;
}

/**
  * @brief  To Run Diagnosis command on magnetometer
  * @param  Chip select value
  * @retval MLX_SUCCESS or ERROR
  */
uint8_t DIAG_CMD(void)
{
  uint8_t DIAG[8],CH_ERROR;
  DIAG[0]=DIAG[1]=DIAG[2]=DIAG[3]=DIAG[4]=DIAG[5]=NULL_DATA;	                                        /* NULL DATA */
  DIAG[6]=DIAG_M_OPCODE;										/* DIAG Command MARKER + OPCODE */
  DIAG[7]=ComputeCRC(DIAG);										/* Computing CRC */
    
  for(int i=1;i<=2;i++)
  {
    
#if DEBUG
    print_str("\n\r DIAGNOSISING for CS ");print_int(i);print_str(":\n\r");
#endif
    
    for(int j=0;j<=1;j++)
    {
      Send_SPI_data(DIAG,i);                                            /* Sending DIAGNOSIS commmad to Magnetometer */
      Delay(5);
    }
    
    CH_ERROR=Error_Check(RxBuffer,NULL_DATA);				/* Checking for ERROR if any & expected OPCODE */
    
    Diag_magnetometer(RxBuffer);

#if DEBUG
        if(CH_ERROR==MLX_SUCCESS)
                print_str("\n\r DIAGNOSIS is OK for CS ");print_int(i);print_str(".\n\r");
#endif
  
  }
  return CH_ERROR;
}

/**
  * @brief  To program magnetometer to Linear motion
  * @param  No value
  * @retval MLX_SUCCESS or ERROR
  */
uint8_t PRG_LINEAR_MOTION(void)
{
	uint8_t CH_ERROR,*READ_DATA;
	uint16_t ADDR_102A=0x102A,ADDR_1032=0x1032,DATA_102A=NULL_DATA,DATA_1032=NULL_DATA;

#if 1 //DEBUG
	print_str("\n\r Programmming Linear Motion:\n\r");
#endif
	for(int i=1;i<=2;i++)
	{
		/* Reading Data at required Locations we are using 2 Address for 64 bit data (valid address 102A & 1032)*/
		READ_DATA=MAG_EEREAD(ADDR_102A,ADDR_1032,i);
		
		//Set MAPXYZ to 1 to use X and Z axes, clear 3D bit
		
                DATA_102A = ((READ_DATA[0]&0xF0)|0x01);
                
                DATA_102A =  (READ_DATA[1]<<8)|DATA_102A;
		
		//Set SMISM and SEL_SMISM
		DATA_1032 = 0x5999;
		
		CH_ERROR=MAG_EEWRITE(ADDR_102A,DATA_102A,i);

#if DEBUG
	if(CH_ERROR==MLX_SUCCESS)
		print_str("\n\r ***********EEWRITE of  ADDR_102A for Magnetometer Chip select ");print_int(i);print_str(" is DONE**********\n\r");
#endif
		
		CH_ERROR=MAG_EEWRITE(ADDR_1032,DATA_1032,i);
		
#if DEBUG
	if(CH_ERROR==MLX_SUCCESS)
		print_str("\n\r ***********EEWRITE of  ADDR_1032 for Magnetometer Chip select ");print_int(i);print_str(" is DONE**********\n\r");
#endif
		
		CH_ERROR=MAG_REBOOT_CMD(i);
		
#if DEBUG
	if(CH_ERROR==MLX_SUCCESS)
		print_str("\n\r REBOOT OK \n\n\r **************Magnetometer Chip select ");print_int(i);print_str(" is Programmed********\n\r");
#endif

	}
	
	return CH_ERROR;
}

/**
  * @brief  To program magnetometer to Joystick motion
  * @param  No value
  * @retval MLX_SUCCESS or ERROR
  */
uint8_t PRG_JOYSTICK_MOTION(void)
{
	uint8_t CH_ERROR,*READ_DATA;
	
	uint16_t ADDR_102A=0x102A,ADDR_1032=0x1032,ADDR_1022=0x1022,ADDR_1024=0x1024,\
				DATA_102A,DATA_1024,DATA_1022;

#if 1//DEBUG
	print_str("\n\r Programmming Joystick Motion:\n\r");
#endif
	
	for(int i=1;i<=2;i++)
	{
		/* Reading Data at required Locations we are using 2 Address for 64 bit data (valid address 102A & 1032)*/
		READ_DATA=MAG_EEREAD(ADDR_102A,ADDR_1032,i);
		
		//Set MAPXYZ to 0, Enable 3D mode
                DATA_102A = ((READ_DATA[0]&0xF0) ^ 0x08);
           
                DATA_102A =  (READ_DATA[1]<<8)|DATA_102A;

		//Set Kalpha to 1.4
		DATA_1022 = 0xB333;
		
		//Set Kbeta to 1.4
		DATA_1024 = 0xB333;
	
		CH_ERROR=MAG_EEWRITE(ADDR_102A,DATA_102A,i);

#if DEBUG
	if(CH_ERROR==MLX_SUCCESS)
		print_str("\n\r EEWRITE OK for ADDR_102A");print_int(i);print_str(".\n\r");
#endif
		
		CH_ERROR=MAG_EEWRITE(ADDR_1022,DATA_1022,i);
		
#if DEBUG
	if(CH_ERROR==MLX_SUCCESS)
		print_str("\n\r EEWRITE OK for ADDR_1022");print_int(i);print_str(".\n\r");
#endif
		
		CH_ERROR=MAG_EEWRITE(ADDR_1024,DATA_1024,i);

#if DEBUG
	if(CH_ERROR==MLX_SUCCESS)
		print_str("\n\r EEWRITE OK for ADDR_1024");print_int(i);print_str(".\n\r");
#endif
		
		CH_ERROR=MAG_REBOOT_CMD(i);

#if DEBUG
	if(CH_ERROR==MLX_SUCCESS)
		print_str("\n\r REBOOT is OK for CS ");print_int(i);print_str(".\n\r");
#endif

	}
	
	return CH_ERROR;
}

/**
 * @brief  To process values from Received buffer.
 * @param  uint8_t* received buffer, MAGNETOMETER_DATA* type structure
 * @retval None
 */
void Process_Data(uint8_t* R_Buffer,MAGNETOMETER_DATA* Data)
{

#if (ALPHA_VALUES||ALPHA_BETA_VALUES)

	//Extract and convert the alpha angle to degrees
	//remove error bits and shift to high byte
	Data->u16_alpha_angle_lsb = (R_Buffer[1] & 0x3F) << 8;
	//add LSb of angle
	Data->u16_alpha_angle_lsb = Data->u16_alpha_angle_lsb + R_Buffer[0];
	//convert to decimal degrees
	Data->f32_alpha_angle_degrees = Data->u16_alpha_angle_lsb * f32_lsb_to_dec_degrees;
	//Extract the virtual gain byte
	Data->u8_virtualgain_dec = R_Buffer[4];
	
#endif

#if ALPHA_BETA_VALUES
	
	//Extract and convert the beta angle to degrees
	//remove error bits and shift to high byte
	Data->u16_beta_angle_lsb = (R_Buffer[3] & 0x3F) << 8;
	//add LSb of angle
	Data->u16_beta_angle_lsb = Data->u16_beta_angle_lsb + R_Buffer[2];
	//convert to decimal degrees
	Data->f32_beta_angle_degrees = Data->u16_beta_angle_lsb * f32_lsb_to_dec_degrees;

#endif

#if XYZ_COMP_VALUES
		
	//remove error bits and shift to high byte
	Data->u16_X_component = (R_Buffer[1] & 0x3F) << 8;
	//add LSb of X Component
	Data->u16_X_component = Data->u16_X_component + R_Buffer[0];
	//remove error bits and shift to high byte
	Data->u16_Y_component = (R_Buffer[3] & 0x3F) << 8;
	//add LSb of Y Component
	Data->u16_Y_component = Data->u16_Y_component + R_Buffer[2];
	//remove error bits and shift to high byte
	Data->u16_Z_component = (R_Buffer[5] & 0x3F) << 8;
	//add LSb of Z Component
	Data->u16_Z_component = Data->u16_Z_component + R_Buffer[4];
	
#endif	
	
	//Extract the error bits
	Data->u8_error_lsb = R_Buffer[1] >> 6;
	//Extract the CRC
	Data->u8_crc_dec = R_Buffer[7];
	//Extract the rolling counter
	Data->u8_rollcnt_dec = R_Buffer[6] & 0x3F;
}

/**
 * @brief  To print Magnetometer Data
 * @param  MAGNETOMETER_DATA* type structure
 * @retval None
 */
void print_data(MAGNETOMETER_DATA* Data)
{
	
#if (ALPHA_VALUES||ALPHA_BETA_VALUES)
      //Send results to serial port
      //print_str("\n\rAlpha Angle (LSb)\t:");
      //print_int(Data->u16_alpha_angle_lsb);print_str("\n\n\r");
      
      print_str("Alpha Angle (Dec)\t:");
      print_float(Data->f32_alpha_angle_degrees);print_str("\n\r");
	  
      //print_str("Virtual Gain (Dec)\t:");
      //print_int(Data->u8_virtualgain_dec);print_str("\n\r");
#endif

#if ALPHA_BETA_VALUES
      //print_str("Beta Angle (LSb)\t:");
      //print_int(Data->u16_beta_angle_lsb);print_str("\n\r");
	  
      print_str("Beta Angle (Dec)\t:");
      print_float(Data->f32_beta_angle_degrees);print_str("\n\r");
#endif

#if XYZ_COMP_VALUES
	print_str("X- Component\t\t:");
	print_int(Data->u16_X_component);print_str("\n\r");

	print_str("Y- Component\t\t:");
	print_int(Data->u16_Y_component);print_str("\n\r");

	print_str("Z- Component\t\t:");
	print_int(Data->u16_Z_component);print_str("\n\r");
#endif

        print_str("Error Bits (Dec)\t:");
        print_int(Data->u8_error_lsb);Call_E1_E0_diag(Data->u8_error_lsb);print_str(".\n\r");
        	  
        //print_str("CRC (Dec)\t\t:");
        //print_int(Data->u8_crc_dec);print_str("\n\r");
      
        print_str("Rolling Counter (Dec)\t:");
        print_int(Data->u8_rollcnt_dec);print_str("\n\n\r");
}

/**
  * @}
  */

/********************************END OF FILE******************************************/
