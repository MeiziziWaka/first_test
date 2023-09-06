/*
   ±¾´úÂëÓÉÌÔ±¦µêÖÇÓùµç×ÓÌá¹©
	 AS5600Ó²¼þ¹ºÂòÁ´½Ó£ºhttps://item.taobao.com/item.htm?spm=a1z38n.10677092.0.0.2c351debwMQV2w&id=584601996739
*/
/*******************************************************************************
* ÎÄ¼þÃû³Æ£ºas5600.c
*
* Õª    Òª£º1.²ÉÓÃÈí¼þÄ£ÄâI2CÍ¨ÐÅÐ­Òé
*           2.³õÊ¼»¯as5600µÄÏà¹ØÉèÖÃ,
*
* µ±Ç°°æ±¾£º
* ×÷    Õß£ºÖÇÓùµç×Ó	
* ÈÕ    ÆÚ£º2018/12/31
* ±àÒë»·¾³£ºkeil5
*
* ÀúÊ·ÐÅÏ¢£º
*******************************************************************************/
#include "as5600.h"
extern  UART_HandleTypeDef huart1;
	
uint16_t _rawStartAngle=0;
uint16_t _zPosition=0;
uint16_t _rawEndAngle=0;
uint16_t _mPosition=0;
uint16_t _maxAngle=0;	
/*******************************************************************************/
/**
  * @brief  Ä£ÄâIICÑÓÊ±
  * @param
  * @note
  * @retval void
  * @author ÖÇÓùµç×Ó
  */
void Sim_I2C1_Delay(uint32_t delay)
{
	while(--delay);	//dly=100: 8.75us; dly=100: 85.58 us (SYSCLK=72MHz)
}

/**
  * @brief  Ä£ÄâIIC¿ªÊ¼Ê±Ðò
  * @param
  * @note
  * @retval void
  * @author ÖÇÓùµç×Ó
  */
uint8_t Sim_I2C1_START(void)
{
	SDA1_OUT();
	Sim_I2C1_SDA_HIG;
	Sim_I2C1_SCL_HIG;
	Sim_I2C1_NOP;

// if(!Sim_I2C1_SDA_STATE) return Sim_I2C1_BUS_BUSY;

	Sim_I2C1_SDA_LOW;
	Sim_I2C1_NOP;

	Sim_I2C1_SCL_LOW;
	Sim_I2C1_NOP;

	//if(Sim_I2C1_SDA_STATE) return Sim_I2C1_BUS_ERROR;

	return Sim_I2C1_READY;
}

/**
  * @brief  Ä£ÄâIICÍ£Ö¹Ê±Ðò
  * @param
  * @note
  * @retval void
  * @author ÖÇÓùµç×Ó
  */
void Sim_I2C1_STOP(void)
{
	SDA1_OUT();
	Sim_I2C1_SCL_LOW;
	Sim_I2C1_SDA_LOW;
	Sim_I2C1_NOP;

//	Sim_I2C1_SCL_LOW;
//  Sim_I2C1_NOP;

	Sim_I2C1_SCL_HIG;
	Sim_I2C1_SDA_HIG;
	Sim_I2C1_NOP;
}

unsigned char Sim_I2C1_Wait_Ack(void)
{
	volatile unsigned char ucErrTime=0;
	SDA1_IN();
	Sim_I2C1_SDA_HIG;
	Sim_I2C1_NOP;;
	Sim_I2C1_SCL_HIG;
	Sim_I2C1_NOP;;
	while(Sim_I2C1_SDA_STATE)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			Sim_I2C1_STOP();
			return 1;
		}
	}
	Sim_I2C1_SCL_LOW;
	return Sim_I2C1_READY;
}

/**
  * @brief  Ä£ÄâIICÓ¦´ðÊ±Ðò
  * @param
  * @note
  * @retval void
  * @author ÖÇÓùµç×Ó
  */
void Sim_I2C1_SendACK(void)
{
	Sim_I2C1_SCL_LOW;
	SDA1_OUT();
	Sim_I2C1_SDA_LOW;
	Sim_I2C1_NOP;
	Sim_I2C1_SCL_HIG;
	Sim_I2C1_NOP;
	Sim_I2C1_SCL_LOW;
	Sim_I2C1_NOP;
}

/**
  * @brief  Ä£ÄâIICÎÞÓ¦´ðÊ±Ðò
  * @param
  * @note
  * @retval void
  * @author ÖÇÓùµç×Ó
  */
void Sim_I2C1_SendNACK(void)
{
	Sim_I2C1_SCL_LOW;
	SDA1_OUT();
	Sim_I2C1_SDA_HIG;
	Sim_I2C1_NOP;
	Sim_I2C1_SCL_HIG;
	Sim_I2C1_NOP;
	Sim_I2C1_SCL_LOW;
	Sim_I2C1_NOP;
}

/**
  * @brief  Ä£ÄâIIC·¢ËÍµ¥×Ö½ÚÊ±Ðò
  * @param
  * @note
  * @retval void
  * @author ÖÇÓùµç×Ó
  */
uint8_t Sim_I2C1_SendByte(uint8_t Sim_i2c_data)
{
	uint8_t i;
	SDA1_OUT();
	Sim_I2C1_SCL_LOW;
	for(i=0; i<8; i++)
	{
		if(Sim_i2c_data&0x80) Sim_I2C1_SDA_HIG;
		else Sim_I2C1_SDA_LOW;

		Sim_i2c_data<<=1;
		Sim_I2C1_NOP;

		Sim_I2C1_SCL_HIG;
		Sim_I2C1_NOP;
		Sim_I2C1_SCL_LOW;
		Sim_I2C1_NOP;
	}
	return Sim_I2C1_READY;
}

/**
  * @brief  Ä£ÄâIIC¶Áµ¥×Ö½Ú£¬ÎÞÓ¦´ð
  * @param
  * @note
  * @retval void
  * @author ÖÇÓùµç×Ó
  */
uint8_t Sim_I2C1_ReceiveByte(void)
{
	uint8_t i,Sim_i2c_data;
	SDA1_IN();
	//Sim_I2C1_SDA_HIG;
// Sim_I2C1_SCL_LOW;
	Sim_i2c_data=0;

	for(i=0; i<8; i++)
	{
		Sim_I2C1_SCL_LOW;
		Sim_I2C1_NOP;
		Sim_I2C1_SCL_HIG;
		// Sim_I2C1_NOP;
		Sim_i2c_data<<=1;

		if(Sim_I2C1_SDA_STATE)	Sim_i2c_data|=0x01;

		// Sim_I2C1_SCL_LOW;
		Sim_I2C1_NOP;
	}
	Sim_I2C1_SendNACK();
	return Sim_i2c_data;
}

/**
  * @brief  Ä£ÄâIIC¶Áµ¥×Ö½Ú£¬´øÓ¦´ð
  * @param
  * @note
  * @retval void
  * @author ÖÇÓùµç×Ó
  */
uint8_t Sim_I2C1_ReceiveByte_WithACK(void)
{

	uint8_t i,Sim_i2c_data;
	SDA1_IN();
	//Sim_I2C1_SDA_HIG;
// Sim_I2C1_SCL_LOW;
	Sim_i2c_data=0;

	for(i=0; i<8; i++)
	{
		Sim_I2C1_SCL_LOW;
		Sim_I2C1_NOP;
		Sim_I2C1_SCL_HIG;
		// Sim_I2C1_NOP;
		Sim_i2c_data<<=1;

		if(Sim_I2C1_SDA_STATE)	Sim_i2c_data|=0x01;

		// Sim_I2C1_SCL_LOW;
		Sim_I2C1_NOP;
	}
	Sim_I2C1_SendACK();
	return Sim_i2c_data;
}


/**
  * @brief  Ä£ÄâIICµÄ¶à×Ö½Ú¶Á
  * @param
  * @note
  * @retval void
  * @author ÖÇÓùµç×Ó
  */
uint8_t Sim_I2C_Read8(uint8_t moni_dev_addr, uint8_t moni_reg_addr, uint8_t moni_i2c_len, uint8_t *moni_i2c_data_buf)
{

	Sim_I2C1_START();
	Sim_I2C1_SendByte(moni_dev_addr << 1 | I2C1_Direction_Transmitter);
	Sim_I2C1_Wait_Ack();
	Sim_I2C1_SendByte(moni_reg_addr);
	Sim_I2C1_Wait_Ack();
	//Sim_I2C1_STOP();
	
	Sim_I2C1_START();
	Sim_I2C1_SendByte(moni_dev_addr << 1 | I2C1_Direction_Receiver);
	Sim_I2C1_Wait_Ack();
	while (moni_i2c_len)
	{
		if (moni_i2c_len==1) *moni_i2c_data_buf =Sim_I2C1_ReceiveByte();
		else *moni_i2c_data_buf =Sim_I2C1_ReceiveByte_WithACK();
		moni_i2c_data_buf++;
		moni_i2c_len--;
	}
	Sim_I2C1_STOP();
	return 0x00;
}

/*******************************************************************************/
/**
  * @brief  Ä£ÄâIICµÄ¶à×Ö½ÚÐ´
  * @param
  * @note   µ±ÆôÓÃcheck¹¦ÄÜµÄÊ±ºò£¬Ö»ÄÜÊÇµ¥×Ö½ÚÐ´µÄÇé¿ö£¬¶à×Ö½ÓÐ´²»¿ÉÆôÓÃcheck¹¦ÄÜ
  * @retval void
  * @author ÖÇÓùµç×Ó
  */
int8_t Sim_I2C1_Write8(uint8_t moni_dev_addr, uint8_t moni_reg_addr, uint8_t moni_i2c_len, uint8_t *moni_i2c_data_buf)
{
	uint8_t i;
	Sim_I2C1_START();
	Sim_I2C1_SendByte(moni_dev_addr << 1 | I2C1_Direction_Transmitter);
	Sim_I2C1_Wait_Ack();
	Sim_I2C1_SendByte(moni_reg_addr);
	Sim_I2C1_Wait_Ack();
	
	//Sim_I2C1_START();
	for (i=0; i<moni_i2c_len; i++)
	{
		Sim_I2C1_SendByte(moni_i2c_data_buf[i]);
		Sim_I2C1_Wait_Ack();
	}
	Sim_I2C1_STOP();	
		return 0;
}


/************************************************************************************************************/
/************************************************************************************************************/
/*******************************************************************************/


uint8_t highByte(uint16_t value)
{
	uint8_t ret;
	value = value>>8;
	ret = (uint8_t)value;
  return ret;
}

uint8_t lowByte(uint16_t value)
{
	uint8_t ret;
	value = value&0x00ff;
	ret = (uint8_t)value;
  return ret;
}
/*******************************************************
 Method: readOneByte
 In: register to read
 Out: data read from i2c
 Description: reads one byte register from i2c
******************************************************/
uint8_t readOneByte(uint8_t in_adr)
{
  uint8_t retVal = -1;
	
	Sim_I2C_Read8(_ams5600_Address,in_adr,1,&retVal);
	Sim_I2C1_NOP;
  return retVal;
}

/*******************************************************
 Method: readOnTwoByte
 In: two registers to read
 Out: data read from i2c as a int16_t
 Description: reads two bytes register from i2c
******************************************************/
uint16_t readTwoBytes(uint8_t in_adr_hi, uint8_t in_adr_lo)
{
  uint16_t retVal = -1;
  uint8_t low=0,high=0;
	
  /* Read Low Byte */
	low = readOneByte(in_adr_lo);
	
	
  /* Read High Byte */  
  high = readOneByte(in_adr_hi);
  
	//printf("high:%d,low:%d  ",high,low);
  retVal = high << 8;
  retVal = retVal | low;
  //printf("retVal:%d\r\n",retVal);
  return retVal;
}


/*******************************************************
 Method: writeOneByte
 In: address and data to write
 Out: none
 Description: writes one byte to a i2c register
******************************************************/
void writeOneByte(uint8_t adr_in, uint8_t dat_in)
{
	uint8_t dat = dat_in;
  Sim_I2C1_Write8(_ams5600_Address,adr_in,1,&dat);
}

/****************************************************
 Method: AMS_5600
 In: none
 Out: i2c address of AMS 5600
 Description: returns i2c address of AMS 5600
   **************************************************/
int16_t getAddress()
{
  return _ams5600_Address; 
}

/*******************************************************
 Method: getMaxAngle
 In: none
 Out: value of max angle register
 Description: gets value of maximum angle register.		èŽ·å–æœ€å¤§è§’åº¦å¯„å­˜å™¨(MANG)çš„å€¼ï¼š05H 06H
******************************************************/
int16_t getMaxAngle(void)
{
  return readTwoBytes(_mang_hi, _mang_lo);
}

/*******************************************************
 Method: getRawAngle
 In: none
 Out: value of raw angle register
 Description: gets raw value of magnet position.
 start, end, and max angle settings do not apply.			èŽ·å–æœªç¼©æ”¾å’Œæœªä¿®æ”¹è§’åº¦å¯„å­˜å™¨(RAM ANGLE)çš„å€¼ï¼š0CH 0DH
******************************************************/
int16_t getRawAngle(void)
{
  return readTwoBytes(_raw_ang_hi, _raw_ang_lo);
}

/*******************************************************
 Method: getStartPosition
 In: none
 Out: value of start position register
 Description: gets value of start position register.	èŽ·å–èµ·å§‹ä½ç½®å¯„å­˜å™¨(ZPOS)çš„å€¼ï¼š01H 02H
******************************************************/
int16_t getStartPosition(void)
{
  return readTwoBytes(_zpos_hi, _zpos_lo);
}  

/*******************************************************
 Method: getEndPosition
 In: none
 Out: value of end position register
 Description: gets value of end position register.		èŽ·å–åœæ­¢ä½ç½®å¯„å­˜å™¨(MPOS)çš„å€¼ï¼š 03H 04H
******************************************************/
int16_t getEndPosition(void)
{
  int16_t retVal = readTwoBytes(_mpos_hi, _mpos_lo);
  return retVal;
}

/*******************************************************
 Method: getScaledAngle
 In: none
 Out: value of scaled angle register
 Description: gets scaled value of magnet position.
 start, end, or max angle settings are used to 
 determine value																			èŽ·å–ç£ä½“ä½ç½®çš„ç¼©æ”¾å€¼ï¼Œå¯åœ¨ANGLEå¯„å­˜å™¨ä¸­è¯»å–ï¼š 0EH 0FH
******************************************************/
int16_t getScaledAngle(void)
{
  return readTwoBytes(_ang_hi, _ang_lo);
}

/*******************************************************
 Method: get Agc
 In: none
 Out: value of AGC register
 Description: gets value of AGC register.							AGCå¯„å­˜å™¨æŒ‡ç¤ºå¢žç›Šã€‚ä¸ºäº†èŽ·å¾—æœ€ç¨³å®šçš„æ€§èƒ½ï¼Œå¢žç›Šå€¼åº”åœ¨å…¶èŒƒå›´çš„ä¸­å¿ƒï¼š1AH
******************************************************/
int16_t getAgc()
{
  return readOneByte(_agc);
}

/*******************************************************
 Method: getMagnitude
 In: none
 Out: value of magnitude register
 Description: gets value of magnitude register.				MAGNITUDEå¯„å­˜å™¨æŒ‡ç¤ºå†…éƒ¨CORDICçš„å¹…å€¼ï¼š1BH 1CH
******************************************************/
int16_t getMagnitude()
{
  return readTwoBytes(_mag_hi, _mag_lo);  
}

/*******************************************************
 Method: getBurnCount
 In: none
 Out: value of zmco register
 Description: determines how many times chip has been
 permanently written to. 															ç¡®å®šèŠ¯ç‰‡è¢«æ°¸ä¹…å†™å…¥çš„æ¬¡æ•°ï¼šZMOCå¯„å­˜å™¨åœ°å€æ˜¯00H
******************************************************/
int16_t getBurnCount()
{
  return readOneByte(_zmco);
}
/*******************************************************
 Method: getRawAngle
 In: none
 Out: value of raw angle register
 Description: gets raw value of magnet position.
 start, end, and max angle settings do not apply			èŽ·å–æœªç¼©æ”¾å’Œæœªä¿®æ”¹è§’åº¦å¯„å­˜å™¨(RAM ANGLE)çš„å€¼ï¼š0CH 0DH
******************************************************/
int16_t AgetRawAngle(void)
{
  return readTwoBytes(_raw_ang_hi, _raw_ang_lo);
}
/*******************************************************
 Method: setEndtPosition
 In: new end angle position
 Out: value of end position register
 Description: sets a value in end position register.
 If no value is provided, method will read position of
 magnet.  																						åœ¨ç»“æŸä½ç½®å¯„å­˜å™¨MPOS(03H 04H)ä¸­è®¾ç½®ä¸€ä¸ªå€¼ã€‚
******************************************************/
int16_t setEndPosition(int16_t endAngle)
{
  if(endAngle == -1)
    _rawEndAngle = getRawAngle();
  else
    _rawEndAngle = endAngle;
 
  writeOneByte(_mpos_hi, highByte(_rawEndAngle));
  HAL_Delay(2); 
  writeOneByte(_mpos_lo, lowByte(_rawEndAngle)); 
  HAL_Delay(2);               
  _mPosition = readTwoBytes(_mpos_hi, _mpos_lo);
  
  return(_mPosition);
}



/*******************************************************
 Method: setStartPosition
 In: new start angle position
 Out: value of start position register
 Description: sets a value in start position register.
 If no value is provided, method will read position of
 magnet.  
******************************************************/
int16_t setStartPosition(int16_t startAngle)
{
  if(startAngle == -1)
  {
    _rawStartAngle = getRawAngle();
  }
  else
    _rawStartAngle = startAngle;

  writeOneByte(_zpos_hi, highByte(_rawStartAngle));
  HAL_Delay(2); 
  writeOneByte(_zpos_lo, lowByte(_rawStartAngle)); 
  HAL_Delay(2);                
  _zPosition = readTwoBytes(_zpos_hi, _zpos_lo);
  
  return(_zPosition);
}

/*******************************************************
 Method: setMaxAngle
 In: new maximum angle to set OR none
 Out: value of max angle register
 Description: sets a value in maximum angle register.
 If no value is provided, method will read position of
 magnet.  Setting this register zeros out max position
 register.
******************************************************/
int16_t setMaxAngle(int16_t newMaxAngle)
{
  int32_t retVal;
  if(newMaxAngle == -1)
  {
    _maxAngle = getRawAngle();
  }
  else
    _maxAngle = newMaxAngle;

  writeOneByte(_mang_hi, highByte(_maxAngle));
  HAL_Delay(2);
  writeOneByte(_mang_lo, lowByte(_maxAngle)); 
  HAL_Delay(2);     

  retVal = readTwoBytes(_mang_hi, _mang_lo);
  return retVal;
}

/*******************************************************
 Method: detectMagnet
 In: none
 Out: 1 if magnet is detected, 0 if not
 Description: reads status register and examines the 
 MH bit
******************************************************/
uint8_t detectMagnet(void)
{
  uint8_t magStatus;
  uint8_t retVal = 0;
  /*0 0 MD ML MH 0 0 0*/
  /* MD high = AGC minimum overflow, Magnet to strong */
  /* ML high = AGC Maximum overflow, magnet to weak*/ 
  /* MH high = magnet detected*/ 
  magStatus = readOneByte(_stat);
  
  if(magStatus & 0x20)
    retVal = 1; 
  
  return retVal;
}

/*******************************************************
 Method: getMagnetStrength
 In: none
 Out: 0 if no magnet is detected
      1 if magnet is to weak
      2 if magnet is just right
      3 if magnet is to strong
 Description: reads status register andexamins the MH,ML,MD bits
******************************************************/
uint8_t getMagnetStrength(void)
{
  uint8_t magStatus;
  uint8_t retVal = 0;
  /*0 0 MD ML MH 0 0 0*/
  /* MD high = AGC minimum overflow, Magnet to strong */
  /* ML high = AGC Maximum overflow, magnet to weak*/ 
  /* MH high = magnet detected*/ 
  magStatus = readOneByte(_stat);
  if(detectMagnet() ==1)
  {
      retVal = 2; /*just right */
      if(magStatus & 0x10)
        retVal = 1; /*to weak */
      else if(magStatus & 0x08)
        retVal = 3; /*to strong */
  }
  
  return retVal;
}

/*******************************************************
 Method: burnAngle
 In: none
 Out: 1 success
     -1 no magnet
     -2 burn limit exceeded
     -3 start and end positions not set (useless burn)
 Description: burns start and end positions to chip.
 THIS CAN ONLY BE DONE 3 TIMES
******************************************************/
uint8_t burnAngle()
{
  uint8_t retVal = 1;
  _zPosition = getStartPosition();
  _mPosition = getEndPosition();
  _maxAngle  = getMaxAngle();
  
  if(detectMagnet() == 1)
  {
    if(getBurnCount() < 3)
    {
      if((_zPosition == 0)&&(_mPosition ==0))
        retVal = -3;
      else
        writeOneByte(_burn, 0x80);
    }
    else
      retVal = -2;
  } 
  else
    retVal = -1;
    
  return retVal;
}

/*******************************************************
 Method: burnMaxAngleAndConfig
 In: none
 Out: 1 success
     -1 burn limit exceeded
     -2 max angle is to small, must be at or above 18 degrees
 Description: burns max angle and config data to chip.
 THIS CAN ONLY BE DONE 1 TIME
******************************************************/
uint8_t burnMaxAngleAndConfig()
{
  uint8_t retVal = 1;
  _maxAngle  = getMaxAngle();
  
  if(getBurnCount() ==0)
  {
    if(_maxAngle*0.087 < 18)
      retVal = -2;
    else
      writeOneByte(_burn, 0x40);    
  }  
  else
    retVal = -1;
    
  return retVal;
}

/**
  * @brief  ´®¿ÚÊäÈë³ÌÐò
  * @param
  * @note   
  * @retval ·µ»ØµÚÒ»¸ö½ÓÊÕµÄ×Ö½Ú
  * @author ÖÇÓùµç×Ó
  */
uint8_t PrintInFromConsole(void)
{
	 uint8_t data;
   //HAL_UART_Receive(&huart1, &data, 1, 0xFFFF);
	 return data;
}

/**
  * @brief  ´®¿Ú½»»¥½çÃæ
  * @param
  * @note   
  * @retval void
  * @author ÖÇÓùµç×Ó
  */
int8_t  MenuChoice=0;
void PrintMenu(void)
{
  printf("AS5600 Serial Interface Programe----SC-electronic\r\n");
	printf("1 - Set start position\t|  "); printf(" 6 - get MPOS\r\n");
  printf("2 - Set end position\t|  ");   printf(" 7 - get raw angle\r\n");
  printf("3 - Set max angle range\t|  ");  printf(" 8 - get scaled angle\r\n");
  printf("4 - Get max angle range\t|  ");  printf(" 9 - detect magnet\r\n");
  printf("5 - Get ZPOS \t\t|  ");     printf("10 - get magnet strength\r\n");
  printf("\r\n");
  printf("Number of burns remaining: "); printf("%d\r\n",(3 - getBurnCount()));
  printf("96 - Burn Angle\r\n");
  printf("98 - Burn Settings (one time)\r\n");
	MenuChoice = PrintInFromConsole();
	printf("You choice NO.%d programe\r\n",MenuChoice);
}

/*******************************************************
 Function: convertRawAngleToDegrees
 In: angle data from AMS_5600::getRawAngle
 Out: human readable degrees as float
 Description: takes the raw angle and calculates 
 float value in degrees.
******************************************************/
float convertRawAngleToDegrees(int16_t newAngle)
{
  /* Raw data reports 0 - 4095 segments, which is 0.087 of a degree */    
  float retVal = newAngle * 0.087;
  return retVal;
}

uint16_t rawdata=0;
float degress =0;
void Programe_Run(void)
{
	uint8_t dect= 0;
	dect = detectMagnet();
	printf("detectMagnet is %d\r\n",dect);
	rawdata = getRawAngle();
	printf("rawdata is %d\r\n",rawdata);
	degress = convertRawAngleToDegrees(rawdata);
	printf("degress is %f\r\n",degress);
}
