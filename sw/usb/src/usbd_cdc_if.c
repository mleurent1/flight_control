/**
  ******************************************************************************
  * @file    usbd_cdc_if.c
  * @author  MCD Application Team
  * @version V2.4.2
  * @date    11-December-2015
  * @brief   Generic media access Layer.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */ 

#include "usbd_cdc_if.h"

USBD_HandleTypeDef USBD_device_handler;

uint8_t USBD_CDC_Itf_rx_buffer[USBD_CDC_ITF_RX_SIZE];
uint8_t USBD_CDC_Itf_tx_buffer[USBD_CDC_ITF_TX_SIZE];

static int8_t USBD_CDC_Itf_Init     (void);
static int8_t USBD_CDC_Itf_DeInit   (void);
static int8_t USBD_CDC_Itf_Control  (uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t USBD_CDC_Itf_Receive  (uint8_t* pbuf, uint32_t *Len);

extern void HostCommand(uint8_t* buffer);

USBD_CDC_ItfTypeDef USBD_CDC_Itf_fops = 
{
  USBD_CDC_Itf_Init,
  USBD_CDC_Itf_DeInit,
  USBD_CDC_Itf_Control,
  USBD_CDC_Itf_Receive
};

USBD_CDC_LineCodingTypeDef linecoding =
  {
    115200, /* baud rate*/
    0x00,   /* stop bits-1*/
    0x00,   /* parity - none*/
    0x08    /* nb. of bits 8*/
  };

/**
  * @brief  USBD_CDC_Itf_Init
  *         Initializes the CDC media low layer
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t USBD_CDC_Itf_Init(void)
{
	USBD_CDC_SetTxBuffer(&USBD_device_handler, USBD_CDC_Itf_tx_buffer, 0);
	USBD_CDC_SetRxBuffer(&USBD_device_handler, USBD_CDC_Itf_rx_buffer);
	return (USBD_OK);
}

/**
  * @brief  USBD_CDC_Itf_DeInit
  *         DeInitializes the CDC media low layer
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t USBD_CDC_Itf_DeInit(void)
{
  /*
     Add your deinitialization code here 
  */  
  return (USBD_OK);
}


/**
  * @brief  USBD_CDC_Itf_Control
  *         Manage the CDC class requests
  * @param  Cmd: Command code            
  * @param  Buf: Buffer containing command data (request parameters)
  * @param  Len: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t USBD_CDC_Itf_Control  (uint8_t cmd, uint8_t* pbuf, uint16_t length)
{ 
  switch (cmd)
  {
  case CDC_SEND_ENCAPSULATED_COMMAND:
    /* Add your code here */
    break;

  case CDC_GET_ENCAPSULATED_RESPONSE:
    /* Add your code here */
    break;

  case CDC_SET_COMM_FEATURE:
    /* Add your code here */
    break;

  case CDC_GET_COMM_FEATURE:
    /* Add your code here */
    break;

  case CDC_CLEAR_COMM_FEATURE:
    /* Add your code here */
    break;

  case CDC_SET_LINE_CODING:
    linecoding.bitrate    = (uint32_t)(pbuf[0] | (pbuf[1] << 8) |\
                            (pbuf[2] << 16) | (pbuf[3] << 24));
    linecoding.format     = pbuf[4];
    linecoding.paritytype = pbuf[5];
    linecoding.datatype   = pbuf[6];
    
    /* Add your code here */
    break;

  case CDC_GET_LINE_CODING:
    pbuf[0] = (uint8_t)(linecoding.bitrate);
    pbuf[1] = (uint8_t)(linecoding.bitrate >> 8);
    pbuf[2] = (uint8_t)(linecoding.bitrate >> 16);
    pbuf[3] = (uint8_t)(linecoding.bitrate >> 24);
    pbuf[4] = linecoding.format;
    pbuf[5] = linecoding.paritytype;
    pbuf[6] = linecoding.datatype;     
    
    /* Add your code here */
    break;

  case CDC_SET_CONTROL_LINE_STATE:
    /* Add your code here */
    break;

  case CDC_SEND_BREAK:
     /* Add your code here */
    break;    
    
  default:
    break;
  }

  return (USBD_OK);
}

/**
  * @brief  USBD_CDC_Itf_Receive
  *         Data received over USB OUT endpoint are sent over CDC interface 
  *         through this function.
  *           
  *         @note
  *         This function will issue a NAK packet on any OUT packet received on 
  *         USB endpoint untill exiting this function. If you exit this function
  *         before transfer is complete on CDC interface (ie. using DMA controller)
  *         it will result in receiving more data while previous ones are still 
  *         not sent.
  *                 
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t USBD_CDC_Itf_Receive (uint8_t* Buf, uint32_t *Len)
{
	if (*Len == 6)
	{
		HostCommand(Buf);
	}
	USBD_CDC_ReceivePacket(&USBD_device_handler);
	return (USBD_OK);
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

