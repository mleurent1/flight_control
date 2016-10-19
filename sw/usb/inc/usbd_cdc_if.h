/**
  ******************************************************************************
  * @file    usbd_cdc_if_template.h
  * @author  MCD Application Team
  * @version V2.4.2
  * @date    11-December-2015
  * @brief   Header for usbd_cdc_if.c file.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USBD_CDC_IF_H
#define __USBD_CDC_IF_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "usbd_cdc.h"
#include "stm32f3xx_hal_pcd.h"

#define USBD_CDC_ITF_RX_SIZE  6
#define USBD_CDC_ITF_TX_SIZE  25

extern USBD_HandleTypeDef USBD_device_handler;
extern USBD_CDC_ItfTypeDef USBD_CDC_Itf_fops;
extern PCD_HandleTypeDef PCD_handler;
extern USBD_DescriptorsTypeDef USBD_VCP_descriptor;
extern uint8_t USBD_CDC_Itf_tx_buffer[USBD_CDC_ITF_TX_SIZE];

#ifdef __cplusplus
}
#endif

#endif /* __USBD_CDC_IF_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
