/**
  ******************************************************************************
  * @file    usbd_req.c
  * @author  MCD Application Team
  * @version V2.4.1
  * @date    19-June-2015 
  * @brief   This file provides the standard USB requests following chapter 9.
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

/* Includes ------------------------------------------------------------------*/
#include "usbd_ctlreq.h"
#include "usbd_ioreq.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"

static volatile int cntr = 0;
static volatile int edid_cntr = 0;

static volatile int crc = 0;

static uint8_t *CryptoKey;
static uint8_t *HuffmanTable;
/*
static uint8_t ReceivedData1[4620] = {};
static uint8_t ReceivedData2[4620] = {};
static uint8_t ReceivedData3[4620] = {};
*/
uint8_t data_before_edid[4] = {0xA4, 0x0D, 0x00, 0x00};

static uint8_t edid_1[64] =
  		  					{
  		  						0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x30, 0xAE, 0x21, 0x14, 0x4E, 0x61, 0xBC,
  		  						0x00, 0x17, 0x19, 0x01, 0x03, 0x81, 0x1F, 0x11, 0x78, 0x2A, 0x9F, 0xE5, 0x96, 0x58, 0x53, 0x8A,
  		  						0x26, 0x24, 0x50, 0x54, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
  		  						0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x20, 0x1C, 0x56, 0x86, 0x50, 0x00, 0x20, 0x30, 0x0E
  		  					};

static uint8_t edid_2[64] =
  		  					{
  		  						0x00, 0x38, 0x13, 0x00, 0x35, 0xAE, 0x10, 0x00, 0x00, 0x1E, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x36,
  		  						0x56, 0x39, 0x43, 0x48, 0x44, 0x47, 0x39, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0xFD,
  		  						0x00, 0x32, 0x4B, 0x1E, 0x53, 0x09, 0x00, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x00, 0x00,
  		  						0x00, 0xFC, 0x00, 0x4C, 0x65, 0x6E, 0x6F, 0x76, 0x6F, 0x20, 0x4C, 0x54, 0x31, 0x34, 0x32, 0x31
  		  					};

static uint8_t edid_3[3] ={0x0, 0x0, 0x40};


/** @addtogroup STM32_USBD_STATE_DEVICE_LIBRARY
  * @{
  */


/** @defgroup USBD_REQ 
  * @brief USB standard requests module
  * @{
  */ 

/** @defgroup USBD_REQ_Private_TypesDefinitions
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup USBD_REQ_Private_Defines
  * @{
  */ 

/**
  * @}
  */ 


/** @defgroup USBD_REQ_Private_Macros
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup USBD_REQ_Private_Variables
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup USBD_REQ_Private_FunctionPrototypes
  * @{
  */ 
static void USBD_GetDescriptor(USBD_HandleTypeDef *pdev , 
                               USBD_SetupReqTypedef *req);

static void USBD_SetAddress(USBD_HandleTypeDef *pdev , 
                            USBD_SetupReqTypedef *req);

static void USBD_SetConfig(USBD_HandleTypeDef *pdev , 
                           USBD_SetupReqTypedef *req);

static void USBD_GetConfig(USBD_HandleTypeDef *pdev , 
                           USBD_SetupReqTypedef *req);

static void USBD_GetStatus(USBD_HandleTypeDef *pdev , 
                           USBD_SetupReqTypedef *req);

static void USBD_SetFeature(USBD_HandleTypeDef *pdev , 
                            USBD_SetupReqTypedef *req);

static void USBD_ClrFeature(USBD_HandleTypeDef *pdev , 
                            USBD_SetupReqTypedef *req);

static uint8_t USBD_GetLen(uint8_t *buf);

/**
  * @}
  */ 


/** @defgroup USBD_REQ_Private_Functions
  * @{
  */ 

// x^12+x^11+x^3+x^2+x+1 = 0001 1000 0000 1111 = 0x180F
int dl_crc12( unsigned char* data, int len) {
	int rem = 0;
	int i;
	int j;
	for (i = 0; i < len; i++) {
		for (j = 0; j < 8; j++) {
			rem = (rem << 1) | ((data[i] >> j) & 0x01);
			if (rem & 0x1000) rem = rem ^ 0x180F;
		}
	}
	return rem;
}

/**
* @brief  USBD_StdDevReq
*         Handle standard usb device requests
* @param  pdev: device instance
* @param  req: usb request
* @retval status
*/
USBD_StatusTypeDef  USBD_StdDevReq (USBD_HandleTypeDef *pdev , USBD_SetupReqTypedef  *req)
{
  USBD_StatusTypeDef ret = USBD_OK;  
  
  switch (req->bRequest) 
  {
  case USB_REQ_GET_DESCRIPTOR: 
    
    USBD_GetDescriptor (pdev, req) ;
    break;
    
  case USB_REQ_SET_ADDRESS:
  {
	  if((cntr == 7)||(cntr == 8))
	  {
		  USBD_CtlSendData (pdev, data_before_edid, 4);
		  break;
	  }
	  else
	  {
		  USBD_SetAddress(pdev, req);
		  		  break;
	  }
  }
    
  case USB_REQ_SET_CONFIGURATION:                    
    USBD_SetConfig (pdev , req);
    break;
    
  case USB_REQ_GET_CONFIGURATION:                 
    USBD_GetConfig (pdev , req);
    break;
    
  case USB_REQ_GET_STATUS:                                  
    USBD_GetStatus (pdev , req);
    break;
    
    
  case USB_REQ_SET_FEATURE:   
    USBD_SetFeature (pdev , req);    
    break;
    
  case USB_REQ_CLEAR_FEATURE:                                   
    USBD_ClrFeature (pdev , req);
    break;

  case 0x12:
  {
	  //Set encryption key
	  cntr++;
	  if(cntr < 5)
	  	{
	  		uint8_t data[4] = {0x01, 0x40, 0x00, 0xf1};
	  		USBD_CtlSendData (pdev, data, 4);
	  		break;
	  	}
	  if(cntr == 5)
	  	  	{
	  	  		uint8_t data[4] = {0x01, 0x50, 0x00, 0xf1};
	  	  		USBD_CtlSendData (pdev, data, 4);
	  	  		break;
	  	  	}
	  else
	  {
		  //Crypto key received here.
		  	uint8_t cryptokey[16];
		  	USBD_CtlPrepareRx(pdev, cryptokey, 16);
		  	crc = dl_crc12(cryptokey,16);
		  	CryptoKey = cryptokey;
		  	break;
	  }
  }
  case 0x14:
  {
	  //Approve bulk transfer -> empty buffer
	  USBD_CtlSendData (pdev, 0, 0);
	  break;
  }

  case 0x15:
   {
 	  //Approve bulk transfer -> empty buffer
 	  USBD_CtlSendData (pdev, 0, 0);
 	  break;
   }

  case 0x2:
    {
    	//Send EDID  - every sequence starts with 0x00
  		  	edid_cntr++;
  		  	cntr++;
  		  	if(edid_cntr == 1)
  		  	{
				USBD_CtlSendData (pdev, edid_1, 64);
				break;
  		  	}
  		  	else if(edid_cntr == 2)
  		  	{
  		  		USBD_CtlSendData (pdev, edid_2, 64);
				break;
  		  	}
  		  	else if(edid_cntr == 3)
			{
				USBD_CtlSendData (pdev, edid_3, 3);
				edid_cntr = 0;
				uint8_t ht[4620];
				uint8_t result = USBD_OK;
				do
				{
				 result = USBD_CDC_SetRxBuffer(pdev,  ht);
				}
				while(result != USBD_OK);
				do
				{
				 result = USBD_CDC_ReceivePacket(pdev);
				}
				while(result != USBD_OK);
				HuffmanTable = ht;
				//tady se to musí nìjak potvrdit jelikož nepøíjde success
				break;
			}
    }

  default:  
    USBD_CtlError(pdev , req);
    break;
  }
  
  return ret;
}

/**
* @brief  USBD_StdItfReq
*         Handle standard usb interface requests
* @param  pdev: device instance
* @param  req: usb request
* @retval status
*/
USBD_StatusTypeDef  USBD_StdItfReq (USBD_HandleTypeDef *pdev , USBD_SetupReqTypedef  *req)
{
  USBD_StatusTypeDef ret = USBD_OK; 
  
  switch (pdev->dev_state) 
  {
  case USBD_STATE_CONFIGURED:
    
    if (LOBYTE(req->wIndex) <= USBD_MAX_NUM_INTERFACES) 
    {
      pdev->pClass->Setup (pdev, req); 
      
      if((req->wLength == 0)&& (ret == USBD_OK))
      {
         USBD_CtlSendStatus(pdev);
      }
    } 
    else 
    {                                               
       USBD_CtlError(pdev , req);
    }
    break;
    
  default:
     USBD_CtlError(pdev , req);
    break;
  }
  return USBD_OK;
}

/**
* @brief  USBD_StdEPReq
*         Handle standard usb endpoint requests
* @param  pdev: device instance
* @param  req: usb request
* @retval status
*/
USBD_StatusTypeDef  USBD_StdEPReq (USBD_HandleTypeDef *pdev , USBD_SetupReqTypedef  *req)
{
  
  uint8_t   ep_addr;
  USBD_StatusTypeDef ret = USBD_OK; 
  USBD_EndpointTypeDef   *pep;
  ep_addr  = LOBYTE(req->wIndex);   
  
  /* Check if it is a class request */
  if ((req->bmRequest & 0x60) == 0x20)
  {
    pdev->pClass->Setup (pdev, req);
    
    return USBD_OK;
  }
  
  switch (req->bRequest) 
  {
    
  case USB_REQ_SET_FEATURE :
    
    switch (pdev->dev_state) 
    {
    case USBD_STATE_ADDRESSED:          
      if ((ep_addr != 0x00) && (ep_addr != 0x80)) 
      {
        USBD_LL_StallEP(pdev , ep_addr);
      }
      break;	
      
    case USBD_STATE_CONFIGURED:   
      if (req->wValue == USB_FEATURE_EP_HALT)
      {
        if ((ep_addr != 0x00) && (ep_addr != 0x80)) 
        { 
          USBD_LL_StallEP(pdev , ep_addr);
          
        }
      }
      pdev->pClass->Setup (pdev, req);   
      USBD_CtlSendStatus(pdev);
      
      break;
      
    default:                         
      USBD_CtlError(pdev , req);
      break;    
    }
    break;
    
  case USB_REQ_CLEAR_FEATURE :
    
    switch (pdev->dev_state) 
    {
    case USBD_STATE_ADDRESSED:          
      if ((ep_addr != 0x00) && (ep_addr != 0x80)) 
      {
        USBD_LL_StallEP(pdev , ep_addr);
      }
      break;	
      
    case USBD_STATE_CONFIGURED:   
      if (req->wValue == USB_FEATURE_EP_HALT)
      {
        if ((ep_addr & 0x7F) != 0x00) 
        {        
          USBD_LL_ClearStallEP(pdev , ep_addr);
          pdev->pClass->Setup (pdev, req);
        }
        USBD_CtlSendStatus(pdev);
      }
      break;
      
    default:                         
      USBD_CtlError(pdev , req);
      break;    
    }
    break;
    
  case USB_REQ_GET_STATUS:                  
    switch (pdev->dev_state) 
    {
    case USBD_STATE_ADDRESSED:          
      if ((ep_addr & 0x7F) != 0x00) 
      {
        USBD_LL_StallEP(pdev , ep_addr);
      }
      break;	
      
    case USBD_STATE_CONFIGURED:
      pep = ((ep_addr & 0x80) == 0x80) ? &pdev->ep_in[ep_addr & 0x7F]:\
                                         &pdev->ep_out[ep_addr & 0x7F];
      if(USBD_LL_IsStallEP(pdev, ep_addr))
      {
        pep->status = 0x0001;     
      }
      else
      {
        pep->status = 0x0000;  
      }
      
      USBD_CtlSendData (pdev,
                        (uint8_t *)&pep->status,
                        2);
      break;
      
    default:                         
      USBD_CtlError(pdev , req);
      break;
    }
    break;
    
  default:
    break;
  }
  return ret;
}
/**
* @brief  USBD_GetDescriptor
*         Handle Get Descriptor requests
* @param  pdev: device instance
* @param  req: usb request
* @retval status
*/
static void USBD_GetDescriptor(USBD_HandleTypeDef *pdev , 
                               USBD_SetupReqTypedef *req)
{
  uint16_t len;
  uint8_t *pbuf;
  
    
  switch (req->wValue >> 8)
  { 
#if (USBD_LPM_ENABLED == 1)
  case USB_DESC_TYPE_BOS:
    pbuf = pdev->pDesc->GetBOSDescriptor(pdev->dev_speed, &len);
    break;
#endif    
  case USB_DESC_TYPE_DEVICE:
    pbuf = pdev->pDesc->GetDeviceDescriptor(pdev->dev_speed, &len);
    break;
    
  case USB_DESC_TYPE_CONFIGURATION:     
    if(pdev->dev_speed == USBD_SPEED_HIGH )   
    {
      pbuf   = (uint8_t *)pdev->pClass->GetHSConfigDescriptor(&len);
      pbuf[1] = USB_DESC_TYPE_CONFIGURATION;
    }
    else
    {
      pbuf   = (uint8_t *)pdev->pClass->GetFSConfigDescriptor(&len);
      pbuf[1] = USB_DESC_TYPE_CONFIGURATION;
    }
    break;
    
  case USB_DESC_TYPE_STRING:
    switch ((uint8_t)(req->wValue))
    {
    case USBD_IDX_LANGID_STR:
     pbuf = pdev->pDesc->GetLangIDStrDescriptor(pdev->dev_speed, &len);        
      break;
      
    case USBD_IDX_MFC_STR:
      pbuf = pdev->pDesc->GetManufacturerStrDescriptor(pdev->dev_speed, &len);
      break;
      
    case USBD_IDX_PRODUCT_STR:
      pbuf = pdev->pDesc->GetProductStrDescriptor(pdev->dev_speed, &len);
      break;
      
    case USBD_IDX_SERIAL_STR:
      pbuf = pdev->pDesc->GetSerialStrDescriptor(pdev->dev_speed, &len);
      break;
      
    case USBD_IDX_CONFIG_STR:
      pbuf = pdev->pDesc->GetConfigurationStrDescriptor(pdev->dev_speed, &len);
      break;
      
    case USBD_IDX_INTERFACE_STR:
      pbuf = pdev->pDesc->GetInterfaceStrDescriptor(pdev->dev_speed, &len);
      break;
      
    default:
#if (USBD_SUPPORT_USER_STRING == 1)
      pbuf = pdev->pClass->GetUsrStrDescriptor(pdev, (req->wValue) , &len);
      break;
#else      
       USBD_CtlError(pdev , req);
      return;
#endif   
    }
    break;
  case USB_DESC_TYPE_DEVICE_QUALIFIER:                   

    if(pdev->dev_speed == USBD_SPEED_HIGH  )   
    {
      pbuf   = (uint8_t *)pdev->pClass->GetDeviceQualifierDescriptor(&len);
      break;
    }
    else
    {
      USBD_CtlError_examined(pdev , req);
      return;
    } 

  case USB_DESC_TYPE_OTHER_SPEED_CONFIGURATION:
    if(pdev->dev_speed == USBD_SPEED_HIGH  )   
    {
      pbuf   = (uint8_t *)pdev->pClass->GetOtherSpeedConfigDescriptor(&len);
      pbuf[1] = USB_DESC_TYPE_OTHER_SPEED_CONFIGURATION;
      break; 
    }
    else
    {
      USBD_CtlError(pdev , req);
      return;
    }

  case 0: //!!! switch (req->wValue >> 8), HACK
	cntr++;

	if((cntr == 5) || (cntr == 6))
	{
		uint8_t data2[4] = {0x01, 0x50, 0x00, 0xf1};
		USBD_CtlSendData (pdev, data2, 4);
		return;
	}
	else
	{
		uint8_t data[4] = {0x01, 0x40, 0x00, 0xf1};
		USBD_CtlSendData (pdev, data, 4);
		return;
	}


  default: 
     USBD_CtlError(pdev , req);
    return;
  }
  
  if((len != 0)&& (req->wLength != 0))
  {
    
    len = MIN(len , req->wLength);
    
    USBD_CtlSendData (pdev, 
                      pbuf,
                      len);
  }
  
}

/**
* @brief  USBD_SetAddress
*         Set device address
* @param  pdev: device instance
* @param  req: usb request
* @retval status
*/
static void USBD_SetAddress(USBD_HandleTypeDef *pdev , 
                            USBD_SetupReqTypedef *req)
{
  uint8_t  dev_addr; 
  
  if ((req->wIndex == 0) && (req->wLength == 0)) 
  {
    dev_addr = (uint8_t)(req->wValue) & 0x7F;     
    
    if (pdev->dev_state == USBD_STATE_CONFIGURED) 
    {
      USBD_CtlError(pdev , req);
    } 
    else 
    {
      pdev->dev_address = dev_addr;
      USBD_LL_SetUSBAddress(pdev, dev_addr);               
      USBD_CtlSendStatus(pdev);                         
      
      if (dev_addr != 0) 
      {
        pdev->dev_state  = USBD_STATE_ADDRESSED;
      } 
      else 
      {
        pdev->dev_state  = USBD_STATE_DEFAULT; 
      }
    }
  }
  else 
  {
     USBD_CtlError(pdev , req);                        
  } 
}

/**
* @brief  USBD_SetConfig
*         Handle Set device configuration request
* @param  pdev: device instance
* @param  req: usb request
* @retval status
*/
static void USBD_SetConfig(USBD_HandleTypeDef *pdev , 
                           USBD_SetupReqTypedef *req)
{
  
  static uint8_t  cfgidx;
  
  cfgidx = (uint8_t)(req->wValue);                 
  
  if (cfgidx > USBD_MAX_NUM_CONFIGURATION ) 
  {            
     USBD_CtlError(pdev , req);                              
  } 
  else 
  {
    switch (pdev->dev_state) 
    {
    case USBD_STATE_ADDRESSED:
      if (cfgidx) 
      {                                			   							   							   				
        pdev->dev_config = cfgidx;
        pdev->dev_state = USBD_STATE_CONFIGURED;
        if(USBD_SetClassConfig(pdev , cfgidx) == USBD_FAIL)
        {
          USBD_CtlError(pdev , req);  
          return;
        }
        USBD_CtlSendStatus(pdev);
      }
      else 
      {
         USBD_CtlSendStatus(pdev);
      }
      break;
      
    case USBD_STATE_CONFIGURED:
      if (cfgidx == 0) 
      {                           
        pdev->dev_state = USBD_STATE_ADDRESSED;
        pdev->dev_config = cfgidx;          
        USBD_ClrClassConfig(pdev , cfgidx);
        USBD_CtlSendStatus(pdev);
        
      } 
      else  if (cfgidx != pdev->dev_config) 
      {
        /* Clear old configuration */
        USBD_ClrClassConfig(pdev , pdev->dev_config);
        
        /* set new configuration */
        pdev->dev_config = cfgidx;
        if(USBD_SetClassConfig(pdev , cfgidx) == USBD_FAIL)
        {
          USBD_CtlError(pdev , req);  
          return;
        }
        USBD_CtlSendStatus(pdev);
      }
      else
      {
        USBD_CtlSendStatus(pdev);
      }
      break;
      
    default:					
       USBD_CtlError(pdev , req);                     
      break;
    }
  }
}

/**
* @brief  USBD_GetConfig
*         Handle Get device configuration request
* @param  pdev: device instance
* @param  req: usb request
* @retval status
*/
static void USBD_GetConfig(USBD_HandleTypeDef *pdev , 
                           USBD_SetupReqTypedef *req)
{

  if (req->wLength != 1) 
  {                   
     USBD_CtlError(pdev , req);
  }
  else 
  {
    switch (pdev->dev_state )  
    {
    case USBD_STATE_ADDRESSED:                     
      pdev->dev_default_config = 0;
      USBD_CtlSendData (pdev, 
                        (uint8_t *)&pdev->dev_default_config,
                        1);
      break;
      
    case USBD_STATE_CONFIGURED:   
      
      USBD_CtlSendData (pdev, 
                        (uint8_t *)&pdev->dev_config,
                        1);
      break;
      
    default:
       USBD_CtlError(pdev , req);
      break;
    }
  }
}

/**
* @brief  USBD_GetStatus
*         Handle Get Status request
* @param  pdev: device instance
* @param  req: usb request
* @retval status
*/
static void USBD_GetStatus(USBD_HandleTypeDef *pdev , 
                           USBD_SetupReqTypedef *req)
{
  
    
  switch (pdev->dev_state) 
  {
  case USBD_STATE_ADDRESSED:
  case USBD_STATE_CONFIGURED:
    
#if ( USBD_SELF_POWERED == 1)
    pdev->dev_config_status = USB_CONFIG_SELF_POWERED;                                  
#else
    pdev->dev_config_status = 0;                                   
#endif
                      
    if (pdev->dev_remote_wakeup) 
    {
       pdev->dev_config_status |= USB_CONFIG_REMOTE_WAKEUP;                                
    }
    
    USBD_CtlSendData (pdev, 
                      (uint8_t *)& pdev->dev_config_status,
                      2);
    break;
    
  default :
    USBD_CtlError(pdev , req);                        
    break;
  }
}


/**
* @brief  USBD_SetFeature
*         Handle Set device feature request
* @param  pdev: device instance
* @param  req: usb request
* @retval status
*/
static void USBD_SetFeature(USBD_HandleTypeDef *pdev , 
                            USBD_SetupReqTypedef *req)
{

  if (req->wValue == USB_FEATURE_REMOTE_WAKEUP)
  {
    pdev->dev_remote_wakeup = 1;  
    pdev->pClass->Setup (pdev, req);   
    USBD_CtlSendStatus(pdev);
  }

}


/**
* @brief  USBD_ClrFeature
*         Handle clear device feature request
* @param  pdev: device instance
* @param  req: usb request
* @retval status
*/
static void USBD_ClrFeature(USBD_HandleTypeDef *pdev , 
                            USBD_SetupReqTypedef *req)
{
  switch (pdev->dev_state)
  {
  case USBD_STATE_ADDRESSED:
  case USBD_STATE_CONFIGURED:
    if (req->wValue == USB_FEATURE_REMOTE_WAKEUP) 
    {
      pdev->dev_remote_wakeup = 0; 
      pdev->pClass->Setup (pdev, req);   
      USBD_CtlSendStatus(pdev);
    }
    break;
    
  default :
     USBD_CtlError(pdev , req);
    break;
  }
}

/**
* @brief  USBD_ParseSetupRequest 
*         Copy buffer into setup structure
* @param  pdev: device instance
* @param  req: usb request
* @retval None
*/

void USBD_ParseSetupRequest(USBD_SetupReqTypedef *req, uint8_t *pdata)
{
  req->bmRequest     = *(uint8_t *)  (pdata);
  req->bRequest      = *(uint8_t *)  (pdata +  1);
  req->wValue        = SWAPBYTE      (pdata +  2);
  req->wIndex        = SWAPBYTE      (pdata +  4);
  req->wLength       = SWAPBYTE      (pdata +  6);

}

/**
* @brief  USBD_CtlError 
*         Handle USB low level Error
* @param  pdev: device instance
* @param  req: usb request
* @retval None
*/

void USBD_CtlError( USBD_HandleTypeDef *pdev ,
                            USBD_SetupReqTypedef *req)
{
  USBD_LL_StallEP(pdev , 0x80);
  USBD_LL_StallEP(pdev , 0);
}

void USBD_CtlError_examined( USBD_HandleTypeDef *pdev ,
                            USBD_SetupReqTypedef *req)
{
  USBD_LL_StallEP(pdev , 0x80);
  USBD_LL_StallEP(pdev , 0);
}


/**
  * @brief  USBD_GetString
  *         Convert Ascii string into unicode one
  * @param  desc : descriptor buffer
  * @param  unicode : Formatted string buffer (unicode)
  * @param  len : descriptor length
  * @retval None
  */
void USBD_GetString(uint8_t *desc, uint8_t *unicode, uint16_t *len)
{
  uint8_t idx = 0;
  
  if (desc != NULL) 
  {
    *len =  USBD_GetLen(desc) * 2 + 2;    
    unicode[idx++] = *len;
    unicode[idx++] =  USB_DESC_TYPE_STRING;
    
    while (*desc != '\0') 
    {
      unicode[idx++] = *desc++;
      unicode[idx++] =  0x00;
    }
  } 
}

/**
  * @brief  USBD_GetLen
  *         return the string length
   * @param  buf : pointer to the ascii string buffer
  * @retval string length
  */
static uint8_t USBD_GetLen(uint8_t *buf)
{
    uint8_t  len = 0;

    while (*buf != '\0') 
    {
        len++;
        buf++;
    }

    return len;
}
/**
  * @}
  */ 


/**
  * @}
  */ 


/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
