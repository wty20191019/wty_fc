#include "usbd_usr.h"
#include "usbd_ioreq.h"
#include "usb_dcd_int.h"

extern USB_OTG_CORE_HANDLE  USB_OTG_dev;

USBD_Usr_cb_TypeDef USR_cb ={  USBD_USR_Init,  USBD_USR_DeviceReset,  USBD_USR_DeviceConfigured,
  USBD_USR_DeviceSuspended,  USBD_USR_DeviceResumed,  USBD_USR_DeviceConnected,  USBD_USR_DeviceDisconnected, };

void OTG_FS_IRQHandler(void)
{
	USBD_OTG_ISR_Handler(&USB_OTG_dev);
}

void USBD_USR_Init(void){  }
void USBD_USR_DeviceReset(uint8_t speed ){}
void USBD_USR_DeviceConfigured (void){}
void USBD_USR_DeviceSuspended(void){}
void USBD_USR_DeviceResumed(void){}
void USBD_USR_DeviceConnected (void){}
void USBD_USR_DeviceDisconnected (void){}







