
#ifndef USBH_ACCESSORY_H_
#define USBH_ACCESSORY_H_

#include "hal_usbh.h"

#if HAL_USE_USBH && HAL_USBH_USE_ACCESSORY

#define USBH_ACCESSORY_VID        0x18d1
#define USBH_ACCESSORY_PID        0x2d00
#define USBH_ACCESSORY_ADB_PID    0x2d01

//! Таймер для отсчета таймаута переключения в режим Accessory
extern virtual_timer_t vt_aoa_switch_tout;
//! Источник событий таймаута переключения в Accessory режим
extern event_source_t aoa_switch_tout_evt_src;

typedef enum {
    USBH_AOA_STATE_NOINIT = 0,
    USBH_AOA_STATE_INIT,
    USBH_AOA_STATE_SW_ACCESS,
    USBH_AOA_STATE_CONNECT
  } HSBH_AccessStates_t;

typedef enum {
    USBH_AOACHAN_STATE_NOINIT = 0,    /*! Object not initialized */
    USBH_AOACHAN_STATE_INIT,          /*! Object initialized. Wait driver load */
    USBH_AOACHAN_STATE_STOP,          /*! Channel stopped */
    USBH_AOACHAN_STATE_ACTIVE,        /*! Driver loaded. Endpoint configured. Ready to start channel */
    USBH_AOACHAN_STATE_READY          /*! Channel started and ready for data transfer */
  } HSBH_AccessChannelStates_t;

#define _accessory_channel_driver_methods                                          \
  _base_asynchronous_channel_methods

struct AccessoryChannelDriverVMT {
	_accessory_channel_driver_methods
};

typedef struct USBH_AccessoryDrv       USBH_AccessoryDrv_t;
typedef struct AccessoryChannelDriver  USBH_AccessoryChannelDriver_t;

struct USBH_AccessoryDrv {
	/* inherited from abstract class driver */
	_usbh_base_classdriver_data
  
  USBH_AccessoryChannelDriver_t *AccessChannel;

  HSBH_AccessStates_t state;
 
  const char *manufacturer;
  const char *model;
  const char *description;
  const char *version;
  const char *uri;
  const char *serial;
  mutex_t mtx;
};

#ifndef HAL_USBHACCESS_CHAN_IQ_SZ
#define HAL_USBHACCESS_CHAN_IQ_SZ      1024
#endif 

#ifndef HAL_USBHACCESS_CHAN_OQ_SZ
#define HAL_USBHACCESS_CHAN_OQ_SZ      1024
#endif

struct AccessoryChannelDriver {
  /* inherited from abstract asyncrhonous channel driver */
  const struct AccessoryChannelDriverVMT *vmt;
  _base_asynchronous_channel_data

  USBH_AccessoryDrv_t *pAccess;

  usbh_ep_t epin;
  usbh_urb_t iq_urb;
  threads_queue_t	iq_waiting;
  uint32_t iq_counter;
  USBH_DEFINE_BUFFER(uint8_t, iq_buff[HAL_USBHACCESS_CHAN_IQ_SZ]);
  uint8_t *iq_ptr;

  usbh_ep_t epout;
  usbh_urb_t oq_urb;
  threads_queue_t	oq_waiting;
  uint32_t oq_counter;
  USBH_DEFINE_BUFFER(uint8_t, oq_buff[HAL_USBHACCESS_CHAN_OQ_SZ]);
  uint8_t *oq_ptr;

  virtual_timer_t vt;
  HSBH_AccessChannelStates_t state;
};

extern USBH_AccessoryChannelDriver_t USBH_AccessChan;
extern USBH_AccessoryDrv_t     USBH_AccessDrv;

extern void usbhAccessChannel_start(USBH_AccessoryChannelDriver_t *pAccessChan);
extern void usbhAccessChannelObjectInit(USBH_AccessoryChannelDriver_t *pAccessChan, USBH_AccessoryDrv_t *pAccess);
extern void usbhAccessObjectInit(USBH_AccessoryDrv_t *pAccess);

#endif /*HAL_USE_USBH && HAL_USBH_USE_ACCESSORY*/

#endif
