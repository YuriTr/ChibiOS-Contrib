

#include "hal.h"
#include "hal_usbh.h"

#if HAL_USBH_USE_ACCESSORY

#if !HAL_USE_USBH
# error "USBH_ACCESSORY needs USBH"
#endif

#include <string.h>
#include "usbh/dev/accessory.h"
#include "usbh/internal.h"

#if USBHACCESS_DEBUG_ENABLE_TRACE
#define udbgf(f, ...)  usbDbgPrintf(f, ##__VA_ARGS__)
#define udbg(f, ...)  usbDbgPuts(f, ##__VA_ARGS__)
#else
#define udbgf(f, ...)  do {} while(0)
#define udbg(f, ...)   do {} while(0)
#endif

#if USBHACCESS_DEBUG_ENABLE_INFO
#define uinfof(f, ...)  usbDbgPrintf(f, ##__VA_ARGS__)
#define uinfo(f, ...)  usbDbgPuts(f, ##__VA_ARGS__)
#else
#define uinfof(f, ...)  do {} while(0)
#define uinfo(f, ...)   do {} while(0)
#endif

#if USBHACCESS_DEBUG_ENABLE_WARNINGS
#define uwarnf(f, ...)  usbDbgPrintf(f, ##__VA_ARGS__)
#define uwarn(f, ...)  usbDbgPuts(f, ##__VA_ARGS__)
#else
#define uwarnf(f, ...)  do {} while(0)
#define uwarn(f, ...)   do {} while(0)
#endif

#if USBHACCESS_DEBUG_ENABLE_ERRORS
#define uerrf(f, ...)  usbDbgPrintf(f, ##__VA_ARGS__)
#define uerr(f, ...)  usbDbgPuts(f, ##__VA_ARGS__)
#else
#define uerrf(f, ...)  do {} while(0)
#define uerr(f, ...)   do {} while(0)
#endif

//! Таймер для отсчета таймаута переключения в режим Accessory
virtual_timer_t vt_aoa_switch_tout;
//! Источник событий таймаута переключения в Accessory режим
EVENTSOURCE_DECL(aoa_switch_tout_evt_src);

#define ACCESSORY_CTRL_GET_PROTOCOL   0x33
#define ACCESSORY_CTRL_SEND_STRINGS   0x34
#define ACCESSORY_CTRL_START          0x35

#define ACCESSORY_STRING_INDEX_MANUF    0
#define ACCESSORY_STRING_INDEX_MODEL    1
#define ACCESSORY_STRING_INDEX_DESCR    2
#define ACCESSORY_STRING_INDEX_VERSION  3
#define ACCESSORY_STRING_INDEX_URL      4
#define ACCESSORY_STRING_INDEX_SERIAL   5

USBH_AccessoryChannelDriver_t USBH_AccessChan;

/*===========================================================================*/
/* USB Class driver loader for ACCESSORY								 		 	               */
/*===========================================================================*/
USBH_AccessoryDrv_t     USBH_AccessDrv;

static usbh_baseclassdriver_t *_access_load(usbh_device_t *dev, const uint8_t *descriptor, uint16_t rem);
static void _access_unload(usbh_baseclassdriver_t *drv);

static const usbh_classdriver_vmt_t access_class_driver_vmt = {
	_access_load,
	_access_unload
};

const usbh_classdriverinfo_t usbhAccessClassDriverInfo = {
	0xff, 0xff, 0xff, "Accessory", &access_class_driver_vmt
};

/**
 * Call back функция для таймера 
 * Генерирует событие для источника, указатель на который передан в качестве параметра. 
 *  
 * @author Yuri (27.06.2016)
 * 
 * @param p указатель на event_source_t 
 */
static void vtcbSetEvent(void *p) {
  if (p) {
    event_source_t * evt_src = (event_source_t *)p;
    chSysLockFromISR();
    chEvtBroadcastI(evt_src);
    chSysUnlockFromISR();
  }
}

/********** Accessory Driver Methods *********/
static usbh_urbstatus_t usbhAccessSendString(usbh_device_t *dev,
		uint16_t wIndex,
		const char *buff) {
    return usbhControlRequest(dev,
                       USBH_REQTYPE_VENDOR | USBH_REQTYPE_OUT,
                       ACCESSORY_CTRL_SEND_STRINGS,
                       0,
                       wIndex,
                       strlen(buff) + 1,
                       (uint8_t *)buff);
}

static usbh_urbstatus_t usbhAccessControlRequest(usbh_device_t *dev,
		uint8_t bmRequestType,
		uint8_t bRequest,
		uint16_t wValue,
		uint16_t wIndex,
		uint16_t wLength,
		uint8_t *buff, 
    uint32_t *actual_len) {

	const USBH_DEFINE_BUFFER(usbh_control_request_t, req) = {
			bmRequestType,
			bRequest,
			wValue,
			wIndex,
			wLength
	};
	return usbhControlRequestExtended(dev, &req, buff, actual_len, MS2ST(1000));
}

static usbh_baseclassdriver_t *_access_load(usbh_device_t *dev, const uint8_t *descriptor, uint16_t rem)  {
  osalDbgCheck(dev != NULL);
  USBH_AccessoryDrv_t *pAccess = &USBH_AccessDrv;
  uinfof("AOA:VID=%x PID=%x",dev->devDesc.idVendor,dev->devDesc.idProduct);
  
  if ( dev->devDesc.idVendor == USBH_ACCESSORY_VID && 
     ( dev->devDesc.idProduct == USBH_ACCESSORY_PID ||
       dev->devDesc.idProduct == USBH_ACCESSORY_ADB_PID ) ) {

    if (pAccess->state != USBH_AOA_STATE_INIT ) {
      return NULL;
    }

    if ((rem < descriptor[0]) || (descriptor[1] != USBH_DT_INTERFACE)) {
      uwarnf("AOA:Wrong Descriptor.");
      return NULL;
    }
    chVTReset(&vt_aoa_switch_tout);
    uinfo("AOA:Accessory Device found");
    const usbh_interface_descriptor_t * const ifdesc = (const usbh_interface_descriptor_t * const)descriptor;
    if (ifdesc->bInterfaceNumber != 0) {
      uwarn("AOA: Will allocate driver along with IF #0");
    }

    if (USBH_AccessDrv.dev != NULL) {
      uwarn("AOA: Can't alloc driver");
      return NULL;
    } 
    
    usbhEPSetName(&dev->ctrl, "AOA[CTRL]");

    osalMutexLock(&pAccess->mtx);

    /* parse the configuration descriptor */
    generic_iterator_t iep, icfg;
    if_iterator_t iif;
    cfg_iter_init(&icfg, dev->fullConfigurationDescriptor, dev->basicConfigDesc.wTotalLength);
    for (if_iter_init(&iif, &icfg); iif.valid; if_iter_next(&iif)) {
      const usbh_interface_descriptor_t *const ifdesc = if_get(&iif);

      uinfof("AOA: Interface #%d", ifdesc->bInterfaceNumber);
      if (ifdesc->bInterfaceNumber != 0) {
        uwarn("AOA: Only one interface allowed");
        uinfof("AOA: Interface found:%d",ifdesc->bInterfaceNumber);
      } else {
        USBH_AccessoryChannelDriver_t *pacc = &USBH_AccessChan;
        //pacc->ifnum = ifdesc->bInterfaceNumber;
        pacc->epin.status = USBH_EPSTATUS_UNINITIALIZED;
        pacc->epout.status = USBH_EPSTATUS_UNINITIALIZED;
        
        for (ep_iter_init(&iep, &iif); iep.valid; ep_iter_next(&iep)) {
          const usbh_endpoint_descriptor_t *const epdesc = ep_get(&iep);
          if ((epdesc->bEndpointAddress & 0x80) && (epdesc->bmAttributes == USBH_EPTYPE_BULK)) {
            uinfof("BULK IN endpoint found: bEndpointAddress=%02x", epdesc->bEndpointAddress);
            usbhEPObjectInit(&pacc->epin, dev, epdesc);
            usbhEPSetName(&pacc->epin, "AOA[IN ]");
          } else if (((epdesc->bEndpointAddress & 0x80) == 0)
              && (epdesc->bmAttributes == USBH_EPTYPE_BULK)) {
            uinfof("BULK OUT endpoint found: bEndpointAddress=%02x", epdesc->bEndpointAddress);
            usbhEPObjectInit(&pacc->epout, dev, epdesc);
            usbhEPSetName(&pacc->epout, "AOA[OUT]");
          } else {
            uinfof("unsupported endpoint found: bEndpointAddress=%02x, bmAttributes=%02x",
                epdesc->bEndpointAddress, epdesc->bmAttributes);
          }
        }
                
        if ((pacc->epin.status != USBH_EPSTATUS_CLOSED)
            || (pacc->epout.status != USBH_EPSTATUS_CLOSED)) {
          uwarn("\tCouldn't find endpoints; can't alloc channel for AOA");
          continue;
        } else {
          pacc->state = USBH_AOACHAN_STATE_ACTIVE;
        }

        pAccess->state = USBH_AOA_STATE_CONNECT;
        //pacc->state = USBHAOA_STATE_ACTIVE;
      }
    }

    osalMutexUnlock(&pAccess->mtx);

  } else {
    usbhEPSetName(&dev->ctrl, "[CTRL]");
    uinfo("AOA:VID/PID are not Accessory Mode");
    switch (pAccess->state) {
    case USBH_AOA_STATE_NOINIT:
      usbhAccessObjectInit(pAccess);
    case USBH_AOA_STATE_INIT: { 
      USBH_DEFINE_BUFFER(uint16_t, ver) = 0;
      USBH_DEFINE_BUFFER(uint32_t, len) = 0;
      uinfo("AOA:Try switch to AOA");
      uinfo("AOA:Get Protocol Version");
      // read protocol version
      usbhAccessControlRequest(dev, 
                               USBH_REQTYPE_VENDOR | USBH_REQTYPE_IN,
                               ACCESSORY_CTRL_GET_PROTOCOL,
                               0,
                               0,
                               2,
                               (uint8_t *) &ver,
                               &len
                               );
      if (len == 2 &&
          (ver == 1 || ver == 2) ) {

        uinfof("AOA:Protocol Version: %d",ver);
        uinfo("AOA:Send Manuf. String");
        usbhAccessSendString(dev,
                           ACCESSORY_STRING_INDEX_MANUF,
                           pAccess->manufacturer);

        uinfo("AOA:Send Model String");
        usbhAccessSendString(dev,
                           ACCESSORY_STRING_INDEX_MODEL,
                           pAccess->manufacturer);
        uinfo("AOA:Send Description String");
        usbhAccessSendString(dev,
                           ACCESSORY_STRING_INDEX_DESCR,
                           pAccess->manufacturer);
        uinfo("AOA:Send Version String");
        usbhAccessSendString(dev,
                           ACCESSORY_STRING_INDEX_VERSION,
                           pAccess->manufacturer);
        uinfo("AOA:Send URL String");
        usbhAccessSendString(dev,
                           ACCESSORY_STRING_INDEX_URL,
                           pAccess->manufacturer);
        uinfo("AOA:Send Serial String");
        usbhAccessSendString(dev,
                           ACCESSORY_STRING_INDEX_SERIAL,
                           pAccess->manufacturer);
        uinfo("AOA:Switch to Accessory Mode");
        usbhControlRequest(dev,
                           USBH_REQTYPE_VENDOR | USBH_REQTYPE_OUT,
                           ACCESSORY_CTRL_START,
                           0, 0, 0, 0);
        pAccess->state = USBH_AOA_STATE_SW_ACCESS;
        chVTSet(&vt_aoa_switch_tout,S2ST(3),vtcbSetEvent,&aoa_switch_tout_evt_src);
        //uinfo("AOA:Start timer");
      } else {
        uwarnf("AOA:Wrong Protocol Version: ver:%d, len:%d",ver ,len);
        pAccess = NULL;
      }
      break;
    }
    default:
      pAccess = NULL;
    }

  }
  
  return (usbh_baseclassdriver_t *) pAccess;
}

static void _stop(USBH_AccessoryChannelDriver_t *pAccessChan);
static void _access_unload(usbh_baseclassdriver_t *drv)  {
  osalDbgCheck(drv != NULL);
  USBH_AccessoryDrv_t *const pAccess = (USBH_AccessoryDrv_t *)drv;
  USBH_AccessoryChannelDriver_t *const pAccessChan = pAccess->AccessChannel;

  osalMutexLock(&pAccess->mtx);

  if (pAccess->state == USBH_AOA_STATE_CONNECT) {
    //if not wait for switch to Accessory Mode, then 
    if (pAccessChan) {
      _stop(pAccessChan);
    }
    osalSysLock();
    usbhAccessChannelObjectInit(pAccessChan, pAccess);
    osalSysUnlock();

  }
  pAccess->state = USBH_AOA_STATE_INIT;

  osalMutexUnlock(&pAccess->mtx);
}

static void _submitOutI(USBH_AccessoryChannelDriver_t *pacc, uint32_t len) {
	udbgf("AOA: Submit OUT %d", len);
	pacc->oq_urb.requestedLength = len;
	usbhURBObjectResetI(&pacc->oq_urb);
	usbhURBSubmitI(&pacc->oq_urb);
}

static void _out_cb(usbh_urb_t *urb) {
	USBH_AccessoryChannelDriver_t *const pacc = (USBH_AccessoryChannelDriver_t *)urb->userData;
	switch (urb->status) {
	case USBH_URBSTATUS_OK:
		pacc->oq_ptr = pacc->oq_buff;
		pacc->oq_counter = HAL_USBHACCESS_CHAN_OQ_SZ;
		chThdDequeueNextI(&pacc->oq_waiting, Q_OK);
		return;
	case USBH_URBSTATUS_DISCONNECTED:
		uwarn("AOA: URB OUT disconnected");
		chThdDequeueNextI(&pacc->oq_waiting, Q_RESET);
		return;
	default:
		uerrf("AOA: URB OUT status unexpected = %d", urb->status);
		break;
	}
	usbhURBObjectResetI(&pacc->oq_urb);
	usbhURBSubmitI(&pacc->oq_urb);
}

/********** Accessory Asyncchannel Methods *********/

static size_t _write_timeout(USBH_AccessoryChannelDriver_t *pacc, const uint8_t *bp,
		size_t n, systime_t timeout) {
  chDbgCheck(n > 0U);

  size_t w = 0;
  chSysLock();

  while (true) {
    if (pacc->state != USBH_AOACHAN_STATE_READY) {
      chSysUnlock();
      return w;
    }
    while (usbhURBIsBusy(&pacc->oq_urb)) {
      if (chThdEnqueueTimeoutS(&pacc->oq_waiting, timeout) != Q_OK) {
        chSysUnlock();
        return w;
      }
    }

    *pacc->oq_ptr++ = *bp++;
    if (--pacc->oq_counter == 0) {
      _submitOutI(pacc, HAL_USBHACCESS_CHAN_OQ_SZ);
      chSchRescheduleS();
    }
    chSysUnlock(); /* Gives a preemption chance in a controlled point.*/

    w++;
    if (--n == 0U)
      return w;

    chSysLock();
  }
}

static msg_t _put_timeout(USBH_AccessoryChannelDriver_t *pacc, uint8_t b, systime_t timeout) {
  chSysLock();
  if (pacc->state != USBH_AOACHAN_STATE_READY) {
    chSysUnlock();
    return Q_RESET;
  }

  while (usbhURBIsBusy(&pacc->oq_urb)) {
    msg_t msg = chThdEnqueueTimeoutS(&pacc->oq_waiting, timeout);
    if (msg < Q_OK) {
      chSysUnlock();
      return msg;
    }
  }

  *pacc->oq_ptr++ = b;
  if (--pacc->oq_counter == 0) {
    _submitOutI(pacc, HAL_USBHACCESS_CHAN_OQ_SZ);
    chSchRescheduleS();
  }
  chSysUnlock();
  return Q_OK;
}

static size_t _write(USBH_AccessoryChannelDriver_t *Channel, const uint8_t *bp, size_t n) {
	return _write_timeout(Channel, bp, n, TIME_INFINITE);
}

static msg_t _put(USBH_AccessoryChannelDriver_t *Channel, uint8_t b) {
	return _put_timeout(Channel, b, TIME_INFINITE);
}

static void _submitInI(USBH_AccessoryChannelDriver_t *pacc) {
	udbg("AOA: Submit IN");
	usbhURBObjectResetI(&pacc->iq_urb);
	usbhURBSubmitI(&pacc->iq_urb);
}

static void _in_cb(usbh_urb_t *urb) {
	USBH_AccessoryChannelDriver_t *const pacc = (USBH_AccessoryChannelDriver_t *)urb->userData;
	switch (urb->status) {
	case USBH_URBSTATUS_OK:
		if (urb->actualLength < 1) {
			uwarnf("AOA: URB IN actualLength = %d, < 1", urb->actualLength);
		} else {
			udbgf("AOA: URB IN data len=%d",
					urb->actualLength);
			pacc->iq_ptr = pacc->iq_buff;
			pacc->iq_counter = urb->actualLength;
			chThdDequeueNextI(&pacc->iq_waiting, Q_OK);
			return;
		}
		break;
	case USBH_URBSTATUS_DISCONNECTED:
		uwarn("AOA: URB IN disconnected");
		chThdDequeueNextI(&pacc->iq_waiting, Q_RESET);
		return;
	default:
		uerrf("AOA: URB IN status unexpected = %d", urb->status);
		break;
	}
	_submitInI(pacc);
}

static size_t _read_timeout(USBH_AccessoryChannelDriver_t *pacc, uint8_t *bp,
                                                size_t n, systime_t timeout) {
  size_t r = 0;

  chDbgCheck(n > 0U);

  chSysLock();
  while (true) {
    if (pacc->state != USBH_AOACHAN_STATE_READY) {
      chSysUnlock();
      return r;
    }
    while (pacc->iq_counter == 0) {
      if (!usbhURBIsBusy(&pacc->iq_urb))
        _submitInI(pacc);
      if (chThdEnqueueTimeoutS(&pacc->iq_waiting, timeout) != Q_OK) {
        chSysUnlock();
        return r;
      }
    }
    *bp++ = *pacc->iq_ptr++;
    if (--pacc->iq_counter == 0) {
      _submitInI(pacc);
      chSchRescheduleS();
    }
    chSysUnlock();

    r++;
    if (--n == 0U)
      return r;

    chSysLock();
  }
}

static msg_t _get_timeout(USBH_AccessoryChannelDriver_t *pacc, systime_t timeout) {
  uint8_t b;

  chSysLock();
  if (pacc->state != USBH_AOACHAN_STATE_READY) {
    chSysUnlock();
    return Q_RESET;
  }
  while (pacc->iq_counter == 0) {
    if (!usbhURBIsBusy(&pacc->iq_urb))
      _submitInI(pacc);
    msg_t msg = chThdEnqueueTimeoutS(&pacc->iq_waiting, timeout);
    if (msg < Q_OK) {
      chSysUnlock();
      return msg;
    }
  }
  b = *pacc->iq_ptr++;
  if (--pacc->iq_counter == 0) {
    _submitInI(pacc);
    chSchRescheduleS();
  }
  chSysUnlock();

  return (msg_t)b;
}

static msg_t _get(USBH_AccessoryChannelDriver_t *Channel) {
	return _get_timeout(Channel, TIME_INFINITE);
}

static size_t _read(USBH_AccessoryChannelDriver_t *Channel, uint8_t *bp, size_t n) {
	return _read_timeout(Channel, bp, n, TIME_INFINITE);
}

static void _vt(void *p) {
	USBH_AccessoryChannelDriver_t *const pacc = (USBH_AccessoryChannelDriver_t *)p;
	chSysLockFromISR();
	uint32_t len = pacc->oq_ptr - pacc->oq_buff;
	if (len && !usbhURBIsBusy(&pacc->oq_urb)) {
		_submitOutI(pacc, len);
	}
	if ((pacc->iq_counter == 0) && !usbhURBIsBusy(&pacc->iq_urb)) {
		_submitInI(pacc);
	}
	chVTSetI(&pacc->vt, MS2ST(16), _vt, pacc);
	chSysUnlockFromISR();
}

void usbhAccessChannel_start(USBH_AccessoryChannelDriver_t *pAccessChan) {
  
  osalDbgCheck((pAccessChan->state == USBH_AOACHAN_STATE_ACTIVE)
      || (pAccessChan->state == USBH_AOACHAN_STATE_READY));
  
  if (pAccessChan->state == USBH_AOACHAN_STATE_READY) {
    return;
  }

  uinfo("AOA:Channel start");

  osalMutexLock(&pAccessChan->pAccess->mtx);

  usbhURBObjectInit(&pAccessChan->oq_urb, &pAccessChan->epout, _out_cb, pAccessChan, pAccessChan->oq_buff, 0);
  chThdQueueObjectInit(&pAccessChan->oq_waiting);
  pAccessChan->oq_counter = HAL_USBHACCESS_CHAN_OQ_SZ;
  pAccessChan->oq_ptr = pAccessChan->oq_buff;
  usbhEPOpen(&pAccessChan->epout);
  
  usbhURBObjectInit(&pAccessChan->iq_urb, &pAccessChan->epin, _in_cb, pAccessChan, pAccessChan->iq_buff, HAL_USBHACCESS_CHAN_IQ_SZ);
  chThdQueueObjectInit(&pAccessChan->iq_waiting);
  pAccessChan->iq_counter = 0;
  pAccessChan->iq_ptr = pAccessChan->iq_buff;
  usbhEPOpen(&pAccessChan->epin);

  chVTSet(&pAccessChan->vt, MS2ST(16), _vt, pAccessChan);

  pAccessChan->state = USBH_AOACHAN_STATE_READY;

  osalMutexUnlock(&pAccessChan->pAccess->mtx);
}

static void _stop(USBH_AccessoryChannelDriver_t *pAccessChan)  {
  //@todo there must be closing all endpoints
  osalSysLock();
  chVTResetI(&pAccessChan->vt);
  usbhEPCloseS(&pAccessChan->epin);
  usbhEPCloseS(&pAccessChan->epout);
  if (pAccessChan->state == USBH_AOACHAN_STATE_READY) {
    chThdDequeueAllI(&pAccessChan->iq_waiting, Q_RESET);
    chThdDequeueAllI(&pAccessChan->oq_waiting, Q_RESET);
  }
  osalOsRescheduleS();
  pAccessChan->state = USBH_AOACHAN_STATE_STOP;
  //pAccessChan->state = USBHFTDIP_STATE_ACTIVE;
  osalSysUnlock();
}

static const struct AccessoryChannelDriverVMT async_channel_vmt = {
	(size_t (*)(void *, const uint8_t *, size_t))_write,
	(size_t (*)(void *, uint8_t *, size_t))_read,
	(msg_t (*)(void *, uint8_t))_put,
	(msg_t (*)(void *))_get,
	(msg_t (*)(void *, uint8_t, systime_t))_put_timeout,
	(msg_t (*)(void *, systime_t))_get_timeout,
	(size_t (*)(void *, const uint8_t *, size_t, systime_t))_write_timeout,
	(size_t (*)(void *, uint8_t *, size_t, systime_t))_read_timeout
};

void usbhAccessChannelObjectInit(USBH_AccessoryChannelDriver_t *pAccessChan, USBH_AccessoryDrv_t *pAccess)  {
  osalDbgCheck(pAccessChan != NULL);
  memset(pAccessChan, 0, sizeof(*pAccessChan));
  pAccessChan->state = USBH_AOACHAN_STATE_INIT;
  pAccessChan->vmt = &async_channel_vmt;
  pAccessChan->pAccess = pAccess;
}

static USBH_DEFINE_BUFFER(const char, manufacturer[]) = HAL_USBHACCESS_STRING_MANUF;
static USBH_DEFINE_BUFFER(const char, model[]) = HAL_USBHACCESS_STRING_MODEL;
static USBH_DEFINE_BUFFER(const char, description[]) = HAL_USBHACCESS_STRING_LONGNAME;
static USBH_DEFINE_BUFFER(const char, version[]) = HAL_USBHACCESS_STRING_VERSION;
static USBH_DEFINE_BUFFER(const char, uri[]) = HAL_USBHACCESS_STRING_URL;
static USBH_DEFINE_BUFFER(const char, serial[]) = HAL_USBHACCESS_STRING_SERIAL;


void usbhAccessObjectInit(USBH_AccessoryDrv_t *pAccess) {
	osalDbgCheck(pAccess != NULL);
	memset(pAccess, 0, sizeof(*pAccess));
	pAccess->info = &usbhAccessClassDriverInfo;
  pAccess->manufacturer = manufacturer;
  pAccess->model = model;
  pAccess->description = description;
  pAccess->version = version;
  pAccess->uri = uri;
  pAccess->serial = serial;
  usbhAccessChannelObjectInit(&USBH_AccessChan, pAccess);
  pAccess->AccessChannel = &USBH_AccessChan;
  pAccess->state = USBH_AOA_STATE_INIT;
	osalMutexObjectInit(&pAccess->mtx);
}

#endif
