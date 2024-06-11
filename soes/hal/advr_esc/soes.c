#include <soes/esc.h>
#include <soes/esc_coe.h>
#include <soes/esc_foe.h>
#include <soes/hal/advr_esc/soes.h>

#include <ecat_options.h>
#include <soes_hook.h>

#define IS_RXPDO(index) ((index) >= 0x1600 && (index) < 0x1800)
#define IS_TXPDO(index) ((index) >= 0x1A00 && (index) < 0x1C00)

_ESCvar  	ESCvar;
uint8_t     MBX[MBXBUFFERS * MAX(MBXSIZE,MBXSIZEBOOT)];
_MBXcontrol MBXcontrol[MBXBUFFERS];
_SMmap      SMmap2[MAX_MAPPINGS_SM2];
_SMmap      SMmap3[MAX_MAPPINGS_SM3];


#if MAX_MAPPINGS_SM2 > 0
static uint8_t rxpdo[MAX_RXPDO_SIZE] __attribute__((aligned (8)));
#else
extern uint8_t * rxpdo;
//extern rx_pdo_t  rx_pdo;
#endif

#if MAX_MAPPINGS_SM3 > 0
static uint8_t txpdo[MAX_TXPDO_SIZE] __attribute__((aligned (8)));
#else
extern uint8_t * txpdo;
//extern tx_pdo_t  tx_pdo;
#endif

/** Function to pre-qualify the incoming SDO download.
 *
 * @param[in] index      = index of SDO download request to check
 * @param[in] sub-index  = sub-index of SDO download request to check
 * @return SDO abort code, or 0 on success
 */
uint32_t ESC_download_pre_objecthandler (uint16_t index,
      uint8_t subindex,
      void * data,
      size_t size,
      uint16_t flags)
{
   if (IS_RXPDO (index) ||
       IS_TXPDO (index) ||
       index == RX_PDO_OBJIDX ||
       index == TX_PDO_OBJIDX)
   {
      uint8_t minSub = ((flags & COMPLETE_ACCESS_FLAG) == 0) ? 0 : 1;
      if (subindex > minSub && COE_maxSub (index) != 0)
      {
         return ABORT_SUBINDEX0_NOT_ZERO;
      }
   }

   if (ESCvar.pre_object_download_hook)
   {
      return (ESCvar.pre_object_download_hook) (index,
            subindex,
            data,
            size,
            flags);
   }

   return 0;
}

/** Hook called from the slave stack SDO Download handler to act on
 * user specified Index and Sub-index.
 *
 * @param[in] index      = index of SDO download request to handle
 * @param[in] sub-index  = sub-index of SDO download request to handle
 * @return SDO abort code, or 0 on success
 */
uint32_t ESC_download_post_objecthandler (uint16_t index, uint8_t subindex, uint16_t flags)
{
   if (ESCvar.post_object_download_hook != NULL)
   {
      return (ESCvar.post_object_download_hook)(index, subindex, flags);
   }

   return 0;
}

/** Function to pre-qualify the incoming SDO upload.
 *
 * @param[in] index      = index of SDO upload request to handle
 * @param[in] sub-index  = sub-index of SDO upload request to handle
 * @return SDO abort code, or 0 on success
 */
uint32_t ESC_upload_pre_objecthandler (uint16_t index,
      uint8_t subindex,
      void * data,
      size_t *size,
      uint16_t flags)
{
   if (ESCvar.pre_object_upload_hook != NULL)
   {
      return (ESCvar.pre_object_upload_hook) (index,
            subindex,
            data,
            size,
            flags);
   }

   return 0;
}

/** Hook called from the slave stack SDO Upload handler to act on
 * user specified Index and Sub-index.
 *
 * @param[in] index      = index of SDO upload request to handle
 * @param[in] sub-index  = sub-index of SDO upload request to handle
 * @return SDO abort code, or 0 on success
 */
uint32_t ESC_upload_post_objecthandler (uint16_t index, uint8_t subindex, uint16_t flags)
{
   if (ESCvar.post_object_upload_hook != NULL)
   {
      return (ESCvar.post_object_upload_hook)(index, subindex, flags);
   }

   return 0;
}

/** Write local process data to Sync Manager 3, Master Inputs.
 */
void TXPDO_update (void)
{
   if(ESCvar.txpdo_override != NULL)
   {
      (ESCvar.txpdo_override)();
   }
   else
   {
      if (MAX_MAPPINGS_SM3 > 0)
      {
         COE_pdoPack (txpdo, ESCvar.sm3mappings, SMmap3);
      }
      ESC_write (ESC_SM3_sma, txpdo, ESCvar.ESC_SM3_sml);
   }
}

/** Read Sync Manager 2 to local process data, Master Outputs.
 */
void RXPDO_update (void)
{
   if(ESCvar.rxpdo_override != NULL)
   {
      (ESCvar.rxpdo_override)();
   }
   else
   {
      ESC_read (ESC_SM2_sma, rxpdo, ESCvar.ESC_SM2_sml);
      if (MAX_MAPPINGS_SM2 > 0)
      {
         COE_pdoUnpack (rxpdo, ESCvar.sm2mappings, SMmap2);
      }
   }
}

/** Initializing the stack software
 */
void soes_init (esc_cfg_t * config)
{
	DPRINT ("Slave stack init started\n");
    /* Call stack configuration */
    ESC_config (config);
    /* Call HW init */
    ESC_init (config);

    /*  wait until ESC is started up */
    while ((ESCvar.DLstatus & 0x0001) == 0)
    {
        ESC_read (ESCREG_DLSTATUS, (void *) &ESCvar.DLstatus,
                sizeof (ESCvar.DLstatus));
        ESCvar.DLstatus = etohs (ESCvar.DLstatus);
    }

#if USE_FOE
    /* Pre FoE to set up Application information */
    bootstrap_foe_init ();
    /* Init FoE */
    FOE_init();
#endif

#if USE_EOE
   /* Init EoE */
   EOE_init();
#endif

    /* reset ESC to init state */
    ESC_ALstatus (ESCinit);
    ESC_ALerror (ALERR_NONE);
    ESC_stopmbx ();
    ESC_stopinput ();
    ESC_stopoutput ();
}


/*  Application loop for cyclic read the EtherCAT state and
 *  staus, update of I/O.
 */
void soes_loop() {

	static uint16_t old_alevent;
    /* Read local time from ESC*/
    ESC_read (ESCREG_LOCALTIME, (void *) &ESCvar.Time, sizeof (ESCvar.Time));
    ESCvar.Time = etohl (ESCvar.Time);
#ifdef STM32F429xx
    //ESCvar.ALevent = ESC_ALeventread();
    if ( ESCvar.ALevent && old_alevent != ESCvar.ALevent) {
    	old_alevent = ESCvar.ALevent;
    	DPRINT("%s ESC_ALeventread 0x%04X\n",__FUNCTION__,ESCvar.ALevent);
    }
#endif
    /* Read DC cyclic activation reg */
    //ESC_read (ESCREG_DC_ACTIVATION, (void *) &ESCvar.DC_activation, sizeof (ESCvar.DC_activation));
    //ESCvar.DC_activation = etohl (ESCvar.DC_activation);

    /* Check the state machine */
    ESC_state();
    /* Check the SM activation event */
    ESC_sm_act_event();

    /* Check mailboxes */
    if (ESC_mbxprocess())
    {
       ESC_coeprocess();
 #if USE_FOE
       ESC_foeprocess();
 #endif
 #if USE_EOE
       ESC_eoeprocess();
 #endif
       ESC_xoeprocess();
    }
 #if USE_EOE
    ESC_eoeprocess_tx();
 #endif

    /* Call emulated eeprom handler if set */
    if (ESCvar.esc_hw_eep_handler != NULL)
    {
       (ESCvar.esc_hw_eep_handler)();
    }

}
