#include <soes/esc.h>
#include <soes/esc_coe.h>
#include <soes/esc_foe.h>
#include <soes/hal/advr_esc/soes.h>

#include <config.h>
#include <soes_hook.h>


#define DEFAULTTXPDOMAP    0x1a00
#define DEFAULTRXPDOMAP    0x1600
#define DEFAULTTXPDOITEMS  1
#define DEFAULTRXPDOITEMS  1

_ESCvar  	ESCvar;
uint8_t     MBX[MBXBUFFERS * MAX(MBXSIZE,MBXSIZEBOOT)];
_MBXcontrol MBXcontrol[MBXBUFFERS];

uint8_t		MBXrun = 0;
uint16_t    SM2_sml,SM3_sml;
_App        App;

uint16_t    txpdomap = DEFAULTTXPDOMAP;
uint16_t    rxpdomap = DEFAULTRXPDOMAP;
uint8_t     txpdoitems = DEFAULTTXPDOITEMS;
uint8_t     rxpdoitems = DEFAULTTXPDOITEMS;


/* Setup config hooks */
static esc_cfg_t config =
{
    .user_arg = "/spi0/et1100",
    .use_interrupt = 1,
    .watchdog_cnt = 0,
    .mbxsize = MBXSIZE,
    .mbxsizeboot = MBXSIZEBOOT,
    .mbxbuffers = MBXBUFFERS,
    .mb[0] = {MBX0_sma, MBX0_sml, MBX0_sme, MBX0_smc, 0},
    .mb[1] = {MBX1_sma, MBX1_sml, MBX1_sme, MBX1_smc, 0},
    .mb_boot[0] = {MBX0_sma_b, MBX0_sml_b, MBX0_sme_b, MBX0_smc_b, 0},
    .mb_boot[1] = {MBX1_sma_b, MBX1_sml_b, MBX1_sme_b, MBX1_smc_b, 0},
    .pdosm[0] = {SM2_sma, 0, 0, SM2_smc, SM2_act},
    .pdosm[1] = {SM3_sma, 0, 0, SM3_smc, SM3_act},
    .pre_state_change_hook = pre_state_change_hook,
    .post_state_change_hook = post_state_change_hook,
    .application_hook = NULL,
    .safeoutput_override = NULL,
    .pre_object_download_hook = ESC_pre_objecthandler,
    .post_object_download_hook = ESC_objecthandler,
    .rxpdo_override = NULL,
    .txpdo_override = NULL,
    .esc_hw_interrupt_enable = NULL,
    .esc_hw_interrupt_disable = NULL,
    .esc_hw_eep_handler = NULL
};


/** Initializing the stack software
 */
void soes_init (void)
{
    DPRINT ("SOES (Simple Open EtherCAT Slave)\n");

    ESCvar.TXPDOsize = ESCvar.ESC_SM3_sml = sizeOfPDO(TX_PDO_OBJIDX);
    ESCvar.RXPDOsize = ESCvar.ESC_SM2_sml = sizeOfPDO(RX_PDO_OBJIDX);

    ESC_config (&config);
    ESC_init (&config);

    /*  wait until ESC is started up */
    while ((ESCvar.DLstatus & 0x0001) == 0)
    {
        ESC_read (ESCREG_DLSTATUS, (void *) &ESCvar.DLstatus,
                sizeof (ESCvar.DLstatus));
        ESCvar.DLstatus = etohs (ESCvar.DLstatus);
    }

    /* Pre FoE to set up Application information */
    bootstrap_foe_init ();
    /* Init FoE */
    FOE_init();

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

    uint8_t mbxtype = 0;

    /* On init restore PDO mappings to default size */
    if ( (ESCvar.ALstatus & 0x0f) == ESCinit ) {
        txpdomap = DEFAULTTXPDOMAP;
        rxpdomap = DEFAULTRXPDOMAP;
        txpdoitems = DEFAULTTXPDOITEMS;
        rxpdoitems = DEFAULTTXPDOITEMS;
    }

    ESC_read (ESCREG_DLSTATUS, (void *) &ESCvar.DLstatus, sizeof (ESCvar.DLstatus));
    ESCvar.DLstatus = etohs (ESCvar.DLstatus);
    /* Read DC cyclic activation reg */
    //ESC_read (ESCREG_DC_ACTIVATION, (void *) &ESCvar.DC_activation, sizeof (ESCvar.DC_activation));
    //ESCvar.DC_activation = etohl (ESCvar.DC_activation);

    /* Check the state machine */
    ESC_state();
    /* Check the SM activation event */
    //ESC_sm_act_event();

    /* we're running normal execution
     *  - MailBox
     *  - CoE
     *  - FoE
     */
    if ( ESC_mbxprocess() ) {

        // set ESCvar.xoe looking at mailbox header
        ESC_mbxtype(&mbxtype);

        if (mbxtype == MBXCOE){
            ESC_coeprocess();
        } else if (mbxtype == MBXFOE){
            ESC_foeprocess();
        }
        ESC_xoeprocess();
    }

}
