#ifndef __SOES_H__
#define __SOES_H__

#include <soes/esc.h>

#ifdef __cplusplus 
extern "C" {
#endif

void RXPDO_update (void);
void TXPDO_update (void);
void soes_init (esc_cfg_t * config);
void soes_loop (void);

#ifdef __cplusplus
}
#endif


#endif /* SOES_H */
