#ifndef __NGP_LINK__
#define __NGP_LINK__

#include <stdint.h>
#include <boolean.h>
#include <libretro.h>

#ifdef __cplusplus
extern "C" {
#endif

void ngp_link_init(retro_environment_t env);
void ngp_link_start(void);
void ngp_link_stop(void);
void ngp_link_reset(void);
/* Called after every CPU instruction with the ticks it took. */
void ngp_link_ran(unsigned ticks);
/* The next byte due that has not raised its receive interrupt yet. */
bool ngp_link_arrived(uint8_t *data);
/* How many bytes are due and unread: the BIOS receive buffer's count. */
unsigned ngp_link_waiting(void);
/* The comm register at 0xB2: bit 0 written is this end's RTS, read is CTS. */
void ngp_link_set_rts(uint8_t value);
uint8_t ngp_link_cts(void);
/* A game wrote SC0BUF itself: send it, and raise INTTX0 when it has gone. */
void ngp_link_tx_direct(uint8_t data);
bool ngp_link_tx_done(void);
/* In mem.c: set SC0BUF from the receive side. */
void ngp_sc0buf_received(uint8_t data);

#ifdef __cplusplus
}
#endif

#endif
