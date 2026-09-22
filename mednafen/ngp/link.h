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
/* Fixed-window frames (core option ngp_fixed_frames), which netplay pins on.
 *
 * Stock ends a retro_run at the instruction that crosses vblank, so a frame is
 * 102485 ticks give or take that instruction, and two cabled units' frames end
 * on different bus ticks: there is no instant at which "every machine at frame
 * N" names one point in the cable's time, which is what a rollback restores
 * the whole cable to. Here every retro_run is the same 102485-tick window, the
 * overshoot carried, so every unit's frame N ends on one tick of the bus.
 *
 * begin returns the ticks this window is to run; end is told how many ran. */
extern bool ngp_fixed_frames;
#define NGP_FRAME_TICKS 102485u
uint32_t ngp_link_frame_begin(uint32_t window);
void ngp_link_frame_end(uint32_t ran, uint32_t ticks);
/* Ticks this machine has run, cable or not: the emulated clock. */
uint64_t ngp_link_own_clock(void);
int ngp_link_StateAction(void *data, int load, int data_only);
/* In mem.c: SC0BUF and COMMStatus. */
int ngp_sio_StateAction(void *data, int load, int data_only);

#ifdef __cplusplus
}
#endif

#endif
