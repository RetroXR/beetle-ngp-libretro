/* The SNK link cable, carried over the frontend's link bus.
 *
 * The Neo Geo Pocket's serial port is a UART: each end clocks its own bytes out
 * and a byte that arrives lands in SC0BUF and raises INTRX0. There is no master
 * and no shared clock, so unlike a Game Boy lead nothing here has to settle who
 * drives the line -- the cable is two byte streams, one each way.
 *
 * A byte sent is stamped a horizon ahead of this machine's clock and broadcast.
 * A byte received waits in the inbox until this machine's own clock reaches its
 * stamp, then the comms hooks hand it to the guest. Both machines rendezvous
 * every grain, so neither can run more than a horizon ahead of the other and a
 * byte always lands at the same emulated moment on the machine receiving it,
 * whatever the host's threads did -- which is what netplay replays rely on. */

#include <string.h>

#include <libretro.h>

#include "link.h"
#include "link_interface.h"
#include "system.h"

extern retro_log_printf_t log_cb;

/* Wire name. A peer with any other id is never joined to this one. */
#define LINK_PROTOCOL "ngp-sio-1"

/* The TLCS-900H's 6.144 MHz, the unit TLCS900h_interpret() counts in (515 a
 * scanline, 199 lines, 59.95 frames). */
#define LINK_CLOCK_RATE ((uint64_t)6144000)

/* Four scanlines. A byte at the 19200 baud every SNK title uses takes about
 * 3200 ticks, so a byte is late by less than its own length; the BIOS polls the
 * receiver once a scanline anyway. Each grain is a rendezvous between two
 * emulation threads, which is what stops this going much lower. */
#define LINK_GRAIN ((uint64_t)2060)

/* One byte on the wire at 19200 baud, start and stop bits included. The BIOS
 * hands a game's whole buffer over in one call, but the UART still shifts it out
 * a byte at a time, and a receiver's interrupt handler is written for that
 * spacing. */
#define LINK_BYTE_TICKS ((uint64_t)3200)

#define LINK_MSG_SIZE 8
#define LINK_MSG_BYTE 1
#define LINK_MSG_RTS  2

#define INBOX_MAX 256

static const struct retro_link_interface *link_if;
static struct retro_link_interface link_storage;
static retro_link_port_t *link_port;
static bool anchored;
static unsigned peers;

static uint64_t now;
static uint64_t limit;
static uint64_t line_free;
/* When the byte a game last wrote to SC0BUF finishes shifting out; 0 = none.
 * Kept even with no cable in: the UART completes into thin air regardless. */
static uint64_t tx_done_at;
static uint64_t own_clock;

static uint8_t inbox[INBOX_MAX];
static uint64_t inbox_tick[INBOX_MAX];
static unsigned inbox_count;
/* How many at the front have raised INTRX0. They stay in the inbox, which is
 * also the BIOS's receive buffer, until a BIOS read takes them. */
static unsigned inbox_raised;

/* The cable's handshake pair: each end's RTS output is the other's CTS input,
 * and several games wait on it between messages rather than on a byte. 0 is
 * asserted, as COMONRTS writes it; a pin nobody drives reads 1. */
#define RTS_MAX 64
static uint8_t local_rts = 1;
static uint8_t peer_rts = 1;
static bool rts_unsaid;
static uint8_t rts_val[RTS_MAX];
static uint64_t rts_tick[RTS_MAX];
static unsigned rts_count;

static void say(enum retro_log_level level, const char *msg, unsigned a, int b)
{
   if (log_cb)
      log_cb(level, msg, a, b);
}

void ngp_link_init(retro_environment_t env)
{
   memset(&link_storage, 0, sizeof(link_storage));
   link_if = NULL;
   if (env(RETRO_ENVIRONMENT_GET_LINK_INTERFACE, &link_storage) ||
       env(RETRO_ENVIRONMENT_GET_LINK_INTERFACE_FINAL, &link_storage))
      link_if = &link_storage;
}

void ngp_link_start(void)
{
   if (!link_if || link_port)
      return;
   link_port = link_if->attach(0, LINK_PROTOCOL, LINK_CLOCK_RATE);
   anchored = false;
   peers = 0;
   inbox_count = 0;
   inbox_raised = 0;
   limit = now;
   if (!link_port)
      say(RETRO_LOG_WARN, "Link cable: the frontend refused port %u%d\n", 0, 0);
}

void ngp_link_stop(void)
{
   if (!link_port)
      return;
   link_if->detach(link_port);
   link_port = NULL;
   inbox_count = 0;
   inbox_raised = 0;
   peers = 0;
}

void ngp_link_reset(void)
{
   /* The clock carries on: the bus must never see it go backwards. Only what
    * was in flight to the old machine goes. */
   inbox_count = 0;
   inbox_raised = 0;
   anchored = false;
}

static void drop_front(void);
void ngp_link_set_rts(uint8_t value);

static void pump(void)
{
   uint8_t buf[LINK_MSG_SIZE];
   uint64_t tick;
   unsigned from;
   size_t len = sizeof(buf);

   while (link_if->recv(link_port, &tick, &from, buf, &len))
   {
      if (len == LINK_MSG_SIZE && buf[0] == LINK_MSG_RTS)
      {
         if (rts_count < RTS_MAX)
         {
            rts_val[rts_count] = buf[2];
            rts_tick[rts_count] = tick;
            rts_count++;
         }
      }
      else if (len == LINK_MSG_SIZE && buf[0] == LINK_MSG_BYTE)
      {
         /* A game with its own receive handler never reads through the BIOS,
          * so what it has already been handed is let go to make room. */
         if (inbox_count == INBOX_MAX && inbox_raised > 0)
            drop_front();
         if (inbox_count < INBOX_MAX)
         {
            inbox[inbox_count] = buf[2];
            inbox_tick[inbox_count] = tick;
            inbox_count++;
         }
         else
            say(RETRO_LOG_WARN, "Link cable: inbox full, byte dropped%u%d\n", 0, 0);
      }
      len = sizeof(buf);
   }
}

static void refresh_peers(void)
{
   unsigned count = 0;
   int id = link_if->peers(link_port, &count);
   unsigned was = peers;

   peers = (id < 0) ? 0 : count;
   if (peers != was)
   {
      /* Bytes on their way to the last cable belong to it, not to this one,
       * and the far end has to be told where this one's RTS stands. */
      rts_count = 0;
      peer_rts = 1;
      rts_unsaid = true;
      inbox_count = 0;
      inbox_raised = 0;
      anchored = false;
      say(RETRO_LOG_WARN, "Link cable: %u machine(s) on the wire, this one is %d\n",
          peers, id);
   }
}

static void rendezvous(void)
{
   uint32_t wake = RETRO_LINK_WAKE_NONE;
   uint64_t grant;

   refresh_peers();
   grant = link_if->advance(link_port, now, now + LINK_GRAIN, now + LINK_GRAIN, &wake);
   anchored = true;
   pump();
   if (rts_unsaid && peers >= 2)
      ngp_link_set_rts(local_rts);

   if (grant == RETRO_LINK_UNBOUNDED)
      limit = now + LINK_GRAIN;       /* uncabled: look again a grain from now */
   else if (grant > now)
      limit = grant;
   else
      limit = now + 1;                /* woken without a grant: ask again at once */
}

void ngp_link_ran(unsigned ticks)
{
   own_clock += ticks;
   if (!link_port)
      return;
   now += ticks;
   if (now >= limit)
      rendezvous();
}

static void drop_front(void)
{
   inbox_count--;
   memmove(&inbox[0], &inbox[1], inbox_count);
   memmove(&inbox_tick[0], &inbox_tick[1], inbox_count * sizeof(inbox_tick[0]));
   if (inbox_raised > 0)
      inbox_raised--;
}

bool ngp_link_arrived(uint8_t *data)
{
   if (!link_port || inbox_raised >= inbox_count || inbox_tick[inbox_raised] > now)
      return false;
   *data = inbox[inbox_raised++];
   return true;
}

unsigned ngp_link_waiting(void)
{
   unsigned n = 0;
   if (!link_port)
      return 0;
   while (n < inbox_count && inbox_tick[n] <= now)
      n++;
   return n;
}

bool system_comms_poll(uint8_t *buffer)
{
   if (!link_port || inbox_count == 0 || inbox_tick[0] > now)
      return false;
   if (buffer)
      *buffer = inbox[0];
   return true;
}

bool system_comms_read(uint8_t *buffer)
{
   if (!system_comms_poll(buffer))
      return false;
   if (buffer)
      drop_front();
   return true;
}

static void send_msg(uint8_t type, uint8_t value, uint64_t tick)
{
   uint8_t msg[LINK_MSG_SIZE];

   /* The first rendezvous anchors the origin the bus measures ticks from; a
    * message sent before it lands in the peer's far future. Asking for where
    * this machine already stands is granted at once. */
   if (!anchored)
   {
      link_if->advance(link_port, now, now + LINK_GRAIN, now, 0);
      anchored = true;
   }
   memset(msg, 0, sizeof(msg));
   msg[0] = type;
   msg[2] = value;
   link_if->send(link_port, tick, RETRO_LINK_BROADCAST, msg, sizeof(msg));
}

void ngp_link_set_rts(uint8_t value)
{
   value &= 1;
   if (!link_port)
   {
      local_rts = value;
      return;
   }
   if (value == local_rts && !rts_unsaid)
      return;
   local_rts = value;
   if (peers < 2)
   {
      rts_unsaid = true;
      return;
   }
   rts_unsaid = false;
   send_msg(LINK_MSG_RTS, value, now + LINK_GRAIN);
}

uint8_t ngp_link_cts(void)
{
   if (!link_port || peers < 2)
      return 1;
   while (rts_count > 0 && rts_tick[0] <= now)
   {
      peer_rts = rts_val[0];
      rts_count--;
      memmove(&rts_val[0], &rts_val[1], rts_count);
      memmove(&rts_tick[0], &rts_tick[1], rts_count * sizeof(rts_tick[0]));
   }
   return peer_rts;
}

void ngp_link_tx_direct(uint8_t data)
{
   uint64_t start = own_clock;
   if (tx_done_at > start)
      start = tx_done_at;
   tx_done_at = start + LINK_BYTE_TICKS;
   system_comms_write(data);
}

bool ngp_link_tx_done(void)
{
   if (tx_done_at == 0 || own_clock < tx_done_at)
      return false;
   tx_done_at = 0;
   return true;
}

void system_comms_write(uint8_t data)
{
   if (!link_port || peers < 2)
      return;

   if (line_free < now + LINK_GRAIN)
      line_free = now + LINK_GRAIN;
   send_msg(LINK_MSG_BYTE, data, line_free);
   line_free += LINK_BYTE_TICKS;
}
