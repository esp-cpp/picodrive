/*
 * PicoDrive
 * (C) notaz, 2009,2010,2013
 * (C) irixxxx, 2019-2024
 *
 * This work is licensed under the terms of MAME license.
 * See COPYING file in the top-level directory.
 */
#include "../pico_int.h"
#include "../sound/ym2612.h"
#include <cpu/sh2/compiler.h>

struct Pico32x Pico32x;
SH2 sh2s[2];

#define SH2_IDLE_STATES (SH2_STATE_CPOLL|SH2_STATE_VPOLL|SH2_STATE_RPOLL|SH2_STATE_SLEEP)

static int REGPARM(2) sh2_irq_cb(SH2 *sh2, int level)
{
  if (sh2->pending_irl > sh2->pending_int_irq) {
    elprintf_sh2(sh2, EL_32X, "ack/irl %d @ %08x",
      level, sh2_pc(sh2));
    return 64 + sh2->pending_irl / 2;
  } else {
    elprintf_sh2(sh2, EL_32X, "ack/int %d/%d @ %08x",
      level, sh2->pending_int_vector, sh2_pc(sh2));
    sh2->pending_int_irq = 0; // auto-clear
    sh2->pending_level = sh2->pending_irl;
    return sh2->pending_int_vector;
  }
}

// MUST specify active_sh2 when called from sh2 memhandlers
void p32x_update_irls(SH2 *active_sh2, unsigned int m68k_cycles)
{
  int irqs, mlvl = 0, slvl = 0;
  int mrun, srun;

  if ((Pico32x.regs[0] & (P32XS_nRES|P32XS_ADEN)) != (P32XS_nRES|P32XS_ADEN))
    return;

  if (active_sh2 != NULL)
    m68k_cycles = sh2_cycles_done_m68k(active_sh2);

  // find top bit = highest irq number (0 <= irl <= 14/2) by binary search

  // msh2
  irqs = Pico32x.sh2irqi[0];
  if (irqs >= 0x10)     mlvl += 8, irqs >>= 4;
  if (irqs >= 0x04)     mlvl += 4, irqs >>= 2;
  if (irqs >= 0x02)     mlvl += 2, irqs >>= 1;

  // ssh2
  irqs = Pico32x.sh2irqi[1];
  if (irqs >= 0x10)     slvl += 8, irqs >>= 4;
  if (irqs >= 0x04)     slvl += 4, irqs >>= 2;
  if (irqs >= 0x02)     slvl += 2, irqs >>= 1;

  mrun = sh2_irl_irq(&msh2, mlvl, msh2.state & SH2_STATE_RUN);
  if (mrun) {
    p32x_sh2_poll_event(msh2.poll_addr, &msh2, SH2_IDLE_STATES & ~SH2_STATE_SLEEP, m68k_cycles);
    if (msh2.state & SH2_STATE_RUN)
      sh2_end_run(&msh2, 0);
  }

  srun = sh2_irl_irq(&ssh2, slvl, ssh2.state & SH2_STATE_RUN);
  if (srun) {
    p32x_sh2_poll_event(ssh2.poll_addr, &ssh2, SH2_IDLE_STATES & ~SH2_STATE_SLEEP, m68k_cycles);
    if (ssh2.state & SH2_STATE_RUN)
      sh2_end_run(&ssh2, 0);
  }

  elprintf(EL_32X, "update_irls: m %d/%d, s %d/%d", mlvl, mrun, slvl, srun);
}

// the mask register is inconsistent, CMD is supposed to be a mask,
// while others are actually irq trigger enables?
// TODO: test on hw..
void p32x_trigger_irq(SH2 *sh2, unsigned int m68k_cycles, unsigned int mask)
{
  Pico32x.sh2irqi[0] |= mask & P32XI_VRES;
  Pico32x.sh2irqi[1] |= mask & P32XI_VRES;
  Pico32x.sh2irqi[0] |= mask & (Pico32x.sh2irq_mask[0] << 3);
  Pico32x.sh2irqi[1] |= mask & (Pico32x.sh2irq_mask[1] << 3);

  p32x_update_irls(sh2, m68k_cycles);
}

void p32x_update_cmd_irq(SH2 *sh2, unsigned int m68k_cycles)
{
  if ((Pico32x.sh2irq_mask[0] & 2) && (Pico32x.regs[2 / 2] & 1))
    Pico32x.sh2irqi[0] |= P32XI_CMD;
  else
    Pico32x.sh2irqi[0] &= ~P32XI_CMD;

  if ((Pico32x.sh2irq_mask[1] & 2) && (Pico32x.regs[2 / 2] & 2))
    Pico32x.sh2irqi[1] |= P32XI_CMD;
  else
    Pico32x.sh2irqi[1] &= ~P32XI_CMD;

  p32x_update_irls(sh2, m68k_cycles);
}

void Pico32xStartup(void)
{
  elprintf(EL_STATUS|EL_32X, "32X startup");

  PicoIn.AHW |= PAHW_32X;
  // TODO: OOM handling
  if (Pico32xMem == NULL) {
    Pico32xMem = plat_mmap(0x06000000, sizeof(*Pico32xMem), 0, 0);
    if (Pico32xMem == NULL) {
      elprintf(EL_STATUS, "OOM");
      return;
    }
    memset(Pico32xMem, 0, sizeof(struct Pico32xMem));

    sh2_init(&msh2, 0, &ssh2);
    msh2.irq_callback = sh2_irq_cb;
    sh2_init(&ssh2, 1, &msh2);
    ssh2.irq_callback = sh2_irq_cb;
  }

  PicoMemSetup32x();
  p32x_pwm_ctl_changed();

  Pico32x.regs[0] |= P32XS_ADEN;

  Pico32x.sh2_regs[0] = P32XS2_ADEN;
  if (Pico.m.ncart_in)
    Pico32x.sh2_regs[0] |= P32XS2_nCART;

  if (!Pico.m.pal)
    Pico32x.vdp_regs[0] |= P32XV_nPAL;
  else
    Pico32x.vdp_regs[0] &= ~P32XV_nPAL;

  rendstatus_old = -1;

  Pico32xPrepare();
  emu_32x_startup();
}

void Pico32xShutdown(void)
{
  elprintf(EL_STATUS|EL_32X, "32X shutdown");
  p32x_par_stop();
  Pico32x.sh2_regs[0] &= ~P32XS2_ADEN;
  Pico32x.regs[0] &= ~P32XS_ADEN;

  rendstatus_old = -1;

  PicoIn.AHW &= ~PAHW_32X;
  if (PicoIn.AHW & PAHW_MCD)
    PicoMemSetupCD();
  else
    PicoMemSetup();
  emu_32x_startup();
}

void p32x_reset_sh2s(void)
{
  elprintf(EL_32X, "sh2 reset");

  sh2_reset(&msh2);
  sh2_reset(&ssh2);
  sh2_peripheral_reset(&msh2);
  sh2_peripheral_reset(&ssh2);
  msh2.state = ssh2.state = 0;

  // if we don't have BIOS set, perform it's work here.
  // MSH2
  if (p32x_bios_m == NULL) {
    sh2_set_gbr(0, 0x20004000);

    if (!Pico.m.ncart_in) { // copy IDL from cartridge
      unsigned int idl_src, idl_dst, idl_size; // initial data load
      unsigned int vbr;
      // initial data
      idl_src = CPU_BE2(*(u32 *)(Pico.rom + 0x3d4)) & ~0xf0000000;
      idl_dst = CPU_BE2(*(u32 *)(Pico.rom + 0x3d8)) & ~0xf0000000;
      idl_size= CPU_BE2(*(u32 *)(Pico.rom + 0x3dc));
      // copy in guest memory space
      idl_src += 0x2000000;
      idl_dst += 0x6000000;
      while (idl_size >= 4) {
        p32x_sh2_write32(idl_dst, p32x_sh2_read32(idl_src, &msh2), &msh2);
        idl_src += 4, idl_dst += 4, idl_size -= 4;
      }

      // VBR
      vbr = CPU_BE2(*(u32 *)(Pico.rom + 0x3e8));
      sh2_set_vbr(0, vbr);

      // checksum and M_OK
      Pico32x.regs[0x28 / 2] = *(u16 *)(Pico.rom + 0x18e);
    }
    // program will set M_OK
  }

  // SSH2
  if (p32x_bios_s == NULL) {
    unsigned int vbr;

    // GBR/VBR
    vbr = CPU_BE2(*(u32 *)(Pico.rom + 0x3ec));
    sh2_set_gbr(1, 0x20004000);
    sh2_set_vbr(1, vbr);
    // program will set S_OK
  }

  msh2.m68krcycles_done = ssh2.m68krcycles_done = SekCyclesDone();
}

void Pico32xInit(void)
{
}

void PicoPower32x(void)
{
  memset(&Pico32x, 0, sizeof(Pico32x));
  memset(p32x_event_times, 0, sizeof(p32x_event_times));

  Pico32x.regs[0] = P32XS_REN|P32XS_nRES; // verified
  Pico32x.regs[0x10/2] = 0xffff;
  Pico32x.vdp_regs[0x0a/2] = P32XV_VBLK|P32XV_PEN;
}

void PicoUnload32x(void)
{
  if (PicoIn.AHW & PAHW_32X)
    Pico32xShutdown();

  sh2_finish(&msh2);
  sh2_finish(&ssh2);

  if (Pico32xMem != NULL)
    plat_munmap(Pico32xMem, sizeof(*Pico32xMem));
  Pico32xMem = NULL;
}

void PicoReset32x(void)
{
  P32X_PAR_BARRIER();
  if (PicoIn.AHW & PAHW_32X) {
    p32x_trigger_irq(NULL, Pico.t.m68c_aim, P32XI_VRES);
    p32x_m68k_poll_event(0, -1);
    p32x_sh2_poll_event(msh2.poll_addr, &msh2, SH2_IDLE_STATES, Pico.t.m68c_aim);
    p32x_sh2_poll_event(ssh2.poll_addr, &ssh2, SH2_IDLE_STATES, Pico.t.m68c_aim);
  }
}

static void Pico32xRenderSync(int lines)
{
  int offs;

  pprof_start(draw);

  offs = (240 - rendlines) >> 1;

  if ((Pico32x.vdp_regs[0] & P32XV_Mx) != 0 && // 32x not blanking
      (!(Pico.video.debug_p & PVD_KILL_32X)))
  {
    int md_bg = Pico.video.reg[7] & 0x3f;

    // we draw lines up to the sync point (not line-by-line)
    PicoDraw32xLayer(offs, lines-Pico32x.sync_line + 1, md_bg);
  }
  else
    PicoDraw32xLayerMdOnly(offs, lines-Pico32x.sync_line + 1);

  pprof_end(draw);
}

void Pico32xDrawSync(SH2 *sh2)
{
  if (p32x_par_on && sh2)
    return; // (the MD renderer belongs to the 68k thread)
  // the fast renderer isn't operating on a line-by-line base
  if (sh2 && !(PicoIn.opt & POPT_ALT_RENDERER) && !PicoIn.skipFrame) {
    unsigned int cycle = (sh2 ? sh2_cycles_done_m68k(sh2) : SekCyclesDone());
    int line = DIVQ32(cycle - Pico.t.m68c_frame_start, 488.5);

    cycle = cycle - Pico.t.m68c_frame_start - (int)(488.5*2)*line/2;
    if (cycle < 50) line--;

    if (line >= rendlines) line = rendlines-1;
    if (Pico32x.sync_line <= line) {
      // make sure the MD image is also sync'ed to this line for merging
      PicoDrawSync(line, 0, 0);

      if (Pico32xDrawMode != PDM32X_OFF) {
        // pfff... need to save and restore some persistent data for MD renderer
        void *dest = Pico.est.DrawLineDest;
        int incr = Pico.est.DrawLineDestIncr;
        Pico32xRenderSync(line);
        Pico.est.DrawLineDest = dest;
        Pico.est.DrawLineDestIncr = incr;
      }
    }

    // remember next line to draw
    Pico32x.sync_line = line+1;
  }
}

static void p32x_render_frame(void)
{
  // the SH-2s up to here, then on with the next frame while this one is
  // drawn: they draw into the other frame buffer, the swap waits for vblank
  // (no sync with SH-2s on their own thread: the front buffer only changes in
  // vblank, which reaches them as an event once they get there)
  if (Pico32xDrawMode != PDM32X_OFF && !PicoIn.skipFrame)
    Pico32xRenderSync(rendlines-1);
  Pico32x.sync_line = rendlines;
}

static void vblank_start_event(unsigned int now)
{
  // enter vblank
  Pico32x.vdp_regs[0x0a/2] |= P32XV_VBLK|P32XV_PEN;

  // FB swap waits until vblank
  if ((Pico32x.vdp_regs[0x0a/2] ^ Pico32x.pending_fb) & P32XV_FS) {
    Pico32x.vdp_regs[0x0a/2] ^= P32XV_FS;
    Pico32xSwapDRAM(Pico32x.pending_fb ^ P32XV_FS);
  }

  p32x_trigger_irq(NULL, now, P32XI_VINT);
  p32x_sh2_poll_event(msh2.poll_addr, &msh2, SH2_STATE_VPOLL, now);
  p32x_sh2_poll_event(ssh2.poll_addr, &ssh2, SH2_STATE_VPOLL, now);
}

// with the SH-2s on their own thread, the 68k side's vblank changes reach
// the 32X as events at their time: nobody waits for the SH-2s to get there
static void p32x_start_blank(void)
{
#ifdef P32X_SH2_THREAD
  if (p32x_par_on) {
    p32x_par_post_event(P32X_EVENT_VBLANK, Pico.t.m68c_aim);
    return;
  }
#endif
  vblank_start_event(Pico.t.m68c_aim);
}

static void vblank_end_event(unsigned int now)
{
  // end vblank
  Pico32x.vdp_regs[0x0a/2] &= ~P32XV_VBLK; // get out of vblank
  if ((Pico32x.vdp_regs[0] & P32XV_Mx) != 0) // no forced blanking
    Pico32x.vdp_regs[0x0a/2] &= ~P32XV_PEN; // no palette access
  if (!(Pico32x.sh2_regs[0] & 0x80)) {
    // Hint on 32X is taking place some 13 H40 px after HBLANK on MD
    Pico32x.hint_counter = (13 * 16/7) << 4;
    p32x_schedule_hint(NULL, now);
  }

  p32x_sh2_poll_event(msh2.poll_addr, &msh2, SH2_STATE_VPOLL, now);
  p32x_sh2_poll_event(ssh2.poll_addr, &ssh2, SH2_STATE_VPOLL, now);
}

static void p32x_end_blank(void)
{
#ifdef P32X_SH2_THREAD
  if (p32x_par_on) {
    p32x_par_post_event(P32X_EVENT_VBLANK_END, Pico.t.m68c_aim);
    return;
  }
#endif
  vblank_end_event(Pico.t.m68c_aim);
}

void p32x_schedule_hint(SH2 *sh2, unsigned int m68k_cycles)
{
  // rather rough, 32x hint is useless in practice
  int after;

  if (!((Pico32x.sh2irq_mask[0] | Pico32x.sh2irq_mask[1]) & 4))
    return; // nobody cares
  if ((Pico.video.status & PVS_VB2) && !(Pico32x.sh2_regs[0] & 0x80))
    return;

  Pico32x.hint_counter += (Pico32x.sh2_regs[4 / 2] + 1) * (int)(488.5 * (1<<4));
  after = Pico32x.hint_counter >> 4;
  Pico32x.hint_counter &= 0xf;
  if (sh2 != NULL)
    p32x_event_schedule_sh2(sh2, P32X_EVENT_HINT, after);
  else
    p32x_event_schedule(m68k_cycles, P32X_EVENT_HINT, after);
}

/* events */
static void fillend_event(unsigned int now)
{
  Pico32x.vdp_regs[0x0a/2] &= ~P32XV_nFEN;
  p32x_sh2_poll_event(msh2.poll_addr, &msh2, SH2_STATE_VPOLL, now);
  p32x_sh2_poll_event(ssh2.poll_addr, &ssh2, SH2_STATE_VPOLL, now);
}

static void hint_event(unsigned int now)
{
  p32x_trigger_irq(NULL, now, P32XI_HINT);
  p32x_schedule_hint(NULL, now);
}

static void mtimer_event(unsigned int now)
{
  p32x_timer_irq(&msh2, now);
}

static void stimer_event(unsigned int now)
{
  p32x_timer_irq(&ssh2, now);
}

typedef void (event_cb)(unsigned int now);

/* times are in m68k (7.6MHz) cycles */
unsigned int p32x_event_times[P32X_EVENT_COUNT];
static unsigned int event_time_next;
static event_cb *p32x_event_cbs[P32X_EVENT_COUNT] = {
  p32x_pwm_irq_event, // P32X_EVENT_PWM
  fillend_event,      // P32X_EVENT_FILLEND
  hint_event,         // P32X_EVENT_HINT
  mtimer_event,       // P32X_EVENT_MTIMER
  stimer_event,       // P32X_EVENT_STIMER
  vblank_start_event, // P32X_EVENT_VBLANK
  vblank_end_event,   // P32X_EVENT_VBLANK_END
};

// schedule event at some time 'after', in m68k clocks
void p32x_event_schedule(unsigned int now, enum p32x_event event, int after)
{
  unsigned int when;

  when = (now + after) | 1;

  elprintf(EL_32X, "32x: new event #%u %u->%u", event, now, when);
  p32x_event_times[event] = when;

  if (event_time_next == 0 || CYCLES_GT(event_time_next, when))
    event_time_next = when;
}

void p32x_event_schedule_sh2(SH2 *sh2, enum p32x_event event, int after)
{
  unsigned int now = sh2_cycles_done_m68k(sh2);
  int left_to_next;

  // (an idle SH-2's clock lags: from the 68k's time, or, with the SH-2s on
  // their own thread, the time they run to: the 68k's may be behind theirs)
  if (sh2->state & SH2_IDLE_STATES)
    now = P32X_SH2_M68K_NOW();

  p32x_event_schedule(now, event, after);

  left_to_next = C_M68K_TO_SH2(sh2, (int)(event_time_next - now));
  if (sh2_cycles_left(sh2) > left_to_next) {
    if (left_to_next < 1)
      left_to_next = 0;
    sh2_end_run(sh2, left_to_next);
  }
}

static void p32x_run_events(unsigned int until)
{
  int oldest, oldest_diff, time;
  int i, diff;

  while (1) {
    oldest = -1, oldest_diff = 0x7fffffff;

    for (i = 0; i < P32X_EVENT_COUNT; i++) {
      if (p32x_event_times[i]) {
        diff = p32x_event_times[i] - until;
        if (diff < oldest_diff) {
          oldest_diff = diff;
          oldest = i;
        }
      }
    }

    if (oldest_diff <= 0) {
      time = p32x_event_times[oldest];
      p32x_event_times[oldest] = 0;
      elprintf(EL_32X, "32x: run event #%d %u", oldest, time);
      p32x_event_cbs[oldest](time);
    }
    else if (oldest_diff < 0x7fffffff) {
      event_time_next = p32x_event_times[oldest];
      break;
    }
    else {
      event_time_next = 0;
      break;
    }
  }

  if (oldest != -1)
    elprintf(EL_32X, "32x: next event #%d at %u",
      oldest, event_time_next);
}

static void run_sh2(SH2 *sh2, unsigned int m68k_cycles)
{
  unsigned int cycles, done;

  pevt_log_sh2_o(sh2, EVT_RUN_START);
  sh2->state |= SH2_STATE_RUN;
  cycles = C_M68K_TO_SH2(sh2, m68k_cycles);
  elprintf_sh2(sh2, EL_32X, "+run %u %d @%08x",
    sh2->m68krcycles_done, cycles, sh2->pc);

  done = sh2_execute(sh2, cycles);

  sh2->m68krcycles_done += C_SH2_TO_M68K(sh2, done);
  sh2->state &= ~SH2_STATE_RUN;
  pevt_log_sh2_o(sh2, EVT_RUN_END);
  elprintf_sh2(sh2, EL_32X, "-run %u %d",
    sh2->m68krcycles_done, done);
}

// sync other sh2 to this one
// note: recursive call
void p32x_sync_other_sh2(SH2 *sh2, unsigned int m68k_target)
{
  SH2 *osh2 = sh2->other_sh2;
  int left_to_event;
  int m68k_cycles;

  if (osh2->state & SH2_STATE_RUN) {
    sh2_end_run(sh2, 0);
    return;
  }

  m68k_cycles = m68k_target - osh2->m68krcycles_done;
  if (m68k_cycles < 46) // Brutal
    return;

  if (osh2->state & SH2_IDLE_STATES) {
    osh2->m68krcycles_done = m68k_target;
    return;
  }

  elprintf_sh2(osh2, EL_32X, "sync to %u %d",
    m68k_target, m68k_cycles);

  run_sh2(osh2, m68k_cycles);

  // there might be new event to schedule current sh2 to
  if (event_time_next) {
    left_to_event = C_M68K_TO_SH2(sh2, (int)(event_time_next - m68k_target));
    if (sh2_cycles_left(sh2) > left_to_event) {
      if (left_to_event < 1)
        left_to_event = 0;
      sh2_end_run(sh2, left_to_event);
    }
  }
}

#define STEP_LS 24
#define STEP_N 164 // NFL, Xmen; TODO at least a scanline (489) for good perf?

#ifdef PD_SH2_STATS
// host cycles spent running each SH-2, and in the whole sync (the platform
// reads and resets them)
unsigned int pd_stat_clock(void);
unsigned int pd_stat_sh2_cycles[3];
#define SH2_STAT_START() unsigned int stat_t0_ = pd_stat_clock()
#define SH2_STAT_END(i) pd_stat_sh2_cycles[i] += pd_stat_clock() - stat_t0_
#else
#define SH2_STAT_START()
#define SH2_STAT_END(i)
#endif

#ifdef P32X_SH2_THREAD
#include <stdatomic.h>
// on their own thread, the SH-2s run in shorter slices: the 68k side waits
// for the current one to end whenever it touches 32X state
#ifndef P32X_PAR_STEP
#define P32X_PAR_STEP 48
#endif
#define SYNC_STEP (p32x_par_on ? P32X_PAR_STEP : STEP_N)
// the 68k side wants the SH-2s to stop at their next slice
static atomic_int par_park;
static atomic_uint par_req, par_done;  // m68k times: allowed, reached
static unsigned int sync_sh2s_normal(unsigned int m68k_target);
#define SYNC_RET unsigned int
#define SYNC_RETURN(t) return (t)
#else
#define SYNC_STEP STEP_N
#define sync_sh2s_normal p32x_sync_sh2s
#define SYNC_RET void
#define SYNC_RETURN(t) return
#endif
//#define sync_sh2s_lockstep p32x_sync_sh2s

/* most timing is in 68k clock */
#ifdef P32X_SH2_THREAD
static
#endif
// (when parallel: returns the time reached, earlier than m68k_target when
// the 68k side asked them to stop)
SYNC_RET sync_sh2s_normal(unsigned int m68k_target)
{
  unsigned int now, target, next;
  int cycles;

  elprintf(EL_32X, "sh2 sync to %u", m68k_target);

  if ((Pico32x.regs[0] & (P32XS_nRES|P32XS_ADEN)) != (P32XS_nRES|P32XS_ADEN)) {
    msh2.m68krcycles_done = ssh2.m68krcycles_done = m68k_target;
    SYNC_RETURN(m68k_target); // rare
  }

  now = msh2.m68krcycles_done;
  if (CYCLES_GT(now, ssh2.m68krcycles_done))
    now = ssh2.m68krcycles_done;

#ifdef PD_SH2_STATS
  unsigned int stat_sync0_ = pd_stat_clock();
#endif
  pprof_start(m68k);
  while (CYCLES_GT(m68k_target, now))
  {
    if (event_time_next && CYCLES_GE(now, event_time_next))
      p32x_run_events(now);

    target = m68k_target;
    if (event_time_next && CYCLES_GT(target, event_time_next))
      target = event_time_next;
    while (CYCLES_GT(target, now))
    {
      next = target;
      if (CYCLES_GT(target, now + SYNC_STEP))
        next = now + SYNC_STEP;
      elprintf(EL_32X, "sh2 exec to %u %d,%d/%d, flags %x", next,
        next - msh2.m68krcycles_done, next - ssh2.m68krcycles_done,
        m68k_target - now, Pico32x.emu_flags);

      pprof_start(ssh2);
      if (!(ssh2.state & SH2_IDLE_STATES)) {
        cycles = next - ssh2.m68krcycles_done;
        if (cycles > 0) {
          SH2_STAT_START();
          run_sh2(&ssh2, cycles > 20U ? cycles : 20U);
          SH2_STAT_END(1);

          if (event_time_next && CYCLES_GT(target, event_time_next))
            target = event_time_next;
          if (CYCLES_GT(next, target))
            next = target;
        }
      }
      pprof_end(ssh2);

      pprof_start(msh2);
      if (!(msh2.state & SH2_IDLE_STATES)) {
        cycles = next - msh2.m68krcycles_done;
        if (cycles > 0) {
          SH2_STAT_START();
          run_sh2(&msh2, cycles > 20U ? cycles : 20U);
          SH2_STAT_END(0);

          if (event_time_next && CYCLES_GT(target, event_time_next))
            target = event_time_next;
          if (CYCLES_GT(next, target))
            next = target;
        }
      }
      pprof_end(msh2);

      now = next;
      if (CYCLES_GT(now, msh2.m68krcycles_done)) {
        if (!(msh2.state & SH2_IDLE_STATES))
          now = msh2.m68krcycles_done;
      }
      if (CYCLES_GT(now, ssh2.m68krcycles_done)) {
        if (!(ssh2.state & SH2_IDLE_STATES)) 
          now = ssh2.m68krcycles_done;
      }
#ifdef P32X_SH2_THREAD
      if (p32x_par_on) {
        // where they are, for the 68k side waiting for them to get there
        atomic_store_explicit(&par_done, now, memory_order_release);
        if (atomic_load_explicit(&par_park, memory_order_relaxed)) {
          m68k_target = now; // stop here, the 68k side wants the 32X
          goto parked;
        }
      }
#endif
    }
  }
#ifdef P32X_SH2_THREAD
parked:
#endif
  pprof_end_sub(m68k);
#ifdef PD_SH2_STATS
  pd_stat_sh2_cycles[2] += pd_stat_clock() - stat_sync0_;
#endif

  // advance idle CPUs
  if (msh2.state & SH2_IDLE_STATES) {
    if (CYCLES_GT(m68k_target, msh2.m68krcycles_done))
      msh2.m68krcycles_done = m68k_target;
  }
  if (ssh2.state & SH2_IDLE_STATES) {
    if (CYCLES_GT(m68k_target, ssh2.m68krcycles_done))
      ssh2.m68krcycles_done = m68k_target;
  }
  SYNC_RETURN(m68k_target);
}

void sync_sh2s_lockstep(unsigned int m68k_target)
{
  unsigned int mcycles;
  
  mcycles = msh2.m68krcycles_done;
  if (CYCLES_GT(mcycles, ssh2.m68krcycles_done))
    mcycles = ssh2.m68krcycles_done;

  while (CYCLES_GT(m68k_target, mcycles)) {
    mcycles += STEP_LS;
    sync_sh2s_normal(mcycles);
  }
}

#ifdef P32X_SH2_THREAD
int p32x_par_on;
unsigned int p32x_par_target;          // the time the SH-2 thread runs to
static atomic_int par_running;         // the SH-2 thread is in a sync
static atomic_int par_m68k_wake;       // the SH-2s ended the 68k's polling
#ifndef P32X_PAR_LAX
#define P32X_PAR_LAX 244 // m68k cycles the SH-2s may trail a 68k access by
#endif                   // (what the serial scheduler allows comm reads)
#ifndef P32X_PAR_LAX_FRAME
#define P32X_PAR_LAX_FRAME 1954 // ... at the frame's sync points (vblank, the
#endif                          // 32X layer render, timers, PWM mixing)
#ifndef P32X_PAR_LAX_PWM
#define P32X_PAR_LAX_PWM 32768 // ... when the PWM is mixed (a quarter frame)
#endif
#ifndef P32X_PAR_AHEAD
#define P32X_PAR_AHEAD 244 // ... and may run ahead of the 68k: work for them
#endif                     // while the 68k side renders, mixes, ...
#ifdef PD_SH2_STATS
#include <stdlib.h>
// a wait that never ends: abort, for a core dump with every thread's stack
#define PAR_SPIN_START() unsigned int spin_t0_ = pd_stat_clock()
#define PAR_SPIN_CHECK() do { if (pd_stat_clock() - spin_t0_ > 720000000u) abort(); } while (0)
#else
#define PAR_SPIN_START()
#define PAR_SPIN_CHECK()
#endif
#ifdef PD_SH2_STATS
// accesses, the 68k thread's cycles waiting at them, the m68k cycles the
// SH-2s trailed by, poll throttles and their wait
unsigned int pd_stat_par[12];
#define PAR_STAT(x) x
#else
#define PAR_STAT(x)
#endif

void p32x_sync_sh2s(unsigned int m68k_target)
{
  if (p32x_par_on)
    p32x_par_barrier(m68k_target);
  else
    sync_sh2s_normal(m68k_target);
}

// 68k thread: the SH-2s may run up to m68k_now (and a little beyond)
void p32x_par_release(unsigned int m68k_now)
{
  m68k_now += P32X_PAR_AHEAD;
  if (CYCLES_GT(m68k_now, atomic_load_explicit(&par_req, memory_order_relaxed))) {
    atomic_store_explicit(&par_req, m68k_now, memory_order_seq_cst);
    plat_sh2_thread_kick();
  }
}

static void par_unpark(void)
{
  if (atomic_load_explicit(&par_park, memory_order_relaxed)) {
    atomic_store_explicit(&par_park, 0, memory_order_seq_cst);
    plat_sh2_thread_kick();
  }
}

// the SH-2 thread out of its sync: from here on the 68k thread owns the 32X
static void par_park_now(void)
{
  PAR_STAT(unsigned int pt0 = pd_stat_clock());
  PAR_SPIN_START();
  atomic_store_explicit(&par_park, 1, memory_order_seq_cst);
  while (atomic_load_explicit(&par_running, memory_order_seq_cst))
    PAR_SPIN_CHECK();
  if (atomic_exchange_explicit(&par_m68k_wake, 0, memory_order_relaxed))
    SekSetStop(0);
  PAR_STAT(pd_stat_par[8] += pd_stat_clock() - pt0);
}

// let the SH-2s run until they are within `lag` of m68k_now
void p32x_par_catch_up(unsigned int m68k_now, unsigned int lag)
{
#ifdef PD_SH2_STATS
  int behind = (int)(m68k_now - atomic_load_explicit(&par_done, memory_order_relaxed));
  unsigned int ct0 = pd_stat_clock();
  if (lag > P32X_PAR_LAX && behind > 0) { // (frame-level and exact syncs)
    pd_stat_par[11] += behind;
    if ((unsigned)behind > pd_stat_par[10]) pd_stat_par[10] = behind;
  }
#endif
  p32x_par_release(m68k_now);
  if (CYCLES_GT(m68k_now - lag, atomic_load_explicit(&par_done, memory_order_acquire))) {
    PAR_SPIN_START();
    par_unpark();
    while (CYCLES_GT(m68k_now - lag, atomic_load_explicit(&par_done, memory_order_acquire)))
      PAR_SPIN_CHECK();
  }
  PAR_STAT(pd_stat_par[9] += pd_stat_clock() - ct0);
}

// 68k thread: the SH-2s at least at m68k_now (at most P32X_PAR_AHEAD
// beyond) and stopped until the 68k side lets them go again
void p32x_par_barrier(unsigned int m68k_now)
{
  PAR_STAT(unsigned int t0 = pd_stat_clock());
  PAR_STAT(pd_stat_par[5]++);
  p32x_par_catch_up(m68k_now, 0);
  par_park_now();
  PAR_STAT(pd_stat_par[6] += pd_stat_clock() - t0);
}

// 68k thread, at the frame's sync points: the SH-2s stopped within
// P32X_PAR_LAX_FRAME of the 68k (until P32X_PAR_LEAVE())
void p32x_par_frame_sync(unsigned int m68k_now)
{
  PAR_STAT(unsigned int t0 = pd_stat_clock());
  PAR_STAT(pd_stat_par[5]++);
  p32x_par_catch_up(m68k_now, P32X_PAR_LAX_FRAME);
  par_park_now();
  PAR_STAT(pd_stat_par[6] += pd_stat_clock() - t0);
}

// 68k thread: the 68k side changes 32X state at m68k_now, as an event the
// SH-2 thread applies when it gets there
void p32x_par_post_event(enum p32x_event event, unsigned int m68k_now)
{
  par_park_now(); // (the event list belongs to the SH-2 thread while it runs)
  p32x_event_schedule(m68k_now, event, 0);
  par_unpark();
  p32x_par_release(m68k_now);
}

// 68k thread, before mixing the PWM: its samples up to near m68k_now
void p32x_par_pwm_sync(unsigned int m68k_now)
{
  p32x_par_catch_up(m68k_now, P32X_PAR_LAX_PWM);
  par_park_now();
}

// 68k thread, around an access to 32X state: the SH-2s stopped within
// P32X_PAR_LAX of the 68k (an access that needs them exactly there syncs
// them further, see p32x_sync_sh2s), let go again after it
void p32x_par_enter(unsigned int m68k_now)
{
  PAR_STAT(unsigned int t0 = pd_stat_clock());
  PAR_STAT(pd_stat_par[0]++);
  PAR_STAT({ int b_ = (int)(m68k_now - atomic_load_explicit(&par_done, memory_order_relaxed)); if (b_ > 0) pd_stat_par[2] += b_; });
  p32x_par_catch_up(m68k_now, P32X_PAR_LAX);
  par_park_now();
  PAR_STAT(pd_stat_par[1] += pd_stat_clock() - t0);
}

void p32x_par_leave(void)
{
  par_unpark();
}

// 68k thread, while the 68k polls the 32X: the SH-2s may trail it by up to
// max_lag (the serial scheduler syncs them every line then, which would leave
// one thread waiting for the other all the time). The SH-2s ending the poll
// reach the 68k at its next line once they caught up that far.
void p32x_par_throttle(unsigned int m68k_now, unsigned int max_lag)
{
  PAR_STAT(unsigned int t0 = pd_stat_clock());
  PAR_STAT(pd_stat_par[3]++);
  p32x_par_catch_up(m68k_now, max_lag);
  if (atomic_exchange_explicit(&par_m68k_wake, 0, memory_order_relaxed))
    SekSetStop(0);
  PAR_STAT(pd_stat_par[4] += pd_stat_clock() - t0);
}

// SH-2 thread: run them towards the allowed time; 0 if there was nothing to do
int p32x_par_work(void)
{
  unsigned int req;
  atomic_store_explicit(&par_running, 1, memory_order_seq_cst);
  if (atomic_load_explicit(&par_park, memory_order_seq_cst)) {
    atomic_store_explicit(&par_running, 0, memory_order_release);
    return 0;
  }
  req = atomic_load_explicit(&par_req, memory_order_acquire);
  if (req == atomic_load_explicit(&par_done, memory_order_relaxed)) {
    atomic_store_explicit(&par_running, 0, memory_order_release);
    return 0;
  }
  p32x_par_target = req;
  req = sync_sh2s_normal(req);
  p32x_timer_do(&msh2, req);
  p32x_timer_do(&ssh2, req);
  atomic_store_explicit(&par_done, req, memory_order_release);
  atomic_store_explicit(&par_running, 0, memory_order_release);
  return 1;
}

int p32x_par_pending(void)
{
  return !atomic_load_explicit(&par_park, memory_order_seq_cst) &&
         atomic_load_explicit(&par_req, memory_order_seq_cst) !=
         atomic_load_explicit(&par_done, memory_order_relaxed);
}

// SH-2 thread: the 68k's polling ended; it resumes at its next run
void p32x_par_wake_m68k(void)
{
  atomic_store_explicit(&par_m68k_wake, 1, memory_order_relaxed);
}

// 68k thread, with the SH-2s idle
void p32x_par_start(void)
{
  unsigned int now = SekCyclesDone();
  if (p32x_par_on || !(PicoIn.AHW & PAHW_32X))
    return;
  atomic_store(&par_req, now);
  atomic_store(&par_done, now);
  atomic_store(&par_park, 0);
  p32x_par_on = 1;
}

void p32x_par_stop(void)
{
  if (!p32x_par_on)
    return;
  p32x_par_barrier(SekCyclesDone());
  p32x_par_on = 0;
  atomic_store(&par_park, 0);
}

// at the start of each 68k line: the SH-2s go on (to where the 68k is)
#define PAR_RUN_HOOK() do { \
  if (p32x_par_on) { \
    if (atomic_exchange_explicit(&par_m68k_wake, 0, memory_order_relaxed)) \
      SekSetStop(0); \
    par_unpark(); \
    p32x_par_release(SekCyclesDone()); \
  } \
} while (0)
#ifndef P32X_PAR_MAX_LAG
#define P32X_PAR_MAX_LAG 977 // m68k cycles: two scanlines
#endif
#define PAR_POLL_SYNC(now) do { \
  if (p32x_par_on) \
    p32x_par_throttle(now, P32X_PAR_MAX_LAG); \
  else \
    p32x_sync_sh2s(now); \
} while (0)
#else
#define PAR_RUN_HOOK() do { } while (0)
#define PAR_POLL_SYNC(now) p32x_sync_sh2s(now)
#endif

#define CPUS_RUN(m68k_cycles) do { \
  PAR_RUN_HOOK(); \
  if (PicoIn.AHW & PAHW_MCD) \
    pcd_run_cpus(m68k_cycles); \
  else \
    SekRunM68k(m68k_cycles); \
  \
  if ((Pico32x.emu_flags & P32XF_Z80_32X_IO) && Pico.m.z80Run \
      && !Pico.m.z80_reset && (PicoIn.opt & POPT_EN_Z80)) \
    PicoSyncZ80(SekCyclesDone()); \
  if (Pico32x.emu_flags & (P32XF_68KCPOLL|P32XF_68KVPOLL)) \
    PAR_POLL_SYNC(SekCyclesDone()); \
} while (0)

#define PICO_32X
#define PICO_CD
#include "../pico_cmn.c"

void PicoFrame32x(void)
{
  int offs = (240-rendlines)>>1;

  if (PicoIn.AHW & PAHW_MCD)
    pcd_prepare_frame();

  PicoFrameStart();
  Pico32x.sync_line = 0;
  if (Pico32xDrawMode != PDM32X_BOTH)
    Pico.est.rendstatus |= PDRAW_SYNC_NEEDED;
  DrawLineDest32x = DrawLineDestBase32x + offs * DrawLineDestIncrement32x;
  PicoFrameHints();

  if (!p32x_par_on) { // (else the SH-2 thread keeps them, see p32x_par_work)
    p32x_timer_do(&msh2, Pico.t.m68c_aim);
    p32x_timer_do(&ssh2, Pico.t.m68c_aim);
  }

  elprintf(EL_32X, "poll: %02x %02x %02x",
    Pico32x.emu_flags & 3, msh2.state, ssh2.state);
}

// calculate multipliers against 68k clock (7670442)
// normally * 3, but effectively slower due to high latencies everywhere
// however using something lower breaks MK2 animations
void Pico32xSetClocks(int msh2_hz, int ssh2_hz)
{
  float m68k_clk = (float)(OSC_NTSC / 7);
  if (msh2_hz > 0) {
    msh2.mult_m68k_to_sh2 = (int)((float)msh2_hz * (1 << CYCLE_MULT_SHIFT) / m68k_clk);
    msh2.mult_sh2_to_m68k = (int)(m68k_clk * (1 << CYCLE_MULT_SHIFT) / (float)msh2_hz);
  }
  if (ssh2_hz > 0) {
    ssh2.mult_m68k_to_sh2 = (int)((float)ssh2_hz * (1 << CYCLE_MULT_SHIFT) / m68k_clk);
    ssh2.mult_sh2_to_m68k = (int)(m68k_clk * (1 << CYCLE_MULT_SHIFT) / (float)ssh2_hz);
  }
}

void Pico32xStateLoaded(int is_early)
{
  if (is_early) {
    Pico32xMemStateLoaded();
    return;
  }

  if (CYCLES_GE(sh2s[0].m68krcycles_done - Pico.t.m68c_aim, 500) ||
      CYCLES_GE(sh2s[1].m68krcycles_done - Pico.t.m68c_aim, 500))
    sh2s[0].m68krcycles_done = sh2s[1].m68krcycles_done = Pico.t.m68c_aim;
  p32x_update_irls(NULL, Pico.t.m68c_aim);
  sh2_peripheral_state_loaded();
  p32x_pwm_state_loaded();
  p32x_run_events(Pico.t.m68c_aim);

  // TODO wakeup CPUs for now. poll detection stuff must go to the save state!
  p32x_m68k_poll_event(0, -1);
  p32x_sh2_poll_event(msh2.poll_addr, &msh2, SH2_IDLE_STATES, msh2.m68krcycles_done);
  p32x_sh2_poll_event(ssh2.poll_addr, &ssh2, SH2_IDLE_STATES, ssh2.m68krcycles_done);
}

void Pico32xPrepare(void)
{
  // fallback in case it was missing in saved config
  if (msh2.mult_m68k_to_sh2 == 0 || msh2.mult_sh2_to_m68k == 0)
    Pico32xSetClocks(PICO_MSH2_HZ, 0);
  if (ssh2.mult_m68k_to_sh2 == 0 || ssh2.mult_sh2_to_m68k == 0)
    Pico32xSetClocks(0, PICO_SSH2_HZ);

  sh2_execute_prepare(&msh2, PicoIn.opt & POPT_EN_DRC);
  sh2_execute_prepare(&ssh2, PicoIn.opt & POPT_EN_DRC);
}

// vim:shiftwidth=2:ts=2:expandtab
