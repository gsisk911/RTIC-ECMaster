# Teensy EtherCAT bridge — LinuxCNC (Raspberry Pi) integration

A realtime LinuxCNC HAL component that drives the Teensy EtherCAT master over
SPI. The Teensy runs the hard-real-time EtherCAT cyclic loop, DC/SYNC0, CiA-402
sequencing, and the safe-state; LinuxCNC does trajectory/kinematics/PID and
exchanges process data with the Teensy once per servo cycle.

See the design + firmware side in [`../docs/linuxcnc-spi-bridge.md`](../docs/linuxcnc-spi-bridge.md).

## Files

| File | Purpose |
| --- | --- |
| `teensy_bridge_layout.h` | **Generated** frame + pin contract (`make config`). Single source of truth shared with the firmware — never hand-edit. |
| `teensy_ecat_bridge.c` | The realtime HAL component (spidev transport, dynamic pins from the header, motion stream + flow control). |
| `teensy_ecat_bridge.hal` | Example HAL wiring (wires `drive0` only; the committed firmware bus is two drives, `drive0` / `drive1`). |

## Wiring (hardware)

Teensy LPSPI3 (slave) ⟷ Pi SPI0 (master), on a shared PCB/Pi-HAT:

| Signal | Teensy pin (default) | Pi (SPI0) |
| --- | --- | --- |
| SCK   | 27 (GPIO_AD_B1_15) | SCLK (GPIO11) |
| MOSI  | 39 (SDI)           | MOSI (GPIO10) |
| MISO  | 26 (SDO)           | MISO (GPIO9)  |
| CS    | 38 (PCS0)          | CE0  (GPIO8)  |
| FRAME_READY | 25 (out)     | a free GPIO input (optional) |
| GND   | GND                | GND |

Teensy pins are configurable in [`../.cargo/config.toml`](../.cargo/config.toml)
(`HOST_SPI_*`). Enable SPI0 on the Pi (`dtparam=spi=on`).

FRAME_READY **toggles once per completed frame** (an edge strobe, not a level):
the host can edge-trigger reads or detect a stall (the line stops toggling). Leave
a normal inter-frame gap (deassert CS between transactions) — the servo-rate
cadence provides this — so the Teensy re-arms cleanly between frames; a frame that
races the re-arm is rejected by the CRC and recovered on the next one.

## Build / install (on the Pi)

```sh
cd linuxcnc
halcompile --install teensy_ecat_bridge.c   # needs teensy_bridge_layout.h on the include path
```

Regenerate the contract whenever the bus XML changes (run on the dev machine,
commit the result):

```sh
make config        # writes src/.../generated.rs, src/hal/spi_layout_generated.rs,
                   # and linuxcnc/teensy_bridge_layout.h
```

## Load (HAL)

```hal
loadrt teensy_ecat_bridge device=/dev/spidev0.0 spi_hz=40000000 lead=10
addf teensy-ecat.update servo-thread
source teensy_ecat_bridge.hal     # the wiring above
```

`lead` is the motion look-ahead depth (cycles). It must exceed the Pi's
worst-case servo-thread latency spike in cycles. Start at 10 and tune against
`teensy-ecat.buffer-depth`.

## HAL pins

- Process data: one pin per `halPin` name, across **all** drives (`drive0-*`,
  `drive1-*` for the committed two-drive bus), e.g.
  `teensy-ecat.drive0-target-position` (HAL_IN, host→drive) and
  `teensy-ecat.drive0-actual-position` (HAL_OUT, drive→host). Types: bit/u32/s32 per
  the XML `halType`.
- Intent (HAL_IN): `enable`, `fault-reset`, `quick-stop`. These are applied
  **bus-wide** — every drive shares one intent (a per-drive host enable is a
  deferred follow-up).
- Status (HAL_OUT): `online`, `link`, `operational`, `fault`, `host-timeout`,
  `wkc`, `expected-wkc`, `phase`, `cycle-index`, `buffer-depth`, `crc-errors`.

## Bring-up & validation

Do this incrementally; cross-check from the Teensy USB console with the `host`
command (link, watchdog, CRC/seq errors, buffer depth) and `stats`/`status`.

1. **Transport.** Load the component; confirm `teensy-ecat.online` goes true and
   `crc-errors` stays 0 (`halcmd show pin teensy-ecat`). On the Teensy, `host`
   should show `link=on frames=...` incrementing and `crc_err=0`.
2. **Feedback.** With the drive powered (not enabled), jog nothing; confirm
   `teensy-ecat.drive0-actual-position` tracks the drive and `wkc == expected-wkc`.
3. **Enable / fault-reset.** Pulse `fault-reset`, then set `enable`; confirm
   `operational` goes true and the drive(s) reach Operation-Enabled (statusword).
   The Teensy owns the controlword — you only assert intent, and it currently
   applies **bus-wide** to every drive.
4. **Position follow.** Wire `motor-pos-cmd`→target and command a slow move;
   confirm `actual-position` follows with acceptable following error.
5. **Host-watchdog trip.** Stop the component (`halcmd stop` / unload). The host
   heartbeat stalls; the Teensy must drive the axis to **quick-stop** within the
   watchdog window (`HOST_WDOG_LIMIT_CYCLES`). Verify on the Teensy `status`
   (phase/fault) and that the drive ramps to a controlled stop.
6. **Underrun (motion stream only).** With a configured `<motionStream>`, force a
   Pi latency spike larger than `lead` cycles; confirm `buffer-depth` hits 0 and
   the Teensy issues a quick-stop (`host` shows the underrun fault flag).

> Per the project rule, the operator flashes/runs the firmware; do not start it
> unexpectedly. Validate each step before proceeding.
