# CLAUDE.md

Firmware for Tabs LoRaWAN sensors (WSMS130 board, STM32L071 + SX1276), built on
Basic MAC and Basic Loader. Most applications are ports of the TrackNet
`tabs-*` apps.

## Layout

- `firmware/` — applications, one directory per app:
  - `tabs-door/`, `tabs-pir/`, `tabs-airq/` — production apps (door/window,
    PIR motion, air quality)
  - `doorwindow/` — older simple door/window app
  - `helloworld/` — minimal bring-up app (has its own standalone Makefile)
  - `common/` — code shared by all apps:
    - `tabs.mk` — common build settings; every app Makefile sets `TARGET` and
      includes it
    - `target.mk` — defines the `wsms130` target (MCU, linker script, board
      include, bootloader location)
    - `brd_wsms130.h` — board pin map, battery and power figures (pulled in via
      `BRD_IMPL_INC`)
    - `tabs/` + `tabs*.svc` — Tabs services: `tabs` (payload/config helpers),
      `tabs_bvt` (battery/temperature ADC), `tabs_led`, `tabs_pwrman`
      (capacity accounting on top of `pwrman`), `tabs_pir` (PIR serial config),
      `tabs_sht20`, `tabs_ccs811` (I2C temperature/humidity and eCO2/TVOC
      sensors; they enable I2C1 on PB6/PB7, the same pins the PIR uses)
    - `wsms130.cfg`, `wsms130-flash.cfg` — OpenOCD config (ST-Link, SWD) and
      flash procs; the flash procs override `basicmac/tools/openocd/flash.cfg`
- `bootloader/wsms130/` — builds Basic Loader for the WSMS130 (out-of-tree
  build that points into `basicmac/basicloader`)
- `basicmac/` — git submodule (fork: github.com/RomanMindlin/basicmac). It
  contains `basicmac/basicloader`, a nested submodule. Each has its own
  CLAUDE.md.

## Build

You need `arm-none-eabi-gcc`, Python 3 with the packages from
`basicmac/requirements.txt` and `basicmac/basicloader/requirements.txt`, and
OpenOCD with an ST-Link.

```sh
git submodule update --init --recursive

make -C bootloader/wsms130            # bootloader.hex (rarely changes)

cd firmware/tabs-pir
make                                  # all variants: eu868 us915 hybrid
make VARIANT=eu868                    # a single variant
make clean
```

Output goes to `build-<variant>/<app>.{hex,bin,zfw,delta.up}`. `make` with no
`VARIANT` builds every variant in parallel (`-j8`). Commands that work on one
variant (`load`, `debug`, `setref`) use the first variant unless you set
`VARIANT=`.

Flash and debug (ST-Link):

```sh
make loadbl                 # flash bootloader (bootloader/wsms130/bootloader.hex)
make load VARIANT=eu868     # flash firmware hex
make debug VARIANT=eu868    # gdb attached through openocd
```

The debug console is USART1 (PA9/PA10) at 2000000 8N1. The same UART is used
for personalization (perso): driving RX at reset enters perso mode.

## FUOTA / delta updates

Builds also produce `build-<variant>/<app>.delta.up`, a signed LZ4 delta update
made against `ref-<variant>.zfw` in the app directory:

- The first build seeds `ref-<variant>.zfw` from that build's own `.zfw`.
- After you deploy a build to devices, run `make setref VARIANT=...` to make it
  the new reference.
- `ref-*.zfw` and `build-*` are gitignored. A delta update applies only to the
  exact firmware it was made from, so keep the `.zfw` of every deployed build.

Updates are signed with the **test key** (`SVC_FWMAN_TEST`,
`basicmac/services/fuota/testkey.pem`), set in `tabs.mk`. Updates are sent over
the air with `basicmac/tools/fuota/fuotacs.py` (ChirpStack v4, see its
README).

Memory map (STM32L071xB, 128K flash): the bootloader takes the first 12K at
0x08000000 and the firmware starts at 0x08003000. Free flash after the
firmware holds the fragmentation (update) storage. The EEPROM holds
personalization data, eefs, and the bootloader's update pointer.

## Writing application code

- Read `basicmac/CLAUDE.md` for the runtime model. The short version: it's a
  single-threaded, cooperative job scheduler (`osjob_t`, `os_setCallback`,
  `os_setTimedCallback`). No blocking calls; interrupt handlers do minimal
  work and schedule a job.
- An app is `main.c` + `app.svc`. `app.svc` lists the services it `require`s
  and binds the app's functions to hooks: `hook.appstart` (entry,
  `app_main`), `hook.lwm_downlink`, `hook.uexti_irq`, `hook.eefs_init`,
  `hook.eefs_fn`. `svcdefs.h` is generated per build; include it so hook
  signatures are type-checked.
- Apps that use `uexti` must set `LMICCFG += EXTI_IRQ_HANDLER=uexti_handler`
  in their Makefile.
- Uplinks go through lwmux: `lwm_request_send(&lwm_job, prio, txfunc)`. The
  txfunc fills `lwm_txinfo` (data, dlen, port) and returns `false` if the data
  does not fit `txinfo->dlen`.
- Persistent config uses eefs keyed by a 12-byte UFID. Generate new UFIDs with
  `basicmac/services/eefs/ufid.py`; never reuse or change an existing one.
- Config downlinks on port 204 (`TABS_PORT_CONFIG`) are TLV, little-endian.
  Tag values are **append-only**: never renumber tags, in either the shared
  `TABS_CFGTAG_*` (128+) or the per-app tags (0+). After a config change the
  device uplinks its full configuration on port 204.
- Uplink ports and payload encodings (`tabs_enc_batt`, `tabs_enc_temp`)
  follow the Tabs payload specification; keep them compatible with existing
  decoders.
- Board-specific constants (GPIOs, battery mAh, quiescent current) belong in
  `brd_wsms130.h`, guarded by `SVC_*` where they depend on a service.
- Code style follows Basic MAC: 4-space indent, `if( cond ) {`, `snake_case`,
  `debug_printf` for logging (`%F` fixed-point, `%E` EUI, `%h` hex dump).

## Testing

The `simul` variant (Unicorn emulator) is commented out in `tabs.mk`, so the
Tabs apps can't be tested without hardware. To check a change, build every
variant (`make`); the Makefiles build with `-Wall`. Upstream examples under
`basicmac/projects/ex-join` still run in simulation (`VARIANT=simul make
test`).

## Submodule workflow

`basicmac` tracks the `master` branch of the fork, and `basicloader` tracks the
`master` branch of its fork. To change them: commit inside the submodule, push
it, then commit the updated submodule pointer here (for basicloader, first in
basicmac, then here).
