# EtherNetIP reference

Public headers are under `lib/EtherNetIP/src/`. Units below are the ones in those headers. The motion math is `LOW_LEVEL_GANTRY_CONTROL.md`, not this page.

## Layers

| Header | What it holds |
|---|---|
| `EipEncapsulation.h`, `EipCpf.h`, `CipMessageRouter.h`, `EipConnectionManager.h` | Codecs. RPI and the actual packet interval are microseconds |
| `Kinetix5100Assembly.h` | Output 104 and input 154. `speed_reference` is commented 0.1 RPM (`:62`); `actual_speed` is commented RPM (`:111`) |
| `Hcs01Assembly.h`, `Hcs01ControlStatus.h` | HCS01 instances 101 and 102 |
| `EipSession.h` | Explicit messages. Default timeout 500 ms. `kMaxFrameSize` is 512 |
| `EipMultiScanner.h` | Production scanner: open, drain, exchange, reopen |
| `EipClass1Timing.h` | Pace, stale policy, `ExchangeStatus` |
| `EipScannerTask.h` | `startScannerTask`, `pauseClass1`, `resumeClass1`, `dumpClass1TimingStats`, `printLastClass1GapReport` |

`multiAxisTask` calls `multiAxisConnectAndRun`, which calls `multiAxisCyclic`, which calls `multiAxisRecover` (`EipScannerTask.cpp`).

## Tasks

| Task | Core | Priority | Stack | Where |
|---|---:|---:|---:|---|
| `EipScannerM` | 1 | 6 | 8,192 | `kScannerCore`, `kScannerPriority`, `kScannerStack` |
| `EipHoldKA`, `EipReopenKA`, `EipCloseKA` | 1 | 7 | 4,096 | `kKeepalivePriority`, `kKeepaliveStack` |
| `EipLog` | 0 | 2 | 4,096 | `kLogTaskCore`, `kLogTaskPriority`, `kLogTaskStack` |

Hold keepalive period is 5 ms. Reopen keepalive is `kReopenKeepalivePeriodMs` = 2 (`EipScannerTaskInternal.h:105`).

## Timing the code uses

| Constant | Value | Line |
|---|---|---|
| `kReconnectIdleMs` | 500 ms | `EipScannerTaskInternal.h:70` |
| `kLinkSettleMs` | 300 ms | `EipScannerTaskInternal.h:69` |
| `kPostResetSettleMs` | 100 ms | `EipScannerTaskInternal.h:71` |
| soft T→O misses | 2 (`shouldTeardownAfterInputMisses` default) | `EipClass1Timing.h:71–73` |
| `kClass1StaleTeardownRpiMultiples` | 10 | `EipClass1Timing.h:79` |
| `kClass1StaleGateGraceRpiMultiples` | 3 | `EipClass1Timing.h:81` |
| `kClass1ConnectionTimeoutMultiplier` | 2 (CIP timeout = RPI × 4 << code) | `EipScannerPolicy.h:16–19` |
| `kMaxChipRecovers` | 3 | `EipScannerPolicy.h:12` |
| chip-recover backoff | start 1,000 ms, double, cap 5,000 ms | `EipScannerPolicy.h:13–14` |
| overrun yield | every 8 consecutive overruns | `EipClass1Timing.h:50` |

After `kMaxChipRecovers`, `escalateChipRecover` waits 10 s, clears the streak and retries (`EipScannerTaskInternal.h:347–353`). It does not call `esp_restart`. The comment at `EipScannerTask.h:7` still says `esp_restart`.

`ExchangeStatus` (`EipClass1Timing.h:119–123`): `kOk`, `kOutputSendFailed` (chip recover), `kInputMiss` (soft retry).

## Locks the Class 1 path takes

The W5500 bus mutex, the SPI3 Class 1 critical section (`spi3_class1_critical_enter` in `EipScannerTask.cpp`), and the process-image mutex inside `sendAxisOutput` / `applyFeedback`. The process-image mutex is also taken from `Gantry` on other tasks. G-B17. Socket logs inside `exchangeOnce` are G-B16.

## AppConfig

This component does not read the stored target IP, RPI or PUU keys. `hcs01_eng_ip` is not read; COMWS uses `kDefaultEngIp` (`Hcs01ComwsClient.h`). G-B5.

## Console

| Command | Call |
|---|---|
| `eiptiming` | `dumpClass1TimingStats` |
| `gaptrace` | `printLastClass1GapReport` |
| `cfg save` | `pauseClass1` (`src/gantry_console_cfg.cpp:97`) |

`ethlink` and `drv` are console commands in `src/gantry_console_commands.cpp` whose handlers use this library.

## Errors

`ExchangeStatus` above is the result of one Class 1 exchange. `pauseClass1` returns false when Class 1 does not close within its timeout, and `cfg save` then does not save (`src/gantry_console_cfg.cpp:97`). `startScannerTask` returns nothing.

## Usage

```cpp
// src/main_init_motion.cpp, once w5500.init() has succeeded
eip::startScannerTask(w5500, w5500Hal, w5500LinkStatus, &eipImageX, &eipImageZ, &eipImageTheta);
```

## Documents

`LOW_LEVEL_GANTRY_CONTROL.md` sections 5 and 6. Open items: G-A14, G-B5, G-B8, G-B9, G-B16, G-B17, G-B19 in `docs/OPEN_ITEMS.md`.
