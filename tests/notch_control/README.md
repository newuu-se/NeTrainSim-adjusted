# Notch-control regression test

`notch_smoke.cpp` is a standalone, assertion-only test that pins down the
behaviour described in [`docs/notch-control.md`](../../docs/notch-control.md).
Its most important assertion is the last one: after `Train::clearNotch()`, the
throttle coefficient `λ` must be **byte-identical** to the value the untouched
default code path produces. That is the guarantee that the published dynamics
and energy models were not perturbed by this extension.

It is deliberately not wired into CMake — running it is opt-in, so a normal
build is unaffected.

## Build and run

Build the core library first:

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_GUI=OFF -DBUILD_SERVER=OFF -DBUILD_INSTALLER=OFF
cmake --build build --target NeTrainSimCore -j4
```

Then compile and run the test (`QT_PREFIX` is the Qt installation prefix):

```sh
QT_PREFIX=$(brew --prefix qt)   # e.g. /opt/homebrew/opt/qt

c++ -std=c++20 -O1 tests/notch_control/notch_smoke.cpp -o /tmp/notch_smoke \
  -I src/NeTrainSim \
  -I build/include \
  -I "$QT_PREFIX/lib/QtCore.framework/Headers" -F "$QT_PREFIX/lib" \
  -L build/src/NeTrainSim -lNeTrainSimCore -framework QtCore \
  -Wl,-rpath,"$PWD/build/src/NeTrainSim" -Wl,-rpath,"$QT_PREFIX/lib"

/tmp/notch_smoke src/data/sampleProject/dieselTrain.dat
```

A different train file can be passed as the argument; the summary line reports
the locomotive's `Nmax` and `maxLocNotch`.

## What it checks

| Group | Assertions |
|---|---|
| Default path | control is off by default; `λ` recorded at 0 / 2.5 / 10 / 22 m/s |
| `N = Nmax` | control turns on; `λ = 1.0` at every speed |
| Lead notch | `getLeadNotch()` equals `getCurrentNotches().front()` (locomotive order is front/leading first) |
| `N = 0` | `λ = 0` at every speed; standstill force is `0 N`, while the uncontrolled standstill force is unchanged (non-zero adhesion limit) |
| `N = Nmax/2` | `λ = (N/Nmax)²` at every speed |
| Clamping | `N + 100` → `Nmax`; `-5` → `0`; powered-off loco reports `0` |
| Restore | `clearNotch()` turns control off and reproduces the recorded baseline exactly |

Exit code is non-zero if any assertion fails. The run also writes a `log.txt`
in the working directory, a side effect of the simulator's own logger.

## Scope note

This test links only against `NeTrainSimCore`, so it exercises the notch path
end to end (train → locomotive → throttle coefficient → tractive force) without
requiring the `Container` / `rabbitmq-c` dependencies that the server target
needs. The server command layer is not covered here.
