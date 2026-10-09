# Externally commanded train speed with per-step energy reporting

## Summary

NeTrainSim needs a mode in which an **external controller commands the train's speed directly**
(for example `30 m/s` or `108 km/h`), rather than commanding a traction notch, and in which the
simulator **reports the energy consumed for that commanded speed at every time step** (`deltaT`).

This request **supersedes the notch/throttle approach** that was proposed in
[PR #3 — "feat: externally commanded train notch (throttle) control"](https://github.com/newuu-se/NeTrainSim-adjusted/pull/3).
PR #3 was **closed without merging**, so none of its code is present on `main`. The requirement
clarified since then is speed-commanded operation: the controller's action space is **physical
speed at a fixed `timeStep`**, and the per-step energy consumption is the signal of interest.

This document is a **feature request and design brief**, not an implementation. No production or
test code is changed by this PR.

## Motivation / what changed

- **PR #3** added externally commanded **notch (throttle)** control: the controller picked a
  discretized notch and the simulator translated that into tractive effort.
- The requirement is now **speed-commanded** operation. Instead of choosing a notch, the
  controller chooses the **target speed**, and the simulator owns the throttle/traction
  computation that realizes it.
- With a speed command, the interesting observable is the **energy consumed in the step just
  simulated** — the controller wants to optimize a speed profile against energy, so it needs a
  reliable per-step energy read-back for the commanded speed.
- An **earlier implementation of this feature already exists in another repository** —
  `data/netrainsim_v2/set_speed.py` on `newuu-se/china-grant-rl-model-2025`, branch `azizbek` (see
  [Prior art](#prior-art)). It sets speed at **file-authoring time**, not per time step, so we
  still need this change; the parts worth reusing are its per-step state JSON and its interactive
  control loop.

## Requested interface (proposal — open to change)

The names below follow the existing conventions of the codebase (camelCase, `SimulatorAPI`
command style, and the RabbitMQ `command` surface). They are a **proposal**, not a dictated
design.

### Command surface

- `setTrainSpeed(trainID, speed)` — set the commanded speed for a single train.
- `setTrainSpeeds(...)` — bulk variant for multiple trains in one call.
- `setTrainSpeedControlEnabled(trainID, enabled)` — explicit **opt-in** flag. Speed control must be
  **off by default**.
- Mirror all of the above on **`SimulatorAPI`** (`src/NeTrainSim/simulatorapi.h` / `.cpp`) and on
  the **server command surface** (`src/NeTrainSimServer/simulationserver.cpp`), consistent with
  existing commands such as `addTrainsToSimulator`, `runSimulator`, and `terminateSimulator`.

### Per-step read-back

For the step just simulated, report at least:

- the **commanded speed** for the step,
- the **achieved speed**,
- **position**,
- **acceleration**,
- **tractive force / tractive power**,
- **energy consumed in that step** — state the unit explicitly (proposed: **kWh**, consistent with
  the battery/tank capacities and conversion factors in `EnergyConsumption`; Wh is acceptable if
  the team prefers finer resolution).

### Plumbing note

`Train::getTotalEnergyConsumption(...)` already computes a **per-step** energy value, and
`Train::getEnergyConsumption(timeStep)` already exists. The work is therefore mainly
**plumbing + a documented per-step delta**, not new physics. The per-step energy reported by the
new surface must **match** `getTotalEnergyConsumption` for that step.

## Physics and safety rules that must be decided and documented

These are the real design questions. They are listed as **open questions** — the implementer and
reviewers should decide and document the answers, not assume them.

1. **Infeasible commands.** A commanded speed can be physically impossible to realize within one
   `deltaT` because of the adhesion limit, maximum tractive power, a power-source shortfall
   (`Train::getMaxProvidedEnergy` / `Train::reducePower`), kinematic acceleration limits, the
   link's free-flow / maximum speed, or braking limits. Decide whether to **clamp** the command, to
   treat it as a **target the operator model tracks**, or to **invert the force equation** for the
   commanded speed.
2. **Definition of "achieved speed".** The step is currently computed as `previousSpeed` plus an
   acceleration (`Train::speedUpDown`). State explicitly whether the commanded speed is the target
   at the **end** of the step or the **average** over it.
3. **Interaction with existing logic.** How does speed control interact with the existing
   car-following / safe-gap logic, signals, dwell / terminals, and the A* throttle optimizer
   (`optimize`)?
4. **Per-train vs per-locomotive.** A consist can have multiple powered units, so a single
   train-level speed command implies a **traction-sharing rule** across locomotives. Decide whether
   the surface is train-level or locomotive-level.
5. **Default path must stay numerically identical** when the feature is disabled — the same
   standard PR #3 was held to.

## Acceptance criteria

- [ ] Opt-in flag (`setTrainSpeedControlEnabled`) exists and defaults to **off**.
- [ ] A commanded speed can be accepted on **every** time step.
- [ ] Per-step energy is reported and **matches** `getTotalEnergyConsumption` for that step.
- [ ] Clamping / safety behaviour is **documented and tested**.
- [ ] Feature **off** produces **bit-identical** results to today's behaviour.
- [ ] Docs updated and **a unit test** covering the new surface is added.

## Where to implement

All references verified against `origin/main`.

### "One time step today" walkthrough

`src/NeTrainSim/simulator.cpp` (around lines 695–730) runs, in order, for each time step:

1. `train->getStepAcceleration(...)`
2. `train->speedUpDown(previousSpeed, stepAcc, timeStep, freeFlowSpeed)`
3. `train->getTractivePower(stepSpd, stepAcc, currentResistanceForces)`
4. `train->getTotalEnergyConsumption(this->timeStep, averageSpd, stepAcc, out.first)`
5. `train->getMaxProvidedEnergy(this->timeStep)` — and, when the step's energy demand exceeds what
   the power source can supply, `train->reducePower(reductionFactor)`
6. `train->moveTrain(...)`

### Speed selection today (operator model, not externally commandable)

- `Train::getNextTimeStepSpeed(gap, minGap, speed, freeFlowSpeed, aMax, T_s, deltaT)` —
  `src/NeTrainSim/traindefinition/train.cpp:669` (car-following / free-flow target with
  jerk-limited `max(speed - mu*dt)` and `min(speed + aMax*dt)` bounds).
- `Train::accelerate(...)` (`train.cpp:805`), `Train::accelerateConsideringJerk(...)`
  (`train.cpp:863`), `Train::speedUpDown(...)` (`train.cpp:882`),
  `Train::adjustAcceleration(...)` (`train.cpp:893`),
  `Train::getAccelerationUpperBound(...)` (`train.cpp:634`).

### Notch / traction / energy

- `Train::updateLocNotch()` — `train.cpp:577` (dispatches to each locomotive with
  `this->currentSpeed`).
- `Locomotive::updateLocNotch(double &trainSpeed)` — `locomotive.cpp:325`;
  `Locomotive::getThrottleLevel(double&, bool&, double&)` — `locomotive.h:266`;
  `Locomotive::getDiscretizedThrottleCoef(double&)` — `locomotive.h:245`;
  `Locomotive::getTractiveForce(...)` — `locomotive.cpp:377`;
  `Locomotive::currentLocNotch` — `locomotive.h:82`.
- `Train::getTotalTractiveForce(...)` — `train.cpp:614`;
  `Train::getTotalResistance(double speed)` — `train.cpp:599`.
- `Train::getEnergyConsumption(double timeStep)` — `train.h:839`;
  `Train::calculateEnergyConsumption(double timeStep, std::string currentRegion)` —
  `train.cpp:1901`;
  `Train::getTotalEnergyConsumption(double& timeStep, double& stepSpeed, double& stepAcceleration, Vector<double>& usedTractivePower)`
  — `train.cpp:1655`; `Train::consumeEnergy(...)` — `train.h:878`;
  `Train::getMaxProvidedEnergy(double& timeStep)` — `train.h:1061` area.
- `EnergyConsumption` (`src/NeTrainSim/traindefinition/energyconsumption.h`) holds drive-line,
  wheel-to-DC-bus and DC-bus-to-tank efficiencies, fuel conversion factors and emissions.

### Integration surfaces

- `SimulatorAPI` singleton — `src/NeTrainSim/simulatorapi.h` / `.cpp` (event-driven, `Mode` enum),
  driven by `src/NeTrainSim/simulatorworker.cpp`.
- CLI — `src/NeTrainSimConsole/main.cpp`.
- Server — `src/NeTrainSimServer/simulationserver.cpp` (RabbitMQ command surface).

**Confirmed absent on `main`:** `git grep -niE 'notchcontrol|setTrainNotch|setTrainSpeed|NTS_JSON' origin/main -- src`
returns nothing — i.e. no speed-control (or notch-control) surface exists today.

## Prior art

An earlier implementation of externally commanded train speed **has been located**:

> **`data/netrainsim_v2/set_speed.py`** on **`newuu-se/china-grant-rl-model-2025`**, branch
> **`azizbek`** (this file exists only on that branch of the 7).

### How it works

- `set_speed.py` rewrites the links file's `FreeFlowSpeed` column, **string-replacing the default
  `19.4`** with a per-distance-band speed limit (`11.11` / `16.67` / `19.44` / `22.22` m/s), and
  writes `data/netrainsim_v2/linksFile_v2_fixed_speed.dat`.
- `rl/train_env.py` points `LINKS_FILE` at that generated file, so the train is forced to the
  chosen speeds **because they are the links' free-flow speeds**.
- Per-step telemetry already exists there: the interactive loop prints `NTS_JSON {…}` per step
  with `speed_mps`, `position_m`, `energy_kwh`, `notch`, `link_max_speed_mps`, and `terminated`.
- `energy_kwh` in that JSON is genuinely **per-step**, not cumulative: it is `Train::energyStat`.
  **We already have this on our `main`** — `this->energyStat = NEC - NER;` and
  `this->cumEnergyStat += this->energyStat;` at `src/NeTrainSim/traindefinition/train.cpp:1914-1915`
  (fields at `train.h:135` and `train.h:137`), with `jsonState["cumEnergyStat"]` at
  `train.cpp:2109`. So per-step energy is **existing plumbing**; the missing piece is only the
  externally commanded speed.

### Why it is not sufficient (why we still need this change)

- Speed is set at **file-authoring time, not per time step**. Changing speed mid-run means
  **regenerating the links file and restarting** the simulation — there is no per-step command.
- The helper is a **string-replace hack**: it assumes the literal default `19.4` and the specific
  50 m link spacing, so it is brittle and network-specific.
- The value it sets is a **link speed limit, not a command**. The operator model can still run
  **slower** than the limit, so the commanded speed is not actually realized.

### What to reuse

- The **per-step state-JSON shape** (`NTS_JSON {…}`) as the read-back format.
- The **interactive stdin/stdout control loop** in `src/NeTrainSimConsole/main.cpp`
  (`--interactive`, accepting `{"notch": N}` and emitting `NTS_JSON {…}`), introduced in commits
  `f511858` and `ce049ec`.

Note explicitly: the **notch-side override** in that repo
(`Locomotive::rlOverrideEnabled` / `rlOverrideThrottle`) is the **ancestor of the closed PR #3
approach** and is **not** what we want. We want the **same plumbing with `speed` as the commanded
quantity**, replacing the notch override.

Speed-file commits in that repo: `740a432` and `495bb5c`.

## Why is this a PR and not an issue?

**GitHub issues are disabled on this repository**, so a pull request is the available channel for
filing this feature request. This PR intentionally contains **documentation only** and changes no
production or test code.
