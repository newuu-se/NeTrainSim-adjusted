# External Notch (Throttle) Control

This document describes the notch-control extension added in this fork so
NeTrainSim can be driven by an external notch controller: a controller outside
the simulator sets the throttle notch of a train, advances the simulation by one
time step, and reads back the resulting state.

It is the local implementation of the feature requested upstream in
[VTTI-CSM/NeTrainSim#42](https://github.com/VTTI-CSM/NeTrainSim/issues/42).

## Design contract

The published train-dynamics and energy-consumption models are **not modified**.

The throttle coefficient `λ` (called `getThrottleLevel` /
`getDiscretizedThrottleCoef` in the code) is the only quantity governed by this
extension, and it is exactly the quantity the simulator already varies
internally. A commanded notch maps onto the same discretized set the original
code uses:

```
λ = (N / Nmax)²
```

where `N` is the notch and `Nmax` is the locomotive's number of notch positions
(`Locomotive::Nmax`, 8 for the sample diesel locomotive). Everything downstream
of `λ` is untouched: acceleration, resistance, tractive-power, fuel/energy, and
CO₂ accounting still derive from actual motion (`mass·a + resistance`), never
from the notch number. Adhesion-based force capping, the energy-shortfall power
reduction, and `reducePower` continue to apply.

**Backward compatibility.** Notch control is *opt-in per train*. Until it is
enabled, the simulator takes its original code paths and produces
numerically identical results.

## Notch semantics

| Aspect | Behaviour |
|---|---|
| Notch range | `0 .. min(maxLocNotch, Nmax)`, or `0 .. Nmax` when `maxLocNotch` is unset |
| Clamping | Out-of-range values are clamped, never rejected |
| `N = 0` | `λ = 0` → no tractive effort requested |
| `N = Nmax` | `λ = 1` → full tractive effort |
| Powered-off loco | Reports and applies notch `0` |
| Standstill, `N = 0` | Tractive force is exactly `0 N` (see below) |
| A\* optimization | Bypassed while notch control is on — the command is authoritative |

### Notch 0 at standstill

In the original code `Locomotive::getTractiveForce` returns the adhesion limit
`μ·m·g` whenever `trainSpeed == 0`, regardless of notch. That is correct for
the simulator's own operator model but wrong for an external controller's
action space, where "notch 0" must mean "no tractive effort". When notch
control is enabled, a commanded `λ ≤ 0` at `trainSpeed == 0` returns `0 N`
before the adhesion calculation. The default path (notch control off) keeps the
original adhesion-limit behaviour.

## C++ API

### `Train`

```cpp
#include "./traindefinition/train.h"

train->setNotch(int notch);                    // same notch on every locomotive
train->setNotches(const Vector<int> &notches); // one notch per locomotive
train->clearNotch();                           // hand control back to the simulator
bool ok        = train->hasNotchControl();     // is external control active?
int  n         = train->getLeadNotch();        // leading locomotive's notch
Vector<int> ns = train->getCurrentNotches();   // one entry per locomotive
```

`setNotches` applies `min(notches.size(), locomotives.size())` entries; trailing
locomotives keep their previous command. Locomotive order is **front (leading)
locomotive first, rear last**, matching the way `Train::rearrangeTrain()` builds
the consist, so `getLeadNotch()` equals `getCurrentNotches().front()`. A consist
can hold different notches per locomotive, so `getLeadNotch()` alone is not a
train-wide value — use `getCurrentNotches()` when that matters. The notch is
idempotent per step — `updateLocNotch` runs twice per simulation step, and the
commanded value is re-applied both times.

### `SimulatorAPI`

Five static helpers exist on both `SimulatorAPI::InteractiveMode` and
`SimulatorAPI::ContinuousMode`:

```cpp
static bool        setTrainNotch(QString networkName, QString trainID, int notch);
static bool        setTrainNotches(QString networkName, QString trainID, QVector<int> notches);
static bool        setTrainNotchControlEnabled(QString networkName, QString trainID, bool enabled);
static QJsonObject getTrainState(QString networkName, QString trainID);
static QJsonArray  getNetworkTrainStates(QString networkName);
```

The `set*` calls return `false` when the train is not found in the network.
`getTrainState` returns an empty object in that case.

## Train state JSON

`Train::getCurrentStateAsJson()` gained three fields:

```json
{
  "notch": 4,
  "notchControlOn": true,
  "notches": [4, 4]
}
```

`notch` and `notches` always hold the notch actually in effect, whether it was
commanded externally or chosen by the simulator's operator model. `notch` is the
leading locomotive only (see above); `notches` is ordered front to back. All
pre-existing fields are unchanged.

## Server commands

The server exchanges messages over RabbitMQ using the same routing as the rest
of the command set:

| Constant | Value |
|---|---|
| Exchange | `CargoNetSim.Exchange` |
| Command queue | `CargoNetSim.CommandQueue.NeTrainSim` |
| Response queue | `CargoNetSim.ResponseQueue.NeTrainSim` |
| Incoming routing key | `CargoNetSim.Command.NeTrainSim` |
| Outgoing routing key | `CargoNetSim.Response.NeTrainSim` |

Command parameters are read from either the `params` object or the message
root. An optional `commandId` is echoed back on every response.

### Request `includeTrainStates`

Passed on the `runSimulator` command. When `true`, every `simulationAdvanced`
event carries the state of all trains on all simulated networks, so a
step-based controller gets its observation in the same round trip instead of a
separate `getTrainState` call.

```json
{ "command": "runSimulator", "byTimeSteps": 1, "includeTrainStates": true }
```

### Command `setTrainNotch`

Sets the notch, or turns external notch control off. At least one of `notch`,
`notches`, or `enable` is required.

```json
{ "command": "setTrainNotch",
  "networkName": "network1", "trainID": "Train1", "notch": 4 }
```

```json
{ "command": "setTrainNotch",
  "networkName": "network1", "trainID": "Train1", "notches": [4, 5] }
```

```json
{ "command": "setTrainNotch",
  "networkName": "network1", "trainID": "Train1", "enable": false }
```

`enable: true` alone is a no-op for the notch value; pass it with `notch` or
`notches` to (re-)enable control. `enable: false` calls `clearNotch()` and
restores the simulator's own controller.

Response event `trainNotchSet`:

```json
{ "event": "trainNotchSet", "host": "NeTrainSim", "success": true,
  "networkName": "network1", "trainID": "Train1",
  "notch": 4, "notches": [4, 4], "notchControlOn": true }
```

### Command `getTrainState`

```json
{ "command": "getTrainState", "networkName": "network1", "trainID": "Train1" }
```

Response event `trainState` — `state` is the full `getCurrentStateAsJson()`
object:

```json
{ "event": "trainState", "host": "NeTrainSim", "success": true,
  "networkName": "network1", "trainID": "Train1",
  "state": { "notch": 4, "notchControlOn": true, "notches": [4, 4], "...": "..." } }
```

Errors are reported through the server's existing `onErrorOccurred` path
(missing fields, unknown network or train).

## External control loop

A reinforcement-learning agent is only one possible consumer; the loop below is
generic. There is no "advance one simulation step" primitive. `runSimulator`
blocks for `byTimeSteps` seconds of simulated time and then emits
`simulationAdvanced`. The control loop is therefore:

1. `runSimulator` with `byTimeSteps: <dt>` and `includeTrainStates: true`
2. read `trainStates` off the `simulationAdvanced` event → observation, reward
3. `setTrainNotch` with the chosen action
4. repeat

Minimal Python sketch (`pika`):

```python
import json, pika

EXCHANGE = "CargoNetSim.Exchange"
CMD_KEY  = "CargoNetSim.Command.NeTrainSim"
RESP_KEY = "CargoNetSim.Response.NeTrainSim"

conn = pika.BlockingConnection(pika.ConnectionParameters("localhost"))
ch = conn.channel()
ch.exchange_declare(exchange=EXCHANGE, exchange_type="topic")
q = ch.queue_declare("", exclusive=True).method.queue
ch.queue_bind(exchange=EXCHANGE, queue=q, routing_key=RESP_KEY)

NMAX = 8
DT = 1  # simulated seconds per step

def command(payload):
    ch.basic_publish(exchange=EXCHANGE, routing_key=CMD_KEY,
                     body=json.dumps(payload).encode())

def step(action):
    """Run one simulated second, then apply `action` (a notch)."""
    command({"command": "runSimulator",
             "byTimeSteps": DT,
             "includeTrainStates": True})
    # consume until the simulationAdvanced event arrives
    for method, _, body in ch.consume(q, inactivity_timeout=1):
        if body is None:
            continue
        msg = json.loads(body)
        if msg.get("event") == "simulationAdvanced":
            break
    states = msg["trainStates"]["network1"]
    obs = states[0]
    reward = -obs.get("currentUsedTractivePower", 0.0)
    command({"command": "setTrainNotch",
             "networkName": "network1", "trainID": obs["trainID"],
             "notch": int(action)})
    return obs, reward

# main loop
command({"command": "setTrainNotch", "networkName": "network1",
         "trainID": "Train1", "notch": 0})          # take control at notch 0
obs, reward = step(NMAX // 2)
while episode_running():
    action = agent(obs, reward)                     # agent returns 0..NMAX
    obs, reward = step(action)
```

Consume the `trainState` / `trainNotchSet` acknowledgements that
`setTrainNotch` and `getTrainState` publish, or route them to a separate queue,
so they do not interleave with the step observations.

## Verification status

* The core library builds and a behavioural test of the notch path passes for
  the bundled sample train (`src/data/sampleProject/dieselTrain.dat`): the
  commanded `λ = (N/Nmax)²` holds at every speed, clamping and powered-off
  handling behave as documented, notch 0 at standstill yields `0 N` instead of
  the `478934 N` adhesion limit, and after `clearNotch()` the coefficient
  sequence is byte-identical to the untouched default path. The test is checked
  in at [`tests/notch_control/notch_smoke.cpp`](../tests/notch_control/notch_smoke.cpp);
  build and run instructions are in that directory's README.
* The server translation unit was compile-checked against stub headers for the
  non-public `Container` and `rabbitmq-c` dependencies. A build against the real
  libraries is still advisable before production use.

## Upstream issue

The feature was requested upstream in
[#42](https://github.com/VTTI-CSM/NeTrainSim/issues/42) and is assigned to the
`v0.1.5` milestone. This fork carries the change so it can be used now; the
design above is intended to be upstreamable as-is.
