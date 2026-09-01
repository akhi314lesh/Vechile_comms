# Incrementally Refactor `index.html` Into a Risk-Triggered V2V Control Architecture

You are modifying the **current project state in `index.html` only**.

## HARD SCOPE RULE

- Work on `index.html` only.
- Do NOT modify, copy from, merge with, or otherwise use `index2.html` or `index3.html`.
- Other existing project files may be inspected only when necessary to understand how `index.html` references them, but the implementation changes in this task must be confined to `index.html`.
- Preserve the existing simulation, Three.js scene, vehicle physics, V2V communication, UI, local sensing, collision system, and existing modes unless a change is required for the architecture below.
- Do NOT rewrite the file from scratch.
- Do NOT remove working functionality just to simplify the code.
- Introduce the new architecture incrementally and keep the simulation runnable after every phase.

---

# GOAL

Refactor the current V2V Assist system so that it behaves as a **risk-triggered autonomous safety system**, not as a continuously autonomous driver.

The desired behavior is:

### Normal / low-risk situation
The vehicle should behave normally.

Do NOT continuously let V2V Assist autonomously:
- steer toward the lane center,
- change acceleration,
- brake,
- or change lanes

unless the existing normal-driving behavior explicitly requires it.

### High collision risk
When the system determines that collision risk is sufficiently high, autonomous intervention is allowed.

The system may then decide:
- how much to steer,
- how much to accelerate/decelerate,
- how much to brake,
- whether a lane change is necessary.

### Critical collision risk
Emergency control gets highest authority.

The system must be able to:
- brake strongly,
- perform emergency steering,
- perform an emergency lane change when a safe escape path exists,
- otherwise remain on a braking trajectory.

IMPORTANT TERMINOLOGY:

Use **low TTC = high collision risk**.

Do not implement or document the logic as "high TTC means high risk."

---

# ARCHITECTURAL PRINCIPLE

The current system contains several intelligent controllers such as:

- V2V perception
- local perception
- TTC
- ACC / IDM
- stopping-distance logic
- curvature speed governor
- lane keeping
- lane changing
- AEB
- evasive steering
- safety shield
- vehicle physics

The problem is not that all of these components must be deleted.

The problem is that too many of them can influence final vehicle control without one explicit decision/arbitration layer.

Refactor toward:

```text
LOCAL SENSING
      +
V2V BSM
      ↓
UNIFIED PERCEPTION / TRACK ASSOCIATION
      ↓
THREAT EVALUATION
      ↓
AUTHORITATIVE TTC + COLLISION RISK
      ↓
RISK STATE / DECISION
      ↓
CONTROL PROPOSALS
      ├── longitudinal proposal
      ├── lane keeping proposal
      ├── lane change proposal
      └── emergency avoidance proposal
      ↓
CENTRAL CONTROL ARBITRATION
      ↓
FINAL STEER / THROTTLE / BRAKE
      ↓
VEHICLE PHYSICS
```

The key rule is:

> Individual controllers should propose actions. The arbitrator decides what actually reaches the vehicle.

Do not let multiple controllers silently overwrite final controls.

---

# EXECUTION METHOD

Implement this in the exact phases below.

After each phase:

1. Verify the code still runs.
2. Preserve existing behavior not related to the phase.
3. Do not start the next phase until the current phase is structurally integrated.
4. Prefer small helper functions and explicit data objects over a giant rewrite of `computePolicy()`.

---

# PHASE 1 — Build the unified threat evaluation layer

First create a dedicated threat evaluation abstraction without changing the vehicle's behavior yet.

Create something conceptually similar to:

```js
evaluateThreats(...)
```

Its input should primarily be:

```js
SENS.perceived
```

plus the ego vehicle state.

Its output should contain:

```js
{
    threats: [...],
    highestRiskThreat: ...,
    riskLevel: ...
}
```

Each threat should contain enough information for every later controller to use the same interpretation of that vehicle, including:

```js
{
    id,
    source,              // LOCAL / V2V / FUSED
    wx,
    wz,
    distance,
    relativeSpeed,
    closingSpeed,
    lateralOffset,
    lane,
    heading,
    ttc,
    collisionRisk,
    trajectoryRisk,
    v2vAge
}
```

Do not create a second perception pipeline.

Use the existing perception system and build the threat evaluator on top of it.

At this phase, the evaluator may be diagnostic only. Do not yet radically change steering, throttle, or braking.

---

# PHASE 2 — Make TTC authoritative and eliminate inconsistent TTC logic

Now make one TTC implementation authoritative for the controller.

The problem currently is that different parts of the code derive TTC differently.

Refactor so that:

```text
Threat Evaluator
      ↓
authoritative TTC
      ↓
ACC / AEB / lane-change safety / diagnostics / HUD / minimap
```

Do not allow the minimap or another controller to invent its own TTC formula.

The authoritative threat calculation should account for:

- relative motion
- actual ego/target geometry
- whether the target is actually on a collision-relevant trajectory
- closing speed
- lane relationship
- lateral offset

Where practical, make predicted collision reasoning consistent with the existing OBB collision geometry used by the simulation.

Do not blindly replace every distance calculation in the project. Replace calculations that are acting as an alternative definition of collision risk.

---

# PHASE 3 — Select the highest-risk threat, not simply the nearest vehicle

Refactor lead/threat selection.

Do not assume:

```text
nearest object = most dangerous object
```

Instead evaluate all relevant perceived objects and rank their collision risk.

The ranking should consider, as appropriate:

- TTC
- closing speed
- projected trajectory
- lane overlap
- lateral offset
- heading
- collision geometry
- source/freshness confidence

Example:

```text
Vehicle A:
distance = 8m
TTC = 8s

Vehicle B:
distance = 16m
TTC = 1.8s
```

Vehicle B should be capable of becoming the primary threat even though it is farther away.

Create one authoritative:

```js
highestRiskThreat
```

and make later control logic consume that result.

---

# PHASE 4 — Add explicit risk states and hysteresis

Introduce explicit risk states:

```text
NORMAL
CAUTION
HIGH
CRITICAL
```

Use the authoritative threat/TTC data.

The exact thresholds should be defined as named constants so they can be tuned later.

Do NOT scatter numeric TTC thresholds throughout the file.

For example, structure the code around named thresholds such as:

```js
TTC_HIGH_ENTER
TTC_HIGH_EXIT
TTC_CRITICAL_ENTER
TTC_CRITICAL_EXIT
```

Use hysteresis so the state does not rapidly oscillate:

```text
NORMAL
HIGH
NORMAL
HIGH
NORMAL
```

when TTC fluctuates around one threshold.

The risk state should become persistent state, not simply a temporary comparison.

Also add recovery logic:

```text
CRITICAL → HIGH → CAUTION → NORMAL
```

when the threat is actually clearing.

Do not immediately drop from emergency control to normal control on the first safe-looking frame.

---

# PHASE 5 — Add unified local/V2V track association

Improve the existing perception representation so local and V2V observations of the same vehicle can be associated.

Desired conceptual model:

```text
LOCAL observation
       +
V2V observation
       ↓
same physical vehicle
       ↓
one track
       ↓
source = FUSED
```

Support:

```text
LOCAL
V2V
FUSED
```

as the meaningful source classification.

Do not allow one physical vehicle to become two independent threats merely because it is seen by both sources.

Maintain V2V freshness/age.

A stale V2V message must not be treated as if it were a current observation.

Preserve useful existing `SENS.perceived` behavior rather than replacing the entire perception system.

---

# PHASE 6 — Convert existing controllers into control proposals

This is the first major control-architecture change.

Do NOT immediately delete the existing controllers.

Instead wrap/refactor them conceptually so they generate proposals.

For example:

```js
lkaProposal
accProposal
aebProposal
laneChangeProposal
emergencyAvoidanceProposal
```

A proposal should contain enough information to explain itself.

For example:

```js
{
    active: true,
    steer: ...,
    reason: ...,
    priority: ...
}
```

and for longitudinal control:

```js
{
    active: true,
    targetSpeed: ...,
    throttle: ...,
    brake: ...,
    reason: ...
}
```

Do NOT let every controller directly write the final `ctrl.steer`, `ctrl.throttle`, and `ctrl.brake`.

The existing physics system should remain the consumer of final controls.

---

# PHASE 7 — Introduce the central control arbitrator incrementally

Create a central function conceptually similar to:

```js
arbitrateControl(...)
```

The arbitrator must become the only layer that produces the final control command.

Conceptually:

```text
controller proposals
        ↓
control arbitrator
        ↓
finalControl
        ↓
physics
```

The arbitrator must explicitly understand control priority.

Use this conceptual authority order:

```text
1. CRITICAL EMERGENCY AVOIDANCE
2. AEB / EMERGENCY BRAKING
3. HIGH-RISK COLLISION AVOIDANCE
4. EMERGENCY LANE CHANGE
5. NORMAL DRIVER LANE REQUEST
6. LANE KEEPING
7. ACC / SPEED GOVERNOR
8. NORMAL DRIVING
```

Do NOT interpret this as requiring one subsystem to disable all others all the time.

Lateral and longitudinal controls may coexist when they do not conflict.

The important point is that final authority is explicit.

Examples:

```text
ACC requests braking
AEB requests stronger braking
→ AEB wins

LKA requests steering
Emergency avoidance requests steering
→ Emergency avoidance wins

Lane keeping requests centerline steering
Lane change is active
→ lane change owns lateral control
```

Add an explicit reason/authority field for debugging.

---

# PHASE 8 — Make V2V Assist risk-triggered rather than continuously autonomous

Now change the behavior of V2V mode.

Do NOT have V2V Assist continuously behave like a fully autonomous driver.

The logic should be conceptually:

```text
V2V MODE
   ↓
Threat Evaluation
   ↓
Risk State
```

### NORMAL

Do not unnecessarily override normal driving.

Do not continuously inject autonomous:

- steering
- acceleration changes
- braking
- lane changes

### CAUTION

Continue monitoring.

Warnings/diagnostics are allowed.

Do not perform unnecessary autonomous maneuvers.

### HIGH

Collision avoidance authority becomes available.

The system can decide:

- reduce speed
- brake
- steer
- prepare/execute avoidance
- evaluate an escape lane

based on actual risk.

### CRITICAL

Emergency control receives highest authority.

The system may take full control necessary to avoid collision.

This is the central behavioral requirement of the project.

---

# PHASE 9 — Separate ACC from AEB

Refactor longitudinal control so ACC is a recommendation rather than the ultimate safety authority.

Desired conceptual architecture:

```text
ACC / IDM
    ↓
target speed recommendation

AEB
    ↓
emergency braking proposal

Arbitrator
    ↓
final throttle/brake
```

ACC must never be able to counteract AEB.

Example:

```text
ACC:
target speed = 14 m/s

AEB:
brake = 1.0

ARBITRATOR:
AEB wins

FINAL:
throttle = 0
brake = 1.0
```

Keep the existing IDM/stopping-distance/PID logic where useful.

The goal is not to remove it; the goal is to define its authority.

---

# PHASE 10 — Separate LKA from lane-change authority

Refactor lateral control so that lane keeping and lane changing have explicit ownership.

Desired behavior:

```text
KEEP_LANE:
    LKA owns lateral guidance

LANE_CHANGE:
    lane-change controller owns lateral guidance

EMERGENCY AVOIDANCE:
    emergency avoidance owns lateral guidance
```

LKA must not fight the lane-change controller.

When a lane change is active, the controller must explicitly know that LKA is subordinate or operating toward the lane-change trajectory rather than the original lane center.

Avoid two independent steering writers.

---

# PHASE 11 — Convert autonomous lane changing into risk-driven emergency lane changing

This is critical.

The existing lane utility system must not automatically change lanes merely because another lane appears preferable when V2V Assist is supposed to be risk-triggered.

Separate:

```text
NORMAL lane change intent
```

from:

```text
EMERGENCY collision-avoidance lane change
```

For autonomous V2V safety behavior:

```text
high collision risk
      ↓
can braking alone solve it?
      ↓
YES → brake/decelerate
NO
      ↓
evaluate adjacent lanes
      ↓
safe escape lane exists?
      ↓
YES → emergency lane change
NO  → emergency braking
```

Evaluate target lanes using the same unified perception and authoritative threat data.

Consider:

- front vehicle
- rear vehicle
- target-lane TTC
- relative velocity
- lane occupancy
- trajectory
- V2V freshness
- projected collision

Do not choose an escape lane using only distance or lane utility.

Do not hardcode:

```text
always steer left
```

or:

```text
always steer right
```

Evaluate both directions where possible.

---

# PHASE 12 — Separate driver-requested lane changes from autonomous emergency lane changes

The lane-change system currently mixes user requests and automatic decisions.

Separate the concepts.

Represent different intentions explicitly, for example:

```text
DRIVER_REQUEST
NORMAL_LANE_CHANGE
EMERGENCY_LANE_CHANGE
ABORT
```

A driver lane request must not be indistinguishable from an autonomous emergency maneuver.

Preserve normal manual behavior.

The new safety architecture should augment safety rather than unexpectedly remove normal user control.

---

# PHASE 13 — Add emergency steering direction selection

Emergency steering must choose the safer direction.

Evaluate:

```text
LEFT
RIGHT
BRAKE ONLY
```

using:

- target-lane occupancy
- front TTC
- rear TTC
- lateral clearance
- trajectory
- collision prediction

Conceptually:

```text
LEFT safer
    → left emergency maneuver

RIGHT safer
    → right emergency maneuver

Neither safe
    → brake
```

Do not make emergency steering a fixed directional response.

---

# PHASE 14 — Make V2V data freshness part of threat confidence

Every V2V observation should carry or expose an age/timestamp.

Conceptually:

```text
fresh V2V
    → current information

aging V2V
    → reduced confidence

stale V2V
    → should not automatically remain an active emergency threat
```

Use the existing message/update system instead of inventing another communications mechanism.

The purpose is to prevent ghost threats from stale BSMs.

---

# PHASE 15 — Make `updatePlayer()` consume only the final arbitrated control

Refactor the control flow so `updatePlayer()` no longer has many competing control writers.

Desired structure:

```js
perception
   ↓
threat evaluation
   ↓
risk state
   ↓
controller proposals
   ↓
arbitration
   ↓
ctrl
   ↓
vehicle physics
```

The final object should look conceptually like:

```js
ctrl = {
    steer: finalControl.steer,
    throttle: finalControl.throttle,
    brake: finalControl.brake
};
```

The critical requirement:

> `updatePlayer()` should not independently override the arbitrator with another hidden steering/braking/throttle decision afterward.

Search the entire `index.html` for every assignment to:

```text
steer
throttle
brake
lane-change state
```

and make sure final control authority is consistent.

---

# PHASE 16 — Preserve and integrate the existing collision system

Do not replace the existing simulation collision detection unnecessarily.

Use its geometry/vehicle dimensions as the reference when building projected collision risk.

The objective is consistency:

```text
predicted collision model
      ≈
actual simulation collision model
```

This is especially important for:

- AEB
- emergency steering
- V2V-only targets
- lane-change safety
- head-on traffic

---

# PHASE 17 — Add controller telemetry/debug output

Add a compact internal debug state that can be shown in the existing UI or logged.

At minimum track:

```text
riskLevel
highestRiskThreat
threatId
threatSource
ttc
closingSpeed
distance
selectedAction
lateralAuthority
longitudinalAuthority
laneChangeState
v2vAge
```

Example:

```text
MODE: V2V
RISK: HIGH
THREAT: CAR_42
SOURCE: FUSED
TTC: 1.92s
ACTION: EMERGENCY BRAKE
LATERAL: LKA
LONGITUDINAL: AEB
```

This is for debugging and validation.

Do not let the debug layer alter vehicle behavior.

---

# PHASE 18 — Add the minimap only after the control architecture works

Do NOT build the minimap first.

The minimap is visualization only.

It must not calculate an independent perception system or alter control.

Use existing:

```text
SENS.perceived
unified tracks
authoritative threat data
controller TTC
```

Requirements:

- Toggle button `[ MINIMAP ]`
- Ego vehicle centered
- Ego heading shown
- Configurable radius such as `MINIMAP_RADIUS = 80`
- Other vehicles shown within radius
- Convert actual Three.js world coordinates into ego-relative minimap coordinates
- Show source:
  - LOCAL
  - V2V
  - FUSED
- Distinguish source using marker shape/style and optionally color; do not rely only on color
- Show vehicle ID when practical
- Selecting/hovering a vehicle should show useful details:
  - ID
  - source
  - distance
  - lane
  - relative speed
  - TTC
  - V2V age
- Use the exact TTC/risk calculated by the controller
- Highlight the same highest-risk threat selected by the controller
- If local + V2V are the same vehicle, display one fused marker
- Minimap must never alter:
  - steering
  - throttle
  - brake
  - TTC
  - perception
  - controller priority

Do not create separate minimap perception/raycast/collision logic.

---

# PHASE 19 — Validation tests

After the architecture is integrated, test at least these scenarios:

### Test 1 — No traffic
V2V Assist should not randomly steer, brake, accelerate, or change lane.

### Test 2 — Safe lead vehicle
ACC/normal speed regulation may operate, but no emergency intervention.

### Test 3 — Moderate closing speed
System should remain stable and not oscillate between control states.

### Test 4 — High-risk rear/front conflict
Highest-risk threat must be selected correctly even if another vehicle is physically closer.

### Test 5 — V2V-only vehicle
A vehicle known through V2V but not local sensing must still produce correct risk evaluation.

### Test 6 — LOCAL + V2V same vehicle
It must become one fused track, not two threats.

### Test 7 — Stale V2V message
Old information must age out or lose sufficient confidence.

### Test 8 — AEB vs ACC
AEB must always beat ACC.

### Test 9 — LKA vs lane change
LKA must not fight an active lane change.

### Test 10 — Emergency lane selection
The system must evaluate both left and right rather than blindly picking one direction.

### Test 11 — No safe lane
If neither escape lane is safe, the system must choose braking rather than forcing a dangerous lane change.

### Test 12 — Threat recovery
After a threat disappears, the system must return gradually:

```text
CRITICAL → HIGH → CAUTION → NORMAL
```

without oscillation or permanent autonomous intervention.

---

# IMPORTANT IMPLEMENTATION RULES

1. Preserve the existing simulation before changing behavior.
2. Introduce new abstractions beside existing logic first.
3. Migrate one responsibility at a time.
4. Do not duplicate perception or TTC calculations.
5. Do not let multiple subsystems write final controls independently.
6. Keep controller-specific logic reusable.
7. Put tunable thresholds in named constants.
8. Keep emergency control clearly separate from normal driving.
9. Keep lane changing clearly separate from lane keeping.
10. Keep ACC clearly subordinate to emergency braking.
11. Use unified perception for local/V2V/fused objects.
12. Treat stale V2V information appropriately.
13. Do not change unrelated UI or simulation features.
14. Do not touch `index2.html` or `index3.html`.
15. Do not remove RL-related code unless it is directly necessary for this refactor; RL is out of scope for this task.

---

# MOST IMPORTANT DESIGN REQUIREMENT

Do not turn this into a continuously autonomous self-driving car.

The intended V2V Assist behavior is:

```text
LOW RISK
    ↓
normal driving

HIGH COLLISION RISK
    ↓
autonomous intervention allowed

CRITICAL COLLISION RISK
    ↓
emergency autonomous control
```

The system should be **risk-triggered, not permanently intervention-driven**.

---

# BEFORE EDITING

First inspect `index.html` and identify:

- current perception flow
- current `SENS.perceived` creation
- current TTC calculations
- `computePolicy()`
- `updatePlayer()`
- ACC/IDM logic
- AEB/FCW logic
- lane keeping
- lane change state machine
- emergency/evasive steering
- collision geometry
- all final writes to steer/throttle/brake

Then implement the phases incrementally.

Do not perform a blind large-scale rewrite.

At the end, provide a concise change report showing:

```text
Phase 1: completed
Phase 2: completed
...
Phase 19: completed
```

and specifically identify any phase that could not be fully implemented because the current code structure prevented it.

The code must remain runnable throughout the refactor.