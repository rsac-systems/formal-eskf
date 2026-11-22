# C-LAYER-BOUNDARY: Layer ownership and dependency boundaries

**ID:** `C-LAYER-BOUNDARY`

**Status:** Draft

**Applies to:** Shared numerical core, portable runtime and deployment adapters.

## Basis

Separate numerical behavior from policy and platform effects so each boundary can be verified independently.

## Required behavior

| Layer | Responsibility | Must not own |
|---|---|---|
| Geodesy utilities | Geographic/Cartesian geometry and frame transforms | Estimator state, sensor policy, clocks or publication |
| Numerical core | Prediction, correction, injection/reset and covariance | Clocks, queues, sensor policy or OS effects |
| Portable runtime | Qualification, history, lifecycle, fusion decisions and bounded session composition | Device access, OS scheduling or publication |
| Platform adapter | Message/time conversion, invocation of session phases and output | Duplicated estimator ownership or undocumented changes to core equations |

Keep numerical APIs independently callable and backend/platform effects behind
their declared boundaries. These are responsibilities, not a required class
hierarchy or a generic dispatch framework. A concrete caller-driven INS session
may own the main state/covariance, component instances and their numerical update
order under [SW-NAV](../software/navigation-runtime.md). Its caller owns scheduling,
clock acquisition and platform effects. Components remain independently usable;
the session shall not introduce virtual sensor policies or a platform scheduler.

Geodesy depends on scalar/linalg utilities, not the ESKF. Runtime and adapters may
compose geodesy with core operations. Geographic frame/datum ownership shall not
be pushed into the numerical prediction/correction APIs. The
[origin allocation](../../architecture/origin-rebasing.md) identifies the concrete
state and history owners for that composition.

## Acceptance and verification

Inspect public interfaces, dependency direction and state ownership. Core and
portable-runtime code must not acquire the platform effects prohibited by the
table. A component proof does not establish the adapter's timing or transactions.
