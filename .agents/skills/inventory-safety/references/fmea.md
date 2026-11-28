# FMEA: Failure Modes and Effects Analysis candidates

Read requirement behavior with its architecture allocation. For each responsible
item/interface, identify plausible failure modes, triggering conditions and local
effects, then link the propagated effect to FHA. Include wrong-but-finite data and
silent partial updates, not only NaN, exceptions or rejected inputs.

Use the contract/owner level for this inventory; a later source-level FMEA may
decompose further. Do not assert a code defect without examining implementation
evidence. A required guard is not evidence that it is implemented or effective.

Record: ID, item/interface and context, failure mode with possible cause, local/
propagated effect, requirements, related FHA/FTA cases and unresolved inputs.
Separate a normal source rejection from failure to reject, and failure to recover
from ordinary unavailability. Include common resource/configuration dependencies.

Basis: [NASA GSFC-HDBK-8004](https://standards.nasa.gov/sites/default/files/standards/GSFC/Baseline/0/GSFC-HDBK-8004_Approved_0.pdf),
section 4.3, and [ECSS-Q-ST-30-02C](https://ecss.nl/wp-content/uploads/standards/ecss-q/ECSS-Q-ST-30-02C6March2009.pdf),
section 6 and Annex B. Both inform item/function, failure-mode, context and effect
identification. Criticality ranking and detection/mitigation evaluation belong to
later analysis; do not introduce RPN or FMECA scoring here.
