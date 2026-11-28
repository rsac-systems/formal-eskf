# FHA: Functional Hazard Assessment candidates

Start at externally useful functions: provide estimates, indicate their usability,
maintain reference/time continuity, and report changes to consumers. Ask what loss,
wrong output, late/intermittent output or misleading availability could do in a
specific operating phase. Follow the effect to the consumer boundary; downstream
vehicle effects remain conditional on the actual consumer and mission.

Record: ID, function/context, failure condition and possible effect, requirements,
related FMEA/FTA candidates, and unresolved assumptions. For example, a current
timestamp on an old estimate can mislead a controller even when all values are
finite. This differs from a visibly unavailable estimate.

Engineering constraints and replay evidence usually contribute to a functional
failure rather than forming independent aircraft hazards. Mark that relationship.
Do not assign an aviation severity from the software effect alone.

Basis: [FAA AC 25.1309-1B](https://www.faa.gov/documentLibrary/media/Advisory_Circular/AC_25.1309-1B.pdf),
sections 6.1–6.3, describes function/failure-condition analysis with operational
context and effects. This project uses that analysis discipline; this preliminary
list is neither a completed FHA nor an assertion of Part 25 applicability.
