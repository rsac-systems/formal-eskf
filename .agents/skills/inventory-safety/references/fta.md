# FTA: Fault Tree Analysis candidates

Select bounded, observable top events from FHA and the FMEA effect chains. State
which interface, operating phase and failure constitute the event. Record known
candidate contributors and the questions needed to construct a tree later.

This phase selects analysis scope; it does not construct gates or claim causal
sufficiency, independence, minimum cut sets or quantitative probability. List a
single cause as a possible contributor, not a proven path to the top event.

Record: ID, top event/boundary, FHA and FMEA links, requirements, and unresolved
causal dependencies. Include shared clocks, configuration, reference transforms,
resources and evidence/oracle dependencies where relevant. A replay qualification
escape is an assurance top event, not itself an in-flight aircraft hazard.

Avoid one top event per requirement. Group related effects only when the boundary
and acceptance predicate remain intelligible. Unspecified thresholds or consumers
remain open; do not invent them to complete a tree.

Basis: [NASA Fault Tree Handbook](https://s3vi.ndc.nasa.gov/ssri-kb/static/resources/Fault%20Tree%20Handbook_NASA.pdf),
sections 3–5, establishes objectives, scope, construction and evaluation as distinct
work. Only objective/scope and preliminary contributors are recorded here.
