---
name: explain-concepts
description: "Explain technical and mathematical concepts with clear symbol definitions, small logical steps, and concrete examples. Use when the user asks what a mechanism means, how it works, why a formula applies, or needs clarification during a design review."
---

# Explain Concepts

Explain in the user's language. Start with the specific question they are asking.
Use these two principles:

- Define each symbol simply and clearly before using it.
- Make each sentence the smallest understandable logical unit, then connect the
  units in dependency order.

## Build the explanation

1. State the immediate purpose in plain words. Identify the operation being
   explained; distinguish prediction, selection, and correction when relevant.
2. Introduce only the concepts needed for the next step. Give each symbol a
   concrete meaning and unit. State the reference time, frame, or baseline when
   it changes the meaning. Distinguish measured values, configured estimates,
   and computed results.
3. Put one understandable claim in each sentence. Name the subject or quantity
   instead of relying on ambiguous pronouns. Connect each step through an
   explicit reason, condition, or result. Short sentences must not omit the
   intermediate facts needed to understand the conclusion.
4. State a rule in words before giving its formula. Keep notation consistent.
   Explain each inequality's direction and boundary. Do not introduce an
   unexplained index, abbreviation, or new symbol inside an equation.
5. When an example helps, use one small scenario. State its assumptions, substitute
   numbers with consistent units, and explain what the result causes next. Show
   another branch only when needed to clarify the distinction being asked about.

Introduce definitions beside their first use. Use a small symbol table when it
reduces backtracking; do not front-load a glossary for the whole subsystem.
Use equations for calculation and plain sentences for their meaning. Do not
substitute implementation code for an explanation unless the user asks for code.

## Preserve the meaning

- Label illustrative values as assumptions. Identify actual defaults and source
  behavior from evidence before claiming them. A worked example does not prove
  that an implementation matches a reference system.
- Keep distinct quantities distinct: arrival time versus measurement time,
  scheduling offset versus expiry limit, or eligibility versus successful fusion.
  Do not turn a useful simplification into a different rule.
- When the user checks their understanding, identify what is correct and clarify
  the specific ambiguity. Do not agree first and then quietly change the meaning.
- If the user is still understanding a design, answer that question. Do not
  introduce a redesign, advance to the next decision, edit specifications, or
  treat the question as approval. Separate explanation from requested advice.
- Scale the response to the question. A short clarification does not need the
  entire workflow again. If the user remains confused, repair the missing logical
  link instead of repeating the same abstract wording or adding more jargon.

## Example of the explanation style

For a question about timestamp delay compensation:

"Let t_m be the timestamp carried by a measurement, in milliseconds.
Let d be the configured estimate of how far that timestamp lags the physical
time represented by the data, in milliseconds.
Let t_e be the estimated effective measurement time, in milliseconds.

We subtract the estimated delay: t_e = t_m - d.
Suppose t_m = 1010 ms and d = 10 ms.
Then t_e = 1000 ms.
The data is treated as representing the physical state at 1000 ms.

Arrival time is when the receiver obtains the data.
If its timestamp already represents 1000 ms, receiving it at 1010 ms does not
by itself justify subtracting another 10 ms."

This example illustrates the explanation method, not a universal sensor model.

Before sending, check that every symbol is defined, every calculation has units,
and the reader can follow each conclusion from the preceding facts without
guessing an omitted step.
