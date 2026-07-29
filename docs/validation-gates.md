# Validation gates — shared legend

Every target has a validation document, and all of them number their gates the
same way, so "gate 3" means the same thing in every conversation:

| Gate | Title |
|---|---|
| 1 | Engine benchmark + invariant check |
| 2 | XR skeleton — session, pacing and input |
| 3 | Render + handedness (BEFORE trusting any teleop motion) |
| 4 | Teleop acceptance |
| 5 | Soak |

- [validation-android.md](validation-android.md) — Quest-class arm64 headset
- [validation-web.md](validation-web.md) — headset browser
- [validation-linux.md](validation-linux.md) — desktop OpenXR (CloudXR)

## The status legend

Every gate line carries exactly one of **PASS** (with date, machine and the
measured number), **UN-RUN** (with `blocked on:`) or **N/A** — never blank. A
gate as a whole may also be **PARTIAL**, which is not a fourth state for a
line: it means the gate decomposes, and every PARTIAL must name a PASS half
with its measured number and an UN-RUN half with its own `blocked on:`. A bare
PARTIAL with no decomposition is the same as blank.

This file exists because that paragraph was previously written out in full in
two validation documents, and a third target was about to make it three copies
of a rule whose whole value is that it is the same rule everywhere.

## Reading across the documents

**The best-evidenced target is the newest one, and that inversion is real.**
The reflex is to assume the oldest target is the validated one. It is the
reverse, and for a mechanical reason: nothing in this repo has ever run on a
headset, so a target's green count measures how much of it can be exercised
*without* one. Android needs a device for everything past gate 1. The web
target needs only a browser. The Linux target needs only a running CloudXR
service — which is why it is the first target whose gate 1 is free and whose
session bring-up can be demonstrated with no headset attached at all.

Do not read a PASS on one target as evidence about another.
