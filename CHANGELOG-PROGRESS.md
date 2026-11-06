# Progress & Change Log

This file is an **append-only timeline** of code changes, captured from **October 19, 2026** onward.

How it works:
- Each snapshot below is a point-in-time capture, taken whenever a "log my changes" request is made.
- Snapshots are never overwritten. Newest snapshot at the top.
- Each snapshot records the delta from the previous snapshot so the progression of individual values is visible over time.
- Limitation: captures happen at log time, not per keystroke. Edits made between two captures are summarized as the net change between them.

Legend: `A -> B` means a value changed from A to B. Baseline = last committed state (commit `61effff` "Add MVLib telemetry").

---

## Timeline (newest first)

### Snapshot 9 — 2026-10-31 14:40 EDT
Delta since Snapshot 8:
- **src/autons.cpp (`skills()`):**
  - 1st pin:
    - Lift after gathering: `900 -> 1000`.
    - Score drive: target `(-4.5, -26) -> (-3.5, -27.25)`.
    - Lift after scoring: `900 -> 1100`.
  - 2nd pin, gather:
    - Turn toward pin: heading `-39.5 -> -37.5`.
    - Lift-lower delay: `400 -> 500` ms.
    - Gather drive: target `(0, -33.5) -> (4, -33.5)`; `.maxSpeed` `60 -> 42`.
    - Claw-drop trigger: `waitUntil(27.5) -> waitUntil(24)`.
  - 2nd pin, score:
    - Drive to goal changed from `moveToPoint(-19, -25, 1250)` to **`moveToPose(-30, -36, 80, 1750, {.forwards = false, .minSpeed = 80})`**. It now also ends at a heading of 80° and can curve on the way in.
    - Lift-out after scoring: `1400 -> 1600`.
    - Removed `intake.move(127)` after the `setPose(0, 0, 0)` that follows scoring.
  - 3rd pin (match load):
    - Removed `intake.move(127)` before `claw_spin`. The intake now starts right after `spinclaw(2)`.
    - Drive toward match load: `(6, 20, 2272) -> (5, 26, 2550)`.
    - 90° turn to the wall: timeout `600 -> 650` ms.
    - Wall drive: `(10, 24.5, 1972) -> (10, 26, 800)`.
    - **Un-commented and now running:** intake + 1500 ms delay (this replaces the committed `moveToPoint(24, 25.5)` + 500 ms), back-out `moveToPoint(0, 27.5)` (was `(6, 25.5)`), `spinclaw(1)`, lift 2600, turn to `-6`, reverse to `(0, -3)` at maxSpeed 80, turn to `0`, lift 1800, score with claw, then lift up.
    - Lift after the match-load score: `2600 -> 2900`.
    - Added comments: "Facing The Wall Near The Match Load", "Turns To The Goal", "Moves To Goal", "Score On Goal".
  - Block comment `/*` moved from line 163 to line 188, so the first match-load goal score now runs. It still closes at the `*/` on line 454. The nested `/*` is now at line 319.
- `project.pros` unchanged.
- **Status:** still uncommitted (HEAD is `61effff`); needs on-field testing.
- **Note:** this is the largest change since tracking started. The route now runs through the first match-load goal score.

### Snapshot 9 — 2026-10-31 14:40 EDT
Delta since Snapshot 8 (2026-10-24 21:41). This covers about a week of edits, so it shows the net change, not each individual edit.
- **src/autons.cpp (`skills()`):**
  - 1st pin, after gathering: lift `900 -> 1000`.
  - 1st pin, score:
    - Drive to goal: target `(-4.5, -26) -> (-3.5, -27.25)`.
    - Lift after scoring: `900 -> 1100`.
  - 2nd pin, gather:
    - Turn toward pin: heading `-39.5 -> -37.5`.
    - Lower-lift delay: `400 -> 500` ms.
    - Gather move: target `(0, -33.5) -> (4, -33.5)`; `.maxSpeed` `60 -> 42`.
    - Claw-drop trigger: `waitUntil(27.5) -> waitUntil(24)`.
  - 2nd pin, score:
    - Drive to goal: `moveToPoint(-19, -25, 1250)` -> **`moveToPose(-30, -36, 80, 1750)`**. This is a new motion type. The robot now curves in and ends facing 80°, where before it only drove to a point.
    - Lift out of goal: `1400 -> 1600`.
    - Removed `intake.move(127)` before the `turnToHeading(32)`.
  - 3rd pin (match load), more of it now runs:
    - Removed `intake.move(127)` right after `setPose`. Added `intake.move(127)` after `spinclaw(2)` instead.
    - Drive toward match load: `(6, 20, 2272) -> (5, 26, 2550)`.
    - Turn to 90° (face the wall): timeout `600 -> 650` ms.
    - Drive into the wall: `(10, 24.5, 1972) -> (10, 26, 800)`.
    - New: `intake.move(127)` followed by `pros::delay(1500)` while loading.
  - Match-load **scoring is now un-commented and runs**:
    - Back out: `moveToPoint(0, 27.5, 1572, backwards)`. The committed version was `(6, 25.5)`.
    - `spinclaw(1)`, then lift to `2600`, then a 200 ms delay.
    - Turn to `-6`.
    - Reverse to the goal: `moveToPoint(0, -3, 1772, backwards, maxSpeed 80)`.
    - Turn to `0`, then lift to `1800`, then `claw_spin.move(127)`, then a 700 ms delay.
    - Lift out: `2600 -> 2900`. Then `claw_spin.move(-127)`, `spinclaw(2)`.
    - New comments: "Turns To The Goal", "Moves To Goal", "Score On Goal".
  - Comment blocks: the `/*` that starts the commented-out code moved from line 163 to **line 188**, just after "end last goal 1 score". It still closes at the `*/` on line 454. The leftover nested `/*` is now at line 319 and is still harmless.
- `project.pros` unchanged.
- **Status:** still uncommitted (HEAD is `61effff`); needs on-field testing.
- **Notes:**
  - The comment "Moves To Goal" sits on `turnToHeading(0, 50)`, which is a turn. The move to the goal is the line above it.
  - The match-load section now has a full intake → back out → score sequence running, so the route runs much longer than it did on 10-24.

### Snapshot 8 — 2026-10-24 21:41 EDT
Delta since Snapshot 7:
- **src/autons.cpp (`skills()`):**
  - 2nd pin, back away from goal: `moveToPoint` y `17 -> 15`.
  - 2nd pin, score:
    - Turn to face goal: heading `146 -> 131` (timeout stays 675 ms).
    - Drive to goal: target `(-15, -25) -> (-19, -25)`.
- No other changes since Snapshot 7. `project.pros` unchanged. The comment blocks are still the same: the `/*` at line 163 closes at line 462.
- **Status:** still uncommitted (HEAD is `61effff`); needs on-field testing.
- **Note:** the turn to the 2nd-pin goal went back 15° (`146 -> 131`) while the drive to the goal moved 4 in further left. These two changes probably offset each other, so the robot comes at the goal on a steeper angle but ends up in about the same place.

### Snapshot 7 — 2026-10-24 20:13 EDT
Delta since Snapshot 6:
- **src/autons.cpp (`skills()`):**
  - Preload, drive to alliance goal: target y `-17 -> -19`. Target is now `(-25, -19)`.
  - 1st pin, turn to face it: heading `-80 -> -83`.
  - 1st pin, gather move: target `(0, -12) -> (-4, -16.5)`; timeout `1410 -> 1510` ms; claw-drop trigger `waitUntil(12) -> waitUntil(10.75)`. Comment changed from "Backs Up" to "Gathers Pin".
    - With `setPose(-5, 0, 0)` still in place, the target is now 1 in to the robot's right and 16.5 in behind it. Before, it was 5 in right and 12 in back. The path is now almost straight back and longer.
  - Removed the comment block Kiro added in Snapshot 6 that explained the 1st-pin gather move. Also removed the "Prepare Claw To Gather The 1st Pin" comment on `spinclaw(1)`.
  - 1st pin, score move: target `(0, -25) -> (-4.5, -26)`; timeout `1250 -> 1325` ms.
  - 2nd pin, back away from goal: `moveToPoint` y `24 -> 17`.
  - 2nd pin, gather move: target y `-32.5 -> -33.5`.
  - 2nd pin, score:
    - Pre-score lift `1600 -> 1800`; delay `600 -> 800` ms.
    - Turn to face goal: heading `134 -> 146`; timeout `600 -> 675` ms.
    - Drive to goal: target `(0, -25) -> (-15, -25)`.
  - 3rd pin (match load), now partly un-commented:
    - Drive toward match load: `moveToPoint(6, 25.5, ...) -> (6, 20, ...)`.
    - Drive along the wall `moveToPoint(10, 24.5, 1972)` now **runs**. In the committed version this line was `(25, 25.5)`.
    - The `/*` that starts the commented-out block moved from line 162 to line 163, just after this move. It still closes at the `*/` on line 462.
  - Removed a few blank lines.
- `project.pros` unchanged.
- **Status:** still uncommitted (HEAD is `61effff`); needs on-field testing.
- **Note:** the route now runs one more drive (`moveToPoint(10, 24.5)`) after the 90° turn at the match load. Make sure the robot has room and isn't still holding a pin it needs to score.

### Snapshot 6 — 2026-10-24 17:55 EDT
Delta since Snapshot 5:
- **src/autons.cpp (`skills()`):**
  - 1st pin: turn to face pin heading `-134 -> -80`; timeout `450 -> 650` ms; `.maxSpeed` `127 -> 110`.
  - Added a comment block above the 1st-pin gather `moveToPoint(0, -12, ...)` explaining the move. The robot starts at the top-left corner of the blue alliance goal, reverses about 13 in on a diagonal toward the pin at a crawl, and the claw drops after 12 in and comes back up holding the pin. (Comment written by Kiro on request.)
- `project.pros` unchanged.
- **Status:** still uncommitted (HEAD is `61effff`); needs on-field testing.
- **Note:** `-80` is a much bigger change than any earlier value for this turn (`-139`, `-145`, `-147`, `-150`, `-134`).

### Snapshot 5 — 2026-10-24 17:27 EDT
Delta since Snapshot 4:
- **src/autons.cpp (`skills()`):**
  - Preload, move to alliance goal: target x `-26 -> -25` (y stays `-17`); timeout `2000 -> 1900` ms.
  - 1st pin: turn to face pin heading `-150 -> -134`; turn timeout `300 -> 450` ms.
- No other changes since Snapshot 4. `project.pros` unchanged.
- **Status:** still uncommitted (HEAD is `61effff`); needs on-field testing.
- **Note:** the 1st-pin turn swung back 16° (`-150 -> -134`), past every earlier value (`-139`, `-145`, `-147`). This probably makes up for the new alliance-goal end point (`-25, -17`), since the robot now begins the turn from somewhere else.
- **Resolved — `/*` flag:** checked `src/autons.cpp`. The `/*` at line 162 closes with the `*/` at line 462. Both sit inside `skills()`, so the rest of the old route (match load, goal 2 scores, and so on) is commented out and the function's closing `}` is untouched. A leftover `/*` at line 327 is nested inside that block. It's harmless, but it may trigger a "`/*` within comment" compiler warning.

### Snapshot 4 — 2026-10-24 17:22 EDT (today's progress)
Delta since Snapshot 3:
- **src/autons.cpp (`skills()`):**
  - Preload, move to alliance goal: target `(-24, -9.8) -> (-26, -17)`; timeout `1300 -> 2000` ms; speed option `.minSpeed = 90 -> .maxSpeed = 90` (now a speed cap instead of a speed floor).
  - 1st pin: back-up turn heading `-147 -> -150`.
  - 2nd pin: turn-toward heading `-38 -> -39.5`, turn timeout `500 -> 550` ms; gather `moveToPoint` y `-33.75 -> -32.5`, timeout `2380 -> 2350` ms.
- No other changes since Snapshot 3. `project.pros` unchanged.
- **Status:** still uncommitted (HEAD is `61effff`); needs on-field testing.
- **Note:** switching `.minSpeed` to `.maxSpeed` on the alliance-goal drive means the robot can slow down near the target instead of being forced to at least 90. Paired with the longer 2000 ms timeout, this run should be slower but more accurate.
- **Still open:** the match-load section's `/*` block comment — confirm it has a matching `*/`.

### Snapshot 3 — 2026-10-20 21:15 EDT
Delta since Snapshot 2:
- **src/autons.cpp (`skills()`):**
  - 1st pin: back-up turn heading `-145 -> -147`.
  - 2nd pin: turn-toward heading `-37.9 -> -38`; gather `moveToPoint` y `-34 -> -33.75`.
- No other changes since Snapshot 2. `project.pros` unchanged.
- **Status:** uncommitted in working tree; needs on-field testing.

### Snapshot 2 — 2026-10-19 (second "add changes" request)
Delta since Snapshot 1:
- **src/autons.cpp (`skills()`):**
  - Preload: alliance-goal `minSpeed` recorded as `90` (Snapshot 1 had noted 80); claw delay `650` ms (Snapshot 1 had noted 750).
  - 1st pin: back-up turn heading `-139 -> -145` (newly captured); turn-to-pin heading `133`.
  - 2nd pin: turn-toward heading `-37.9` and `134`; gather y `-33.5 -> -34`; pre-score lift `1400 -> 1600` with delay `600` ms; score-down lift `600`; post-score delay `450` ms; lift-out `1400`.
  - Added 3RD PIN (MATCH LOAD) section header; match-load body left commented out with a single `/*` opener.
- **Status:** uncommitted in working tree; needs on-field testing.

### Snapshot 1 — 2026-10-19 (first "add changes" / catch-up request)
First capture of the uncommitted working tree against baseline commit `61effff`.
- **Files:** src/autons.cpp, project.pros
- **src/autons.cpp (`skills()` routine):**
  - Added section header comments (SCORING PRELOAD, GATHERING 1ST PIN, SCORING THE 1ST PIN, GATHER THE 2ND PIN, SCORING THE 2ND PIN) and inline comments throughout.
  - Preload: alliance-goal approach `minSpeed` `100 -> 80`; claw placement delay `650 -> 750` ms.
  - 1st pin: gather `moveToPoint` y `-15 -> -12`; lift `move_absolute` `750 -> 900`; turn-to-goal heading `127 -> 133`.
  - 2nd pin: turn heading `-39.3 -> -37.9` and `130 -> 134`; gather `moveToPoint` y `-35 -> -33.5`; lift heights `1200 -> 1400` and `700 -> 600`.
  - Commented out the 3rd pin / match-load section with `/*`.
- **project.pros:** upload description -> "J-Lucky Cats 20850C"; icon `pros -> pizza`; slot `1 -> 3`.
- **Why:** Field tuning of the skills route for more reliable pin gathering/scoring.
- **Status:** uncommitted in working tree; needs on-field testing.
- **Note / flag:** the match-load section opens a `/*` block comment — confirm there is a matching `*/` later in the file so it doesn't comment out more than intended or break the build.

### Snapshot 0 — 2026-10-19 — Tracking started
- Created this progress log. Baseline is commit `61effff` "Add MVLib telemetry".

<!--
Snapshot template — copy for each new capture, add at the TOP of the timeline:

### Snapshot N — YYYY-MM-DD HH:MM TZ
Delta since Snapshot N-1:
- **file:** value `A -> B`, ...
- **Status:** ...
-->
