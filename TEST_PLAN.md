# Soccer Robot Test Plan (State Machine Steps 4–11)

Work through the phases **in order**. Each phase depends on the ones before it, so the first failure tells you which state to look at. When something goes wrong, copy the printed lines around it (`STATE a -> b`, `search: ...`, `approach: ...`, etc.).

All tuning values are in `Raspberry Pi Code/calibration.json` (section.key below). Edit, then restart the program.

**State numbers:** -2 Defense, -1 Init, 0 Search, 1 Approach, 2 Get behind, 3 Attack. Set the starting state with `state.number = ...` in `main()` in `philfixeverything.py`.

---

## Phase 0: Startup check (no motion, 1 minute)

On the Pi, in the `Raspberry Pi Code` folder:
```
python3 -c "import calibration, states; print('ok')"
```
This catches syntax errors and missing JSON keys without starting any hardware. Fix anything it reports before going on.

- [ ] Prints `ok`

---

## Phase 1: Hand-held run (wheels free, catches runtime errors safely)

Set `state.number = 0` in `main()`. Hold the robot off the floor and run it. Turn it by hand when it "turns" (the gyro tracks you), and move the ball by hand.

| ✓ | Do | Expect to see |
|---|---|---|
| [ ] | Ball 4 ft in front | `search: ball at bearing ...`, `STATE 0 -> 1` |
| [ ] | Bring the ball to about 1.2 ft | `approach: at ball` → `STATE 1 -> 2` |
| [ ] | Rotate the robot until heading ≈ 0 with the ball centered | `get behind: lined up` → `STATE 2 -> 3` |
| [ ] | Pull the ball away and hide it | `lost ball -> search`, then 8 steps, then `STATE 0 -> -2` |
| [ ] | Roll the ball back into view | `defense: ball at bearing` → `STATE -2 -> 1` |

The goal of this phase is to **get through every state at least once without a traceback.** The movements won't look right in the air, and that's fine.

---

## Phase 2: Measure the blind distance (stationary, debug window)

Robot stopped, camera at pan/tilt 90. Move the ball toward the front bumper until **B:** in the debug window switches to 0.

- Blind distance = ________ ft

Then set:

| Setting | Value | Set to |
|---|---|---|
| `approach.stop_ft` | blind + 0.4 ft | ______ |
| `get_behind.radius_ft` | same as `stop_ft` | ______ |
| `attack.blind_ft` | blind + 0.2 ft | ______ |

---

## Phase 3: Search and Approach on the floor

Keep `state.number = 0`. Point the robot **toward the "goal" direction at power-up**, because `goal_heading` stays 0 = "the way it faced at startup." Press Ctrl-C when it reaches state 2 if you only want to test these two states.

| ✓ | # | Setup | Pass | If not, tune |
|---|---|---|---|---|
| [ ] | S1 | Ball 4 ft in front | Approach starts with no turning | — |
| [ ] | S2 | Ball behind | Finds it in 3–5 steps | `search.step_deg` |
| [ ] | S3 | No ball | 8 steps → Defense | — |
| [ ] | S4 | Ball found, then quickly moved left (then right) | First step turns toward it | `search.memory` |
| [ ] | S5 | Red things in the room, no ball | No false `ball at bearing` | `search.confirm_frames` → 3, `min_area.ball` |
| [ ] | A1 | Ball 5 ft straight ahead | Drives straight, slows, stops at `stop_ft` | `approach.speed`, `approach.min_speed` |
| [ ] | A2 | Ball 4 ft, 45° to one side | Curves in, ball stays centered | `approach.k_rot` (weaving → lower it) |
| [ ] | A3 | Ball 4 ft, 90° to one side | Turns in place first, then drives | `approach.turn_only_deg` |
| [ ] | A4 | Pull the ball away while it's driving | Back to Search, turns the right way | `approach.lost_time` |

---

## Phase 4: Get behind, then Attack on the floor

Same startup method (heading 0 = goal direction). Put a box or tape at that end as the "goal."

| ✓ | # | Setup | Pass | If not, tune |
|---|---|---|---|---|
| [ ] | G1 | Ball 3 ft to the robot's right (90°) | Approaches, then orbits **CCW**; heading goes toward 0 | If it orbits the wrong way, check the sign of `left` in `rotate_for_kick()` |
| [ ] | G2 | While orbiting | Ball stays roughly centered | **`get_behind.ff` first** (ball drifts the orbit direction → raise it), then `get_behind.k_face` |
| [ ] | G3 | While orbiting | Distance holds at about `radius_ft` | `get_behind.k_dist`, `get_behind.max_fwd` |
| [ ] | G4 | Ball directly between the robot and the "goal" | Lines up within 10° → `STATE 2 -> 3` quickly | `get_behind.tol`, `get_behind.face_tol` |
| [ ] | G5 | Ball against a side wall | Times out after 8 s and attacks | `get_behind.timeout` |
| [ ] | T1 | Continues from G4 | Pushes in a straight line toward heading 0 | `attack.k_head` |
| [ ] | T2 | Ball rolls slightly sideways while pushing | Robot **slides** sideways, stays square to the goal | `attack.k_strafe` |
| [ ] | T3 | Kick the ball well off to the side | `off line` → `STATE 3 -> 1` → sets up again | `attack.realign_deg` |
| [ ] | T4 | Let it push into the end wall | Escape triggers; timeout or lost → Search | `attack.timeout` |

---

## Phase 5: Defense on the floor

Set `state.number = -2`. After `GYRO BIAS` prints, **turn the robot about 90° by hand** and set it down, so the face phase has something to do.

| ✓ | # | Pass | If not, tune |
|---|---|---|---|
| [ ] | D1 | Turns to heading 0, then backs up straight (no sideways drift) | `defense.k_head` |
| [ ] | D2 | Stops after `backup_time` (no posts) and goes to guard | `defense.backup_time` (how far back it ends up) |
| [ ] | D3 | Guard: pauses, sweeps +40°, pauses, sweeps −40°, … | `defense.look_time`, `defense.sweep_deg` |
| [ ] | D4 | Roll the ball into view during each phase | `STATE -2 -> 1` every time |
| [ ] | D5 | Rear line reached before time runs out | Some bouncing off the line is OK; note it if it looks bad |

---

## Phase 6: Vision tuning (Step 11), then goal-color items

### 6a. HSV tuning
Under the **field lighting**, with `SHOW_MASKS = True`, adjust the `hsv` ranges until each mask shows a clean blob and almost nothing else.

- [ ] Red (ball)
- [ ] Green post
- [ ] Purple post
- [ ] Hat color (yellow / orange)

### 6b. Once green and purple are reliable
- [ ] **Init:** uncomment the `goal_bearing()` block in `init()` and **delete the ball-test block** (`TAKE THIS OUT ONCE I HAVE A GOAL TO LOOK AT`).
- [ ] **Init Test 4:** with the real posts, run Init with the goal switch in **both** positions. It should face the correct goal each time and ignore the wrong one.
- [ ] **Defense:** stand about 1.5 ft in front of your own goal, read the separation of their posts, and set `defense.sep_deg` to it.
- [ ] **Attack:** print `rs.goal_heading` and check that it moves toward the real goal center when the posts are in view.
- [ ] **Heading hold (old Test 5):** covered by D1 and T1. Re-check if the robot drifts.

### 6c. Pre-competition checklist
- [ ] `DEBUG_VIEW = False`
- [ ] `SHOW_MASKS = False`
- [ ] Re-measure the loop rate (it should be higher with the debug view off)
- [ ] `main()` back to `state.number = -1`
- [ ] PK switch tested: drives straight for `pk.time`, then normal play
- [ ] Goal switch set correctly for the match
