# `q_des` is not a physical configuration, and `q` is

A property of the archive that will keep surfacing, so it is written down once.

## The test

Take a clip's angles, drive the articulation to them in the air, read `body_pos_w`, and
per frame fit the plane through the feet the clip calls **stance**. If the angles describe
a real robot standing on flat ground, that fit is well behaved: the implied trunk tilt is
steady across the cycle and no swing foot comes out *below* the plane its own stance feet
define.

## The result — WALK, same pipeline, two streams

| | FL | FR | RL | RR | implied trunk tilt | swing feet below the plane |
|---|---|---|---|---|---|---|
| `q_des` commanded | 46.0 | 123.5 | **−11.9** | 81.9 mm | +3.00°, range **−2.68 to +8.97** | RL **66.7%** |
| `q` achieved | 29.0 | 11.9 | 4.9 | 11.4 mm | +2.02°, range +0.96 to +3.33 | **0% on every leg** |

`q_des` requires the trunk to swing through **11.65° inside one gait cycle** to keep three
stance feet on the ground, and still leaves the left-rear "swing" foot below that plane on
two thirds of its swing frames. It is not a pose the robot was ever in.

`q` is consistent: 2.37° of tilt across the whole cycle, nothing below the plane.

## Why, and what follows

Nothing is wrong with the archive. `q_des` is what the sport controller *asked* for at
kp 40; the real robot's joints sagged under load and ended up at `q`. The gap is the same
PD sag that shows up everywhere else in this project — about 40 mm of leg extension.

What follows for anything that reasons about geometry:

- **Foot positions, clearances, stance planes and body heights must come from `q`.** A
  clearance computed from `q_des` is a clearance the robot was never at, and it will be
  wrong by tens of millimetres in a direction that looks plausible.
- **The replay is right to play `q_des`** — that is what the controller commanded and what
  the sim's own PD should sag from. The two uses are different and only the *measurement*
  side has to change.
- Anything that derives a target from the commanded stream — the stance geometry that
  `--plant-comp height` was built against, for instance — is deriving it from a pose that
  does not exist.
