"""Simultaneous stance-foot count for every clip -- the CLAUDE.md 2.5 gate.

WHY THIS IS A CLIP-LEVEL MEASUREMENT AND NOT A SIM ONE
------------------------------------------------------
Every arm in this project replays a recorded clip open loop.  The contact pattern the
controller COMMANDS is therefore a property of the clip and of whatever an intervention
does to the clip's foot trajectories -- it is not something the simulator decides.  So the
gate is answered here, off ``data/skill_clips.npz``, in a second and with no GPU.

That also fixes the gate's scope, which is the useful part:

  * an intervention that edits a foot's HEIGHT (``--swing-lift``) moves the commanded
    contact pattern and must be measured here;
  * an intervention that adds TORQUE to legs the recording already has down
    (``--roll-couple``, ``--yaw-moment``) cannot move it, because it never writes
    ``q_des`` for a swing leg.  It is exempt by construction, and this script says so
    rather than leaving it to be assumed.

WHAT THE NUMBERS MEAN (the gait definitions, confirmed against our own recordings)
    WALK   4-beat, >= 3 feet down at all times, STATICALLY stable
    TROT   2-beat, diagonal pairs, 2 feet down, DYNAMICALLY stable
Static stability is why WALK survives an open-loop replay and TROT does not: this harness
has no balance feedback to supply the dynamic half.
"""
import argparse, sys
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from terrain_toolkit.paths import PROJECT_ROOT as PROJ                      # noqa: E402

ARCHIVE = PROJ / "data" / "skill_clips.npz"


def stance_counts(contact):
    """contact: (T, 4) bool -> per-frame number of feet down."""
    return np.asarray(contact, bool).sum(axis=1)


def report(name, contact, fs, note=""):
    c = stance_counts(contact)
    T = len(c)
    hist = {k: int((c == k).sum()) for k in range(5)}
    duty = float(np.asarray(contact, bool).mean())
    below3 = float((c < 3).mean())
    beats = int(np.count_nonzero(np.diff(np.asarray(contact, bool), axis=0)))
    print(f"{name:22s} fs={fs:6.1f}Hz T={T:5d}  duty={duty:.3f}  "
          f"mean feet={c.mean():.2f}  min={c.min()}  "
          f"time<3 feet={below3*100:5.1f}%   hist{hist}  {note}")
    return dict(name=name, duty=duty, mean_feet=float(c.mean()), min_feet=int(c.min()),
                frac_below_3=below3, hist=hist, transitions=beats)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--archive", default=str(ARCHIVE))
    ap.add_argument("--rate", choices=("lo", "hi"), default="lo")
    args = ap.parse_args()

    z = np.load(args.archive, allow_pickle=True)
    names = [str(x) for x in z["clip_names"]]
    print(f"archive: {args.archive}")
    print(f"leg order (stored): {[str(x) for x in z['leg_order']]}   rate: {args.rate}\n")

    out = []
    for nm in names:
        key = f"{nm}__{args.rate}__contact"
        if key not in z:
            print(f"{nm:22s} -- no {args.rate} rate in archive"); continue
        contact = z[key]
        t = z[f"{nm}__{args.rate}__t"]
        fs = 1.0 / float(np.median(np.diff(t))) if len(t) > 1 else float("nan")
        out.append(report(nm, contact, fs))

    print()
    print("GATE (CLAUDE.md 2.5): an intervention that moves a clip's foot HEIGHT must keep")
    print("WALK at >= 3 feet down.  WALK's row above is the reference; re-run this with the")
    print("edited clip to check.  Torque-only terms (--roll-couple, --yaw-moment) never")
    print("write a swing leg's q_des and are exempt by construction.")
    return out


if __name__ == "__main__":
    main()
