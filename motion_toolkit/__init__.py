"""CPU-only analysis of the read-only curated Go2 log set (36 sessions).

Nothing here writes to the curated tree and nothing imports Isaac Lab / torch.
Paths are resolved by :mod:`terrain_toolkit.paths` (``$GO2_CURATED_ROOT``, else
the sibling directory ``../curated``).
"""

from .session import LEGS, JOINTS, POS_STOP_F, Session, load_session, iter_sessions  # noqa: F401
from .window import MotionWindow, detect_motion  # noqa: F401
from .contact import ContactResult, detect_contact, classify_pattern  # noqa: F401
from .profile import profile_session, profile_all  # noqa: F401
from .transitions import transition_table, predecessor_table, summarize  # noqa: F401
