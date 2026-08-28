"""CPU-only analysis of the read-only curated Go2 log set (36 sessions).

Nothing here writes to the curated tree and nothing imports Isaac Lab / torch.
Paths are resolved by :mod:`terrain_toolkit.paths` (``$GO2_CURATED_ROOT``, else
the sibling directory ``../curated``).

The re-exports below are LAZY (PEP 562).  They used to be eager, which meant that
importing anything from this package -- even the numpy-only contact detector --
also imported ``profile`` and ``transitions``, and those need pandas.  The Isaac
Lab container has no pandas, so ``sim.diagnose`` asking for the contact rule
killed the replay after the handover with no traceback surfacing.  ``session``,
``window`` and ``contact`` are numpy-only and must stay importable on their own.
"""

_LAZY = {
    "LEGS": "session", "JOINTS": "session", "POS_STOP_F": "session",
    "Session": "session", "load_session": "session", "iter_sessions": "session",
    "MotionWindow": "window", "detect_motion": "window",
    "ContactResult": "contact", "detect_contact": "contact", "classify_pattern": "contact",
    "profile_session": "profile", "profile_all": "profile",
    "transition_table": "transitions", "predecessor_table": "transitions",
    "summarize": "transitions",
}

__all__ = sorted(_LAZY)


def __getattr__(name):
    mod = _LAZY.get(name)
    if mod is None:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    import importlib
    return getattr(importlib.import_module(f".{mod}", __name__), name)


def __dir__():
    return __all__
