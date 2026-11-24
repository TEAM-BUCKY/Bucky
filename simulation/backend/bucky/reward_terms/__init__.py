"""Reward terms. Importing this package registers the built-in PWS and rule terms.

To add a weight, write a ``@reward_term`` function in ``pws.py`` or ``rules.py``, or in a new
module that you import below. See ``registry.py``.
"""
from bucky.reward_terms.registry import (  # noqa: F401
    PROFILES, TERMS, RewardContext, TermSpec, check_weight, profile_terms, reward_term, unregister,
)
from bucky.reward_terms import pws, rules  # noqa: F401,E402  (registers the terms)
