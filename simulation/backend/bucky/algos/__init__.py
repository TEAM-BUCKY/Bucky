"""Algorithm registry package.

Importing this package registers every built-in algorithm into the registry. Import the
registry API from here::

    from bucky.algos import registry
    spec = registry.get_spec(algo)
    model = spec.build(env=..., params=..., net_arch=..., seed=..., device=..., ...)
"""
from __future__ import annotations

# Importing these has the side effect of registering the built-in algos into the registry
# (sb3_algos = core SB3 + hybrid; contrib_algos = sb3-contrib extras).
from bucky.algos import contrib_algos, registry, sb3_algos  # noqa: F401

__all__ = ["registry"]
