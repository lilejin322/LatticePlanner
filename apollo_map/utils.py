"""Helpers the vendored map package imports from its parent package."""

import math


def require_nonnegative(**values: float) -> None:
    """Raise ``ValueError`` for the first value that is not finite and nonnegative."""
    for name, value in values.items():
        if not (math.isfinite(value) and value >= 0):
            raise ValueError(f"{name} must be finite and nonnegative")
