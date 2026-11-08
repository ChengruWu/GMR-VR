"""Experimental, offline EVA Body input normalization."""

from .source import normalize_observation

__version__ = "1.0.0"


def create_backend():
    """Create the EVA in-process Body backend."""
    from .backend import BodyBackend

    return BodyBackend()


__all__ = ["create_backend", "normalize_observation"]
