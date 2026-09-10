"""AUT GAME rat package - the player lives in its own folder.

Rat           = the body (sensors + movement)
ExplorerBrain = the decision algorithm (swappable)
RatMemory     = what the rat remembers
See rat/ALGORITHM.md for the full algorithm explanation.
"""
from .rat import ExplorerBrain, Rat, RatError, RatMemory

__all__ = ["ExplorerBrain", "Rat", "RatError", "RatMemory"]
