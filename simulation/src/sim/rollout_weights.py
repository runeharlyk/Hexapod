"""Score weights shared by the evaluation objective and the score-aligned training reward.

Split into their own module so `hexapod_mj_env` can import them without importing `rollout`,
which imports the env back.
"""

SCORE_W_STUCK = 0.30
SCORE_W_KNOCK = 0.25
