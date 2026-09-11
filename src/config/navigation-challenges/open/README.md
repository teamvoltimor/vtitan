# Open Challenge tuning overlay

Empty, and expected to stay empty. Open Challenge tuning lives entirely in
`src/shared/config/navigation/` (the base config) -- do not add files
here to work around an Obstacles Challenge problem. This directory exists
so `NavigationTuning.load_default(challenge=ScenarioType.OPEN)` has a
symmetrical target and so `load_default(challenge=ScenarioType.OPEN) ==
load_default()` stays a meaningful invariant (see
`test_challenge_tuning_overlay.py`), not just true by omission.
