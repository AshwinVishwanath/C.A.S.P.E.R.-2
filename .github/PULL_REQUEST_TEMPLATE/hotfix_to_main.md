<!--
Use for: hotfix/* -> main. A bug found on main needs fixing before the
next normal promotion (e.g. before launch). Keep the PR tight.
-->

## Summary
<!-- One sentence: what is broken, what does this fix. -->

## Why hotfix path (not flight-testing first)
<!-- Justify bypassing the normal validation ladder. Time pressure? Trivial scope? -->

## Tiers touched
- [ ] SAFETY-CRITICAL — **double-check interlocks before merge**
- [ ] CORE-FLIGHT
- [ ] Other: __________

## Validation done
- [ ] `make clean && make -j8` succeeds with 0 warnings
- [ ] Manual test on hardware: __________
- [ ] Reproduced the original failure with the previous build
- [ ] Verified the new build does not reproduce the failure

## Post-merge sync
- [ ] `git checkout flight-testing && git merge main && git push`
- [ ] `git checkout dev && git merge main && git push`
