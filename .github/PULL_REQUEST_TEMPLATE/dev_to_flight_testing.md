<!--
Use for: dev -> flight-testing. The work is graduating from experimental
into hardware-validation territory. Be thorough.
-->

## Summary
<!-- What is being promoted and why now? -->

## Why this is ready for hardware validation
<!--
Evidence the change has matured beyond experimental status. Think:
sim runs, HIL replays, ground tests, code review, peer feedback.
-->

## Tiers touched (after merge into flight-testing)
- [ ] SAFETY-CRITICAL — **Requires explicit sign-off before merge**
- [ ] CORE-FLIGHT
- [ ] TEST-HARNESS
- [ ] EXPERIMENTAL (stays experimental on `flight-testing`? add reasoning)
- [ ] GROUND-STATION
- [ ] HOST-SIDE
- [ ] None

## Hardware test plan
<!-- What tests will validate this on flight-testing before promoting to main? -->
- [ ] HIL replay of flight log: scenario __________
- [ ] Bench test: __________
- [ ] Captive carry / static / ground fire: __________
- [ ] Other: __________

## Pre-merge checklist
- [ ] `make clean && make -j8` succeeds with 0 warnings
- [ ] Tier1/Tier2 unit tests pass (if applicable)
- [ ] Tier banner headers updated for any new files
- [ ] No `EXPERIMENTAL` banner left on files being promoted out of dev
- [ ] `BRANCHING.md` and PRD references updated if behavior changed

## Risk if merged
<!-- One sentence: what is the worst-case if this misbehaves on flight-testing hardware? -->
