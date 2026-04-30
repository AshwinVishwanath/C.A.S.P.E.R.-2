<!--
Use for: flight-testing -> main. The work has passed hardware validation
and is being blessed as the new stable baseline. Highest bar.
-->

## Summary
<!-- What's being promoted to the proven baseline. -->

## Hardware validation evidence
<!--
This is the bar: flight-testing has been put through real hardware exercise.
Attach logs, links to flight data, bench-test results, etc.
-->
- [ ] HIL run: __________
- [ ] Ground / captive test: __________
- [ ] Live flight: __________
- [ ] Decoder/telemetry parsing verified end-to-end
- [ ] Soak time on `flight-testing` (≥ N days / hours): __________

## Tiers touched (now landing on main)
- [ ] SAFETY-CRITICAL
- [ ] CORE-FLIGHT
- [ ] TEST-HARNESS
- [ ] GROUND-STATION
- [ ] HOST-SIDE
- [ ] None

## Pre-merge checklist
- [ ] `make clean && make -j8` succeeds with 0 warnings
- [ ] All tier banners present and accurate
- [ ] No `EXPERIMENTAL` tier files in this PR (those should not reach `main`)
- [ ] `INTERFACE_SPEC.md` updated if telemetry/command surface changed
- [ ] `BRANCHING.md`, PRD, and `DEV_UPDATE.md` updated for any user-visible change
- [ ] Version / build identifier bumped if applicable

## Post-merge tasks
- [ ] Merge `main` -> `flight-testing` to keep validation branch in sync
- [ ] Merge `main` -> `dev` to keep experimental branch in sync
- [ ] Tag release if this is a flight cut

## Rollback plan
<!-- If this turns out broken on main, what's the revert path? -->
