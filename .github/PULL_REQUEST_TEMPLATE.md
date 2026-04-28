<!--
GitHub picks this default template for new PRs. If your PR is a branch
promotion, switch templates by appending a query string to the PR-create URL:

  ?template=dev_to_dev.md
  ?template=dev_to_flight_testing.md
  ?template=flight_testing_to_main.md
  ?template=hotfix_to_main.md

See BRANCHING.md for the maturity model and which template to use.
-->

## Summary
<!-- 1-3 bullets describing what changed and why. -->

## Target branch
<!-- Which long-lived branch are you merging into? -->
- [ ] `dev` (experimental work)
- [ ] `flight-testing` (validated work, ready for hardware test)
- [ ] `main` (proven, hardware-tested)

## Tiers touched
<!-- List by tier label of any modified files. SAFETY-CRITICAL changes need a sign-off. -->
- [ ] SAFETY-CRITICAL
- [ ] CORE-FLIGHT
- [ ] TEST-HARNESS
- [ ] EXPERIMENTAL
- [ ] GROUND-STATION
- [ ] HOST-SIDE
- [ ] None (docs / config only)

## Test evidence
<!-- Build clean? Tests run? HIL run? On-board flash test? Attach logs/screenshots if relevant. -->
- [ ] `make clean && make -j8` succeeds with 0 warnings
- [ ] Bench/HIL test where applicable
- [ ] Tier1/Tier2 unit tests pass (if telemetry/protocol code touched)
