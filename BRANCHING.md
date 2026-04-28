# Branching Model

C.A.S.P.E.R.-2 firmware uses a three-tier maturity ladder. Each long-lived
branch represents a different level of confidence in the code on it.

```
   dev  ─►  flight-testing  ─►  main
 (raw)     (validating)        (proven)
```

Code only flows **upward** through PRs. Updates to upstream branches flow
**downward** through merges to keep the lower branches from drifting.

---

## The three branches

### `main` — proven baseline
- Has been flown, or has passed full hardware validation.
- This is what you flash if you need a known-good build today.
- Only accepts PRs from `flight-testing`.
- Never push directly. Hotfixes go through a `hotfix/*` branch off `main`.

### `flight-testing` — under hardware validation
- Safety- or core-critical changes that will eventually land on `main`.
- Pyro logic, FSM transitions, telemetry framing, sensor drivers,
  EKF math — anything that the flight computer must do correctly.
- Code here is **not yet trusted**. It must demonstrate it works on real
  hardware (HIL, ground tests, captive-carry, etc.) before promotion.
- Accepts PRs from feature branches and from `dev` (when a dev feature
  has matured enough to deserve hardware validation).
- Only `main` is downstream — when `main` updates, merge `main` →
  `flight-testing` to stay in sync.

### `dev` — speculative, untrusted
- Anything that **may** one day fly but isn't ready for hardware validation.
- New EKF derivations, alternative algorithms, scaffolding modules,
  HIL tooling, host-side scripts, MATLAB analysis, exploratory drivers.
- Failure here is fine. Success here means "promote to `flight-testing`."
- Accepts PRs from feature branches.
- After `main` (or `flight-testing`) updates, merge them down into `dev`
  to avoid divergence drift.

---

## Promotion path

```
feature/* ─► dev              (experimental work merges here)
feature/* ─► flight-testing   (validated work merges here)
dev       ─► flight-testing   (graduating a dev feature)
flight-testing ─► main        (after hardware validation passes)
```

Every promotion step is a PR. Use the right PR template for the merge target
(see [.github/PULL_REQUEST_TEMPLATE/](.github/PULL_REQUEST_TEMPLATE/)).

## Sync (downstream) path

After `main` is updated:
```
git checkout flight-testing && git merge main && git push
git checkout dev            && git merge main && git push
```

After `flight-testing` is updated (independent of a `main` merge):
```
git checkout dev && git merge flight-testing && git push
```

The goal: never let `dev` or `flight-testing` get more than a week behind
upstream. Old divergence becomes hard divergence.

---

## What goes where — quick reference

| Change type                                                | Where it starts | Promotes to        |
| ---------------------------------------------------------- | --------------- | ------------------ |
| Bug fix in a flight-proven module                          | `flight-testing` (or `hotfix/*` off `main` if urgent) | `main` |
| Tweak to pyro/FSM/telemetry — needs flight validation      | `flight-testing` | `main` after HW test |
| New sensor driver replacing a working one                  | `dev`           | `flight-testing` once benched, then `main` |
| Brand-new algorithm (e.g. new EKF, attitude, controller)   | `dev`           | `flight-testing` after sim/HIL passes |
| Scaffolding / experimental module (`hse_test`, etc.)       | `dev`           | maybe never        |
| Host-side tools (Python, MATLAB)                           | `dev`           | `main` if they become canonical |
| Documentation only                                         | `main` directly via PR | — |

---

## In-file tier labels

Every file in `Software/App/**` and `Software/Core/Src/main.c` carries an
ASCII header banner identifying its **tier**:

```c
/* ============================================================
 *  TIER:     SAFETY-CRITICAL
 *  MODULE:   Pyro Manager
 *  SUMMARY:  Per-channel arm/continuity/FSM-gated firing.
 * ============================================================ */
```

| TIER value         | Meaning                                                                |
| ------------------ | ---------------------------------------------------------------------- |
| `SAFETY-CRITICAL`  | Failure can endanger flight, hardware, or people. Must have interlocks.|
| `CORE-FLIGHT`      | Required for the flight computer's mission (sense, decide, log, comm). |
| `TEST-HARNESS`     | Diagnostics / calibration / HIL — flight-relevant, mode-gated.         |
| `EXPERIMENTAL`     | Unproven, dev-branch only. May not work.                               |
| `GROUND-STATION`   | Runs on the ground board, not the flight computer.                     |
| `HOST-SIDE`        | Off-target tools (Python, MATLAB).                                     |

The tier is intrinsic to the file's role and does not change with branch.
Maturity (stable / testing / experimental) is implied by which branch you
are on. Files unique to `dev` (e.g. `hse_test/*`) additionally carry:

```c
/* >>>  EXPERIMENTAL — DEV BRANCH ONLY — NOT FLIGHT-CERTIFIED  <<< */
```

When opening any file in any branch, the tier banner tells you at a glance
how careful to be with the change you're about to make.

---

## Hotfix flow

If a bug is found on `main` that needs fixing before the next normal
promotion (e.g. on launch day):

```
git checkout main
git checkout -b hotfix/<short-name>
# ... fix ...
# PR hotfix/<short-name> -> main
# After merge:
git checkout flight-testing && git merge main && git push
git checkout dev            && git merge main && git push
```

Never amend or force-push `main`. Hotfixes are full commits like everything
else.
