function tx_active = casper_radio_tx_schedule(time_s, ...
                                                tx_period_s, ...
                                                tx_airtime_s)
%CASPER_RADIO_TX_SCHEDULE Deterministic TX-active boolean clock.
%
% Synopsis:
%   tx_active = casper_radio_tx_schedule(time_s, tx_period_s, tx_airtime_s)
%
% Inputs:
%   time_s       : scalar double, current sim time [s].
%   tx_period_s  : scalar double, TX cadence period [s] (e.g. 0.100 = 10 Hz).
%   tx_airtime_s : scalar double, TX airtime per event [s] (e.g. 0.015 SF7).
%
% Outputs:
%   tx_active : scalar logical, true while a TX is in progress.
%
% Semantics (T05 §6.1):
%   - TX events anchored at t = 0, tx_period_s, 2*tx_period_s, ...
%   - Each TX is rectangular and lasts tx_airtime_s.
%   - tx_active = (mod(time_s, tx_period_s) < tx_airtime_s).
%   - No jitter in Phase 0; deterministic for reproducibility.
%
% Numerical robustness:
%   To avoid double-precision modular drift after many periods, snap the
%   phase to a 1 µs grid before comparison. This matches the truth-stream
%   solver step (10 kHz = 100 µs) and the mag sample rate (100 Hz = 10 ms)
%   with several decades of margin.
%
% Source firmware reference:
%   Software/App/radio/radio_manager.c (10 Hz TX cadence).
%   Software/App/radio/radio_config.h RADIO_TX_PERIOD_MS = 100.

    %#codegen
    if tx_period_s <= 0
        tx_active = false;
        return;
    end
    if tx_airtime_s <= 0
        tx_active = false;
        return;
    end

    phase_s    = mod(time_s, tx_period_s);
    phase_snap = round(phase_s * 1e6) * 1e-6;
    tx_active  = phase_snap < tx_airtime_s;
end
