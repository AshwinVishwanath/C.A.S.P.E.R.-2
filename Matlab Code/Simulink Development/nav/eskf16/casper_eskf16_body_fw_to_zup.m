function vec_zup = casper_eskf16_body_fw_to_zup(vec_fw)
%CASPER_ESKF16_BODY_FW_TO_ZUP Rotate a body-fw vector into EKF16's body-Zup
% convention.
%
% Convention map (verified by tracing the visual-model accel chain):
%   - imuSensor outputs accel_g with the industry-standard "+g on the
%     axis pointing UP against gravity" on the pad.
%   - imu_unit_convert in build_casper_sim_phase0 flips that sign
%     (`a_std = -accel_g * 9.80665`) and converts to m/s^2.
%   - FrameSwitch_Accel runs casper_frame_switch_body, which gives
%     `vec_fw = [v(2); v(1); -v(3)]`.
%
%   Combined: imuSensor pad +g on body Z  ->  std accel = [0, 0, -g]
%             after frame_switch_body     ->  fw accel  = [0, 0, +g]
%
%   So at the FrameSwitch_Accel output the convention is "+g on Z" on
%   the pad — which is the SAME convention as EKF16. The mapping is
%   identity.
%
% On-pad sanity:
%   fw accel  = [0, 0, +g]
%   zup accel = [0, 0, +g]
%
% NOTE: This contradicts the legacy MATLAB driver's pad assertion
% `expect_a = [0, +g, 0]` (Y = nose = up). That assertion is derived
% from casper_imu_lsm_model which does NOT sign-flip — i.e., it returns
% raw specific force. The visual model deliberately re-engineers the
% chain to apply the +g-up-on-Z convention via the negation in
% imu_unit_convert, so the EKF16 sees +g on Z just like the symbolic
% derivation expects.
%
% Inputs:
%   vec_fw  : 3x1 (any units)
%
% Outputs:
%   vec_zup : 3x1 (same units)

    v = double(vec_fw(:));
    vec_zup = v;
end
