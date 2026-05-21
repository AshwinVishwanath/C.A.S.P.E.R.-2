function [done, init_out, state_out] = casper_attitude_static_init( ...
        accel_body_fw_mps2, mag_body_fw_uT, mag_new_sample, dt_s, params, state_in)
%CASPER_ATTITUDE_STATIC_INIT  Pad-state accel/mag averaging + initial quat.
%
%   Mirrors casper_attitude.c casper_att_static_init():
%     - Accumulate accel at every call (833 Hz).
%     - Accumulate mag only when mag_new_sample == true (~100 Hz).
%     - init_elapsed += 1/833 each call (firmware uses fixed 1/833, NOT dt_s)
%       to match the firmware's literal dt assumption. We use dt_s instead
%       here for sim flexibility but default to 1/833 if not supplied.
%     - Completion: mag_count >= StaticInitSamples (500) OR
%                   init_elapsed >= StaticInitTimeout_s (10).
%     - On completion: compute pitch/roll from gravity, then yaw from
%       tilt-compensated mag (if available); build quaternion via Hamilton
%       ZYX; compute m_ref_ned = R_b2n * mag_avg.
%     - Phase 0 also freezes gyro_bias = mean of first 100 gyro samples.
%       However firmware does NOT do this in casper_att_init — bias starts
%       at zero and is updated by EMA. Spec §5.3 calls out the static-bias
%       init, so we do it here (gyro_bias is in `state_in.bias_*`).
%
%   Inputs:
%     accel_body_fw_mps2  (3x1)  body-frame accel, m/s^2 (firmware-frame)
%     mag_body_fw_uT      (3x1)  body-frame mag, uT (firmware-frame), only
%                                used if mag_new_sample
%     mag_new_sample      (bool) true when a new mag sample arrived
%     dt_s                (1x1)  call interval, s (firmware uses 1/833)
%     params              struct fields used:
%       .StaticInitSamples       (e.g. 500)
%       .StaticInitTimeout_s     (e.g. 10.0)
%       .M_ref_nav_uT            (3x1) expected nav-frame mag (Z-up), uT
%                                  If supplied, used IF mag never arrived
%                                  (fallback). Not strictly needed if mag works.
%     state_in            struct fields:
%       .accel_sum_mps2          (3x1) cumulative accel
%       .accel_count             (scalar)
%       .mag_sum_uT              (3x1) cumulative mag
%       .mag_count               (scalar)
%       .init_elapsed_s          (scalar)
%       .gyro_bias_sum_radps     (3x1)
%       .gyro_bias_count         (scalar)
%
%   Outputs:
%     done       (bool) true when init complete this tick
%     init_out   struct (only populated when done == true):
%       .q_body_to_nav          (4x1) Hamilton quaternion
%       .m_ref_nav_uT           (3x1) reference mag in nav-frame (Z-up)
%       .mag_available          (bool)
%     state_out  struct (updated accumulators)

    assert(numel(accel_body_fw_mps2) == 3, 'accel must be 3x1');
    accel_body_fw_mps2 = accel_body_fw_mps2(:);
    if mag_new_sample
        assert(numel(mag_body_fw_uT) == 3, 'mag must be 3x1 when new sample');
        mag_body_fw_uT = mag_body_fw_uT(:);
    end

    % Defaults / state init
    if nargin < 6 || isempty(state_in)
        state_in = struct( ...
            'accel_sum_mps2',     zeros(3,1), ...
            'accel_count',        0, ...
            'mag_sum_uT',         zeros(3,1), ...
            'mag_count',          0, ...
            'init_elapsed_s',     0, ...
            'gyro_bias_sum_radps', zeros(3,1), ...
            'gyro_bias_count',    0);
    end

    init_out = struct( ...
        'q_body_to_nav', [1;0;0;0], ...
        'm_ref_nav_uT',  zeros(3,1), ...
        'mag_available', false);

    state_out = state_in;
    state_out.accel_sum_mps2 = state_in.accel_sum_mps2 + accel_body_fw_mps2;
    state_out.accel_count    = state_in.accel_count + 1;
    if mag_new_sample
        state_out.mag_sum_uT = state_in.mag_sum_uT + mag_body_fw_uT;
        state_out.mag_count  = state_in.mag_count + 1;
    end
    state_out.init_elapsed_s = state_in.init_elapsed_s + dt_s;

    done_mag = (state_out.mag_count >= params.StaticInitSamples);
    timeout  = (state_out.init_elapsed_s >= params.StaticInitTimeout_s);

    if ~done_mag && ~timeout
        done = false;
        return;
    end

    % ── Compute initial attitude ─────────────────────────────────────────
    ops = casper_quat_ops();

    % Accel average → gravity direction
    accel_avg_mps2 = state_out.accel_sum_mps2 / state_out.accel_count;

    ax = accel_avg_mps2(1);
    ay = accel_avg_mps2(2);
    az = accel_avg_mps2(3);

    % Matches casper_attitude.c L152-154 exactly
    pitch_rad = atan2(-ax, sqrt(ay*ay + az*az));
    roll_rad  = atan2( ay, az);

    if done_mag && state_out.mag_count > 0
        mag_avg_uT = state_out.mag_sum_uT / state_out.mag_count;

        % Tilt-compensated heading (matches casper_attitude.c L165-171)
        cp = cos(pitch_rad); sp = sin(pitch_rad);
        cr = cos(roll_rad);  sr = sin(roll_rad);
        mx = mag_avg_uT(1); my = mag_avg_uT(2); mz = mag_avg_uT(3);

        mx_h = mx*cp + my*sr*sp + mz*cr*sp;
        my_h = my*cr - mz*sr;
        yaw_rad = atan2(-my_h, mx_h);

        q = ops.from_euler(roll_rad, pitch_rad, yaw_rad);

        % m_ref_nav = R_b2n * mag_avg_body
        R = ops.to_dcm(q);
        m_ref_nav_uT = R * mag_avg_uT;
        mag_avail = true;
    else
        % Timeout fallback: gravity-only init
        q = ops.from_euler(roll_rad, pitch_rad, 0);
        if isfield(params, 'M_ref_nav_uT') && ~isempty(params.M_ref_nav_uT)
            m_ref_nav_uT = params.M_ref_nav_uT(:);
        else
            m_ref_nav_uT = zeros(3,1);
        end
        mag_avail = false;
    end

    init_out.q_body_to_nav = q;
    init_out.m_ref_nav_uT  = m_ref_nav_uT;
    init_out.mag_available = mag_avail;

    done = true;
end
