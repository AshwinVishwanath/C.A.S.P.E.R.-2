function result = casper_metric_determinism(RunA, RunB)
%CASPER_METRIC_DETERMINISM Two-run byte-compare for sensor + estimate streams.
%
% Synopsis:
%   result = casper_metric_determinism(RunA, RunB)
%
% Inputs:
%   RunA, RunB : structs with fields
%       sensor_streams (any nested struct/array)
%       estimate       (any nested struct/array)
%
% Outputs:
%   result : MetricResult struct
%       value.sensor_hash_a, .sensor_hash_b
%       value.estimate_hash_a, .estimate_hash_b
%       pass = (sensor_hash_a == sensor_hash_b) &&
%              (estimate_hash_a == estimate_hash_b)
%
% Hash is computed via casper_data_hash() (MD5 of byte-stream-of-array).

    arguments
        RunA struct
        RunB struct
    end

    hash_sensor_a   = casper_data_hash(RunA.sensor_streams);
    hash_sensor_b   = casper_data_hash(RunB.sensor_streams);
    hash_estimate_a = casper_data_hash(RunA.estimate);
    hash_estimate_b = casper_data_hash(RunB.estimate);

    pass_sensor   = isequal(hash_sensor_a,   hash_sensor_b);
    pass_estimate = isequal(hash_estimate_a, hash_estimate_b);
    pass_all      = pass_sensor && pass_estimate;

    val = struct( ...
        'sensor_hash_a',   hash_sensor_a, ...
        'sensor_hash_b',   hash_sensor_b, ...
        'estimate_hash_a', hash_estimate_a, ...
        'estimate_hash_b', hash_estimate_b, ...
        'sensor_match',    pass_sensor, ...
        'estimate_match',  pass_estimate);

    thr = struct('match', 'identical');

    details = sprintf( ...
        'sensor[%s vs %s] estimate[%s vs %s]', ...
        short_hash(hash_sensor_a),   short_hash(hash_sensor_b), ...
        short_hash(hash_estimate_a), short_hash(hash_estimate_b));

    result = struct( ...
        'name',      'determinism', ...
        'value',     val, ...
        'threshold', thr, ...
        'pass',      pass_all, ...
        'details',   details);
end

function s = short_hash(h)
    if ischar(h) || isstring(h)
        h = char(h);
        if numel(h) >= 8
            s = h(1:8);
        else
            s = h;
        end
    else
        s = '<n/a>';
    end
end
