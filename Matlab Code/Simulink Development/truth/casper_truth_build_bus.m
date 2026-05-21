function bus = casper_truth_build_bus()
%CASPER_TRUTH_BUILD_BUS Return a Simulink.Bus object for TruthBus.
%
% Synopsis:
%   bus = casper_truth_build_bus()
%
% Returns a Simulink.Bus object describing the TruthBus layout in
% SIMULINK_PATTERNS.md §7. The caller is responsible for placing it into
% the base workspace (or in a Data Dictionary) under the name 'TruthBus':
%
%   bus = casper_truth_build_bus();
%   assignin('base', 'TruthBus', bus);
%
% Fields:
%   pos_NED          (3x1 double, meters)
%   vel_NED          (3x1 double, m/s)
%   accel_NED        (3x1 double, m/s^2)
%   quat_std         (4x1 double, scalar-first body-to-NED)
%   omega_body_std   (3x1 double, rad/s)
%   time_s           (1x1 double, seconds)
%   mach             (1x1 double)
%   air_density_kgm3 (1x1 double)
%   air_temp_K       (1x1 double)
%   air_pressure_pa  (1x1 double)
%
% Source firmware reference:
%   None (sim-side bus layout).

    fields = { ...
        % {name,             dims, units,            description}
        {'pos_NED',          3,    'm',              'world-frame NED position'}, ...
        {'vel_NED',          3,    'm/s',            'world-frame NED velocity'}, ...
        {'accel_NED',        3,    'm/s^2',          'world-frame NED net accel (gravity-free)'}, ...
        {'quat_std',         4,    '',               'body-to-NED quaternion, scalar-first [w x y z]'}, ...
        {'omega_body_std',   3,    'rad/s',          'body-frame angular rate'}, ...
        {'time_s',           1,    's',              'simulation time'}, ...
        {'mach',             1,    '',               'Mach number'}, ...
        {'air_density_kgm3', 1,    'kg/m^3',         'air density'}, ...
        {'air_temp_K',       1,    'K',              'air temperature'}, ...
        {'air_pressure_pa',  1,    'Pa',             'air pressure'} ...
    };

    elems(numel(fields), 1) = Simulink.BusElement();
    for k = 1:numel(fields)
        e = Simulink.BusElement();
        e.Name        = fields{k}{1};
        e.Dimensions  = fields{k}{2};
        e.DataType    = 'double';
        e.SampleTime  = -1;
        e.Complexity  = 'real';
        e.Description = fields{k}{4};
        e.DocUnits    = fields{k}{3};
        e.DimensionsMode = 'Fixed';
        elems(k) = e;
    end

    bus = Simulink.Bus();
    bus.Description = 'TruthBus for force-posed Phase 0 sim';
    bus.Elements    = elems;
end
