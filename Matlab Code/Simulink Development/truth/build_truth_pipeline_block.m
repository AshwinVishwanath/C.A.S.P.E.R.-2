function lib_path = build_truth_pipeline_block(varargin)
%BUILD_TRUTH_PIPELINE_BLOCK Programmatically construct the truth_source subsystem.
%
% Synopsis:
%   lib_path = build_truth_pipeline_block()
%   lib_path = build_truth_pipeline_block('LibPath', '/abs/path/casper_sim_lib.slx')
%
% Creates (or updates) a Simulink library at <T01 build dir>/casper_sim_lib.slx
% containing a subsystem 'truth_source' that:
%   - Reads ten scalar/vector signals from a base-workspace struct of
%     timeseries called 'truth_ts' via 'From Workspace' blocks.
%   - Packs them into a Simulink TruthBus via a Bus Creator block.
%   - Exposes a single Outport 'truth_bus' at solver rate.
%
% The companion script test_truth_pipeline.m:
%   1. Loads truth_trajectory.mat
%   2. Builds 'truth_ts' (Simulink struct-of-timeseries) into base workspace
%   3. Runs a 0.1 s sim referencing this library block
%   4. Asserts the first sample equals the raw CSV t=0 values
%
% Outputs:
%   lib_path : char, absolute path to the saved .slx library
%
% Source firmware reference:
%   None. Sim-side wrapper around the truth trajectory cache.

    p = inputParser();
    addParameter(p, 'LibPath', '', @(x) ischar(x) || isstring(x));
    parse(p, varargin{:});
    lib_path = char(p.Results.LibPath);

    if isempty(lib_path)
        here = fileparts(mfilename('fullpath'));
        lib_path = fullfile(here, 'casper_sim_lib.slx');
    end

    lib_name = 'casper_sim_lib';
    sub_name = 'truth_source';

    % Close any open instance so we can re-create deterministically.
    if bdIsLoaded(lib_name)
        close_system(lib_name, 0);
    end

    if isfile(lib_path)
        % Open existing library, remove any prior truth_source.
        load_system(lib_path);
        set_param(lib_name, 'Lock', 'off');
        if getSimulinkBlockHandle([lib_name '/' sub_name]) ~= -1
            delete_block([lib_name '/' sub_name]);
        end
    else
        new_system(lib_name, 'Library');
        load_system(lib_name);
    end

    % Ensure TruthBus is defined in the base workspace before we add the
    % Bus Creator (Simulink needs the type to exist at compile time).
    truth_bus_obj = casper_truth_build_bus();
    assignin('base', 'TruthBus', truth_bus_obj);

    % Build the truth_source subsystem programmatically.
    sub_path = [lib_name '/' sub_name];
    add_block('built-in/Subsystem', sub_path);
    set_param(sub_path, 'Position', [80 60 360 360]);

    % Each TruthBus field has a backing From Workspace block.
    fields = {
        % {bus_field,           dims, var_in_truth_ts}
        {'pos_NED',          3, 'pos_NED'         }, ...
        {'vel_NED',          3, 'vel_NED'         }, ...
        {'accel_NED',        3, 'accel_NED'       }, ...
        {'quat_std',         4, 'quat_std'        }, ...
        {'omega_body_std',   3, 'omega_body_std'  }, ...
        {'time_s',           1, 'time_s'          }, ...
        {'mach',             1, 'mach'            }, ...
        {'air_density_kgm3', 1, 'air_density_kgm3'}, ...
        {'air_temp_K',       1, 'air_temp_K'      }, ...
        {'air_pressure_pa',  1, 'air_pressure_pa' } ...
    };

    nf = numel(fields);
    y_step = 50;
    y0 = 30;

    % From Workspace blocks
    for k = 1:nf
        name = fields{k}{1};
        var  = fields{k}{3};
        fw_path = [sub_path '/FW_' name];
        add_block('simulink/Sources/From Workspace', fw_path);
        set_param(fw_path, ...
            'VariableName',     ['truth_ts.' var], ...
            'SampleTime',       '-1', ...
            'OutputAfterFinalValue', 'Holding final value', ...
            'Interpolate',      'on', ...
            'ZeroCross',        'off');
        ypos = y0 + (k-1) * y_step;
        set_param(fw_path, 'Position', [40 ypos 140 ypos+30]);
    end

    % Bus Creator
    bc_path = [sub_path '/TruthBusCreator'];
    add_block('simulink/Signal Routing/Bus Creator', bc_path);
    set_param(bc_path, ...
        'Inputs',     num2str(nf), ...
        'OutDataTypeStr', 'Bus: TruthBus', ...
        'NonVirtualBus', 'on');
    set_param(bc_path, 'Position', [220 y0 240 y0 + nf * y_step]);

    % Outport
    op_path = [sub_path '/truth_bus'];
    add_block('built-in/Outport', op_path);
    set_param(op_path, 'Position', [310 (y0 + nf * y_step / 2) 340 (y0 + nf * y_step / 2 + 20)]);

    % Wire FW blocks -> Bus Creator. Name each line so Bus Creator picks
    % the right ElementName instead of "signal1..N".
    for k = 1:nf
        name = fields{k}{1};
        lh = add_line(sub_path, ['FW_' name '/1'], ['TruthBusCreator/' num2str(k)], ...
            'autorouting', 'on');
        set_param(lh, 'Name', name);
    end
    % Bus Creator -> Outport
    lh_out = add_line(sub_path, 'TruthBusCreator/1', 'truth_bus/1', 'autorouting', 'on');
    set_param(lh_out, 'Name', 'truth_bus');

    % Lock the library and save.
    set_param(lib_name, 'Lock', 'on');
    save_system(lib_name, lib_path);
    close_system(lib_name, 0);

    fprintf('[build_truth_pipeline_block] wrote %s\n', lib_path);
end
