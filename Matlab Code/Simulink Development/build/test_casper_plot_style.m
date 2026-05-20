function test_casper_plot_style()
%TEST_CASPER_PLOT_STYLE Smoke test for casper_plot_style.
%   Generates a 2x2 tiled figure exercising title spacing, units in axis
%   labels, sgtitle, legends, and the Wong color-blind palette; writes to
%   ./plots/_style_smoke.png; asserts file exists and is > 50 KB.

    here = fileparts(mfilename('fullpath'));
    out_dir = fullfile(here, 'plots');
    if ~exist(out_dir, 'dir')
        mkdir(out_dir);
    end
    out_path = fullfile(out_dir, '_style_smoke.png');

    t = linspace(0, 10, 1001);
    a1 = sin(2*pi*0.5*t);
    a2 = sin(2*pi*0.5*t + pi/3) * 0.8;
    a3 = sin(2*pi*0.5*t + 2*pi/3) * 0.6;
    b  = exp(-0.2*t) .* cos(2*pi*1.0*t);
    c  = cumsum(randn(1, numel(t))) * 0.1;
    d  = atan2(sin(t), cos(0.5*t)) * 180/pi;

    fig = figure('Visible', 'off');
    tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

    nexttile;
    plot(t, a1, t, a2, t, a3, 'LineWidth', 1.2);
    xlabel('Time [s]'); ylabel('Acceleration [m/s^{2}]');
    title('IMU 3-axis (pad)');
    legend({'X', 'Y', 'Z'}, 'Location', 'best');

    nexttile;
    plot(t, b, 'LineWidth', 1.2);
    xlabel('Time [s]'); ylabel('Innovation [m]');
    title('Baro innovation (mach-shock window)');

    nexttile;
    plot(t, c, 'LineWidth', 1.2);
    xlabel('Time [s]'); ylabel('Bias drift [m/s^{2}]');
    title('Accel bias state');

    nexttile;
    plot(t, d, 'LineWidth', 1.2);
    xlabel('Time [s]'); ylabel('Heading [deg]');
    title('Yaw vs time');

    sgtitle('Plot style smoke test (casper_plot_style)');

    casper_plot_style(fig, struct('WidthIn', 12, 'HeightIn', 8));
    exportgraphics(fig, out_path, 'Resolution', 300);
    close(fig);

    info = dir(out_path);
    assert(~isempty(info), 'PNG was not created at %s', out_path);
    assert(info.bytes > 50e3, 'PNG suspiciously small (%d bytes), expected > 50 KB', info.bytes);

    fprintf('[T0-plot-style] PASS — wrote %s (%d KB)\n', out_path, round(info.bytes/1024));
end
