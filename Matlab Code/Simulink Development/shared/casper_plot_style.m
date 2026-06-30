function casper_plot_style(fig, opts)
%CASPER_PLOT_STYLE Apply unified Phase 0 plot style to a figure.
%
%   casper_plot_style()           applies to gcf with defaults
%   casper_plot_style(fig)        applies to specified figure handle
%   casper_plot_style(fig, opts)  applies with overrides
%
%   opts (optional struct, all fields optional):
%       .WidthIn       figure width in inches (default 10)
%       .HeightIn      figure height in inches (default 6)
%       .FontName      axes font (default 'Helvetica')
%       .AxesFontSize  axes label/tick font size (default 10)
%       .TitleFontSize per-axes title font size (default 12)
%       .SgTitleFontSize sgtitle font size if present (default 14)
%
%   Applies:
%     - White figure background, configurable inches-based sizing
%     - Helvetica 10 pt axes ticks/labels, 12 pt bold titles, 14 pt sgtitle
%     - Tick direction out, no top/right axes box, light grid (alpha 0.25)
%     - Title interpreter 'none' (so underscores aren't subscripted)
%     - Wong 8-color color-blind-safe palette as default ColorOrder
%
%   Intended usage (every task plot generator):
%       fig = figure;
%       tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
%       % ... nexttile / plot / xlabel('Time [s]') / ylabel('Altitude [m]') ...
%       sgtitle('Phase 0 — Altitude tracking');
%       casper_plot_style(fig);
%       exportgraphics(fig, 'plots/altitude.png', 'Resolution', 300);
%
%   See plan: Matlab Code/Simulink Development/docs (plot
%   formatting standards section).

    if nargin < 1 || isempty(fig)
        fig = gcf;
    end
    if nargin < 2
        opts = struct();
    end

    % Defaults
    width_in        = getfield_or(opts, 'WidthIn',        10);
    height_in       = getfield_or(opts, 'HeightIn',        6);
    font_name       = getfield_or(opts, 'FontName',       'Helvetica');
    axes_font_size  = getfield_or(opts, 'AxesFontSize',   10);
    title_font_size = getfield_or(opts, 'TitleFontSize',  12);
    sg_font_size    = getfield_or(opts, 'SgTitleFontSize', 14);

    % Figure-level style
    set(fig, 'Color',             'w', ...
             'Units',             'inches', ...
             'Position',          [1 1 width_in height_in], ...
             'PaperPositionMode', 'auto');

    % Wong 8-color color-blind-safe palette
    cb = [0.000 0.447 0.741;   % blue
          0.851 0.325 0.098;   % vermillion
          0.929 0.694 0.125;   % yellow
          0.494 0.184 0.556;   % purple
          0.466 0.674 0.188;   % bluish green
          0.301 0.745 0.933;   % sky blue
          0.635 0.078 0.184;   % reddish purple
          0.000 0.000 0.000];  % black
    set(fig, 'DefaultAxesColorOrder', cb);

    % Per-axes style
    ax_list = findall(fig, 'Type', 'axes');
    for k = 1:numel(ax_list)
        ax = ax_list(k);
        % Skip legend / colorbar axes (those are 'Tag'd internally)
        if isprop(ax, 'Tag') && (strcmp(ax.Tag, 'legend') || strcmp(ax.Tag, 'Colorbar'))
            continue;
        end
        set(ax, 'FontName',  font_name, ...
                'FontSize',  axes_font_size, ...
                'LineWidth', 0.75, ...
                'TickDir',   'out', ...
                'Box',       'off', ...
                'ColorOrder', cb);
        grid(ax, 'on');
        try
            ax.GridAlpha = 0.25;
            ax.MinorGridAlpha = 0.15;
        catch
            % Older release without these props: ignore.
        end
        % Title restyling (preserve text, just reformat)
        if ~isempty(ax.Title) && ~isempty(ax.Title.String)
            set(ax.Title, 'FontName',    font_name, ...
                          'FontSize',    title_font_size, ...
                          'FontWeight',  'bold', ...
                          'Interpreter', 'none');
        end
        % Axis label restyling
        if ~isempty(ax.XLabel) && ~isempty(ax.XLabel.String)
            set(ax.XLabel, 'FontName', font_name, 'FontSize', axes_font_size, ...
                           'Interpreter', 'tex');
        end
        if ~isempty(ax.YLabel) && ~isempty(ax.YLabel.String)
            set(ax.YLabel, 'FontName', font_name, 'FontSize', axes_font_size, ...
                           'Interpreter', 'tex');
        end
        if ~isempty(ax.ZLabel) && ~isempty(ax.ZLabel.String)
            set(ax.ZLabel, 'FontName', font_name, 'FontSize', axes_font_size, ...
                           'Interpreter', 'tex');
        end
    end

    % sgtitle restyling (if present)
    tcl_list = findall(fig, 'Type', 'tiledlayout');
    for k = 1:numel(tcl_list)
        tcl = tcl_list(k);
        if isprop(tcl, 'Title') && ~isempty(tcl.Title) && ~isempty(tcl.Title.String)
            set(tcl.Title, 'FontName',    font_name, ...
                           'FontSize',    sg_font_size, ...
                           'FontWeight',  'bold', ...
                           'Interpreter', 'none');
        end
    end
    % Also catch direct sgtitle handles on figure
    sg = findall(fig, 'Tag', 'sgtitle');
    for k = 1:numel(sg)
        set(sg(k), 'FontName',    font_name, ...
                   'FontSize',    sg_font_size, ...
                   'FontWeight',  'bold', ...
                   'Interpreter', 'none');
    end

    % Legend restyling (if any)
    lg_list = findall(fig, 'Type', 'legend');
    for k = 1:numel(lg_list)
        set(lg_list(k), 'FontName', font_name, ...
                        'FontSize', axes_font_size, ...
                        'Box',      'off', ...
                        'Color',    'none');
    end
end

function val = getfield_or(s, field, default)
    if isfield(s, field)
        val = s.(field);
    else
        val = default;
    end
end
