function fig = plot_lap_result(result)
%PLOT_LAP_RESULT Plot speed, acceleration, curvature, and limiter versus s.

arguments
    result (1,1) struct
end

fig = figure(Name="QSSLTS lap result");
layout = tiledlayout(fig, 5, 1, TileSpacing="compact", Padding="compact");

nexttile(layout);
plot(result.s_m, result.v_mps, LineWidth=1.3);
grid on; ylabel("v [m/s]");

nexttile(layout);
plot(result.s_m, result.ax_mps2, LineWidth=1.3);
grid on; ylabel("a_x [m/s^2]");

nexttile(layout);
plot(result.s_m, result.ay_mps2, LineWidth=1.3);
grid on; ylabel("a_y [m/s^2]");

nexttile(layout);
plot(result.s_m, result.track.kappa_1pm, LineWidth=1.3);
grid on; ylabel("\kappa [1/m]");

nexttile(layout);
[limiterNames, ~, limiterCode] = unique(string(result.limiter), "stable");
stairs(result.s_m, limiterCode, LineWidth=1.3);
grid on; ylabel("limiter"); xlabel("s [m]");
yticks(1:numel(limiterNames));
yticklabels(limiterNames);
end
