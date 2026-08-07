function data = qsslts_kpi_data(result, summary)
%QSSLTS_KPI_DATA Format lap or dynamic-event result cards.

arguments
    result (1,1) struct
    summary (1,1) struct
end

if isfield(result, "event")
    timeLabel = "Event time";
    timeMeta = "赛事计时时间";
    energyLabel = "Path energy";
    energyMeta = "完整开放路径储能侧能量";
else
    timeLabel = "Lap time";
    timeMeta = "固定赛线单圈时间";
    energyLabel = "Lap energy";
    energyMeta = "储能侧单圈能量";
end

if isfield(result, "energy") ...
        && isfield(result.energy, "E_lap_stored_kWh")
    energy_kWh = result.energy.E_lap_stored_kWh;
elseif isfield(result, "energy") ...
        && isfield(result.energy, "E_lap_ts_kWh")
    energy_kWh = result.energy.E_lap_ts_kWh;
else
    energy_kWh = NaN;
end
if isfinite(energy_kWh)
    energyText = sprintf("%.3f", energy_kWh);
else
    energyText = "—";
end

if result.solver.converged
    convergenceText = sprintf("已收敛 · %d iter", ...
        result.solver.iterations);
else
    convergenceText = sprintf("未收敛 · %d iter", ...
        result.solver.iterations);
end

data = struct( ...
    lapTime=sprintf("%.3f", summary.lap_time_s), ...
    maxSpeed=sprintf("%.2f", summary.max_speed_mps), ...
    lapEnergy=energyText, convergence=convergenceText, ...
    timeLabel=timeLabel, timeMeta=timeMeta, ...
    energyLabel=energyLabel, energyMeta=energyMeta);
end
