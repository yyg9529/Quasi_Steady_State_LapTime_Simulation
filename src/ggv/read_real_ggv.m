function ggvReal = read_real_ggv(filename)
%READ_REAL_GGV Read a gridded real-vehicle GGV CSV.
%   Required columns are v_mps, ay_g, ax_max_g, ax_min_g. Every listed
%   speed/lateral pair must be unique; missing grid points remain infeasible.

arguments
    filename (1,1) string
end

if ~isfile(filename)
    error("QSSLTS:RealGGVFileNotFound", ...
        "Real GGV file not found: %s", filename);
end

inputTable = readtable(filename, TextType="string", ...
    VariableNamingRule="preserve");
required = ["v_mps", "ay_g", "ax_max_g", "ax_min_g"];
if ~all(ismember(required, string(inputTable.Properties.VariableNames)))
    error("QSSLTS:RealGGVColumns", ...
        "Real GGV CSV must contain v_mps,ay_g,ax_max_g,ax_min_g.");
end
if any(~isfinite(inputTable{:, cellstr(required)}), "all")
    error("QSSLTS:RealGGVFinite", "Real GGV values must be finite.");
end

vGrid_mps = unique(inputTable.v_mps(:), "sorted");
ayGrid_g = unique(inputTable.ay_g(:), "sorted").';
if numel(vGrid_mps) < 2 || numel(ayGrid_g) < 3 ...
        || ayGrid_g(1) >= 0 || ayGrid_g(end) <= 0
    error("QSSLTS:RealGGVGrid", ...
        "Real GGV must have >=2 speeds and signed lateral coverage.");
end

[foundSpeed, speedIndex] = ismember(inputTable.v_mps, vGrid_mps);
[foundLateral, lateralIndex] = ismember(inputTable.ay_g, ayGrid_g);
linearIndex = sub2ind([numel(vGrid_mps), numel(ayGrid_g)], ...
    speedIndex, lateralIndex);
if ~all(foundSpeed) || ~all(foundLateral) ...
        || numel(unique(linearIndex)) ~= height(inputTable)
    error("QSSLTS:RealGGVDuplicate", ...
        "Real GGV speed/lateral pairs must be unique.");
end

axMax_g = nan(numel(vGrid_mps), numel(ayGrid_g));
axMin_g = nan(numel(vGrid_mps), numel(ayGrid_g));
axMax_g(linearIndex) = inputTable.ax_max_g;
axMin_g(linearIndex) = inputTable.ax_min_g;
feasible = isfinite(axMax_g) & isfinite(axMin_g);

ayLimitPos_g = zeros(numel(vGrid_mps), 1);
ayLimitNeg_g = zeros(numel(vGrid_mps), 1);
for iSpeed = 1:numel(vGrid_mps)
    validAy_g = ayGrid_g(feasible(iSpeed, :));
    if isempty(validAy_g) || ~any(validAy_g >= 0) || ~any(validAy_g <= 0)
        error("QSSLTS:RealGGVCoverage", ...
            "Each real-GGV speed must include positive and negative ay.");
    end
    ayLimitPos_g(iSpeed) = max(validAy_g);
    ayLimitNeg_g(iSpeed) = min(validAy_g);
end

accelLimiter = repmat("measured", size(feasible));
brakeLimiter = repmat("measured", size(feasible));
accelLimiter(~feasible) = "lateral_infeasible";
brakeLimiter(~feasible) = "lateral_infeasible";

ggvReal.v_mps = vGrid_mps;
ggvReal.ay_g = ayGrid_g;
ggvReal.ax_max_g = axMax_g;
ggvReal.ax_min_g = axMin_g;
ggvReal.feasible = feasible;
ggvReal.ay_limit_pos_g = ayLimitPos_g;
ggvReal.ay_limit_neg_g = ayLimitNeg_g;
ggvReal.accel_limiter = accelLimiter;
ggvReal.brake_limiter = brakeLimiter;
ggvReal.gravity_mps2 = 9.80665;
ggvReal.source = "real_csv";
ggvReal.source_file = filename;
end
