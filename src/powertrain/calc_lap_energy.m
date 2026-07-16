function energy = calc_lap_energy( ...
        track, v_mps, vehicle, aero, powertrain, endurance)
%CALC_LAP_ENERGY Estimate lap and repeated-lap TSAC energy.
%   Uses fixed DC voltage, constant efficiencies, no regeneration, and no
%   thermal derating. Closed tracks include the final N-to-1 segment.

arguments
    track (1,1) struct
    v_mps double {mustBeFinite, mustBeNonnegative}
    vehicle (1,1) struct
    aero (1,1) struct
    powertrain (1,1) struct
    endurance (1,1) struct
end

powertrain = validate_powertrain_config(powertrain);
validateInputs(track, v_mps, vehicle, endurance);
v_mps = v_mps(:);
ds_m = track.ds_m(:);

if logical(track.is_closed)
    segmentStartSpeed_mps = v_mps;
    segmentEndSpeed_mps = circshift(v_mps, -1);
else
    segmentStartSpeed_mps = v_mps(1:end-1);
    segmentEndSpeed_mps = v_mps(2:end);
end

segmentAx_mps2 = (segmentEndSpeed_mps.^2 ...
    - segmentStartSpeed_mps.^2) ./ (2 * ds_m);
segmentSpeedMean_mps = 0.5 * (segmentStartSpeed_mps ...
    + segmentEndSpeed_mps);
if any(segmentSpeedMean_mps <= 0)
    error("QSSLTS:EnergyZeroSpeedSegment", ...
        "Positive mean speed is required for every energy segment.");
end

aeroForce = calc_aero_forces(segmentSpeedMean_mps, aero);
segmentDriveForce_N = max(vehicle.mass.total_kg * segmentAx_mps2 ...
    + aeroForce.drag_N, 0);
segmentWheelPower_W = segmentDriveForce_N .* segmentSpeedMean_mps;
efficiency = powertrain.drivetrain_efficiency ...
    * powertrain.inverter.eta_const * powertrain.motor.eta_const;
segmentTsacPower_W = segmentWheelPower_W / efficiency ...
    + powertrain.battery.P_ts_aux_W;
segmentTime_s = ds_m ./ segmentSpeedMean_mps;
segmentEnergyTs_kWh = segmentTsacPower_W .* segmentTime_s / 3.6e6;
segmentEnergyStored_kWh = segmentEnergyTs_kWh ...
    / powertrain.battery.eta_discharge;

lapTs_kWh = sum(segmentEnergyTs_kWh);
lapStored_kWh = sum(segmentEnergyStored_kWh);
enduranceStored_kWh = lapStored_kWh * endurance.num_laps ...
    * endurance.safety_factor;
socWindow = powertrain.battery.SOC_init - powertrain.battery.SOC_min;
nominalRequired_kWh = enduranceStored_kWh / socWindow;
socEnd = powertrain.battery.SOC_init ...
    - enduranceStored_kWh / powertrain.battery.E_nominal_kWh;

energy.segment_ax_mps2 = segmentAx_mps2;
energy.segment_speed_mean_mps = segmentSpeedMean_mps;
energy.segment_drive_force_N = segmentDriveForce_N;
energy.segment_wheel_power_W = segmentWheelPower_W;
energy.segment_tsac_power_W = segmentTsacPower_W;
energy.segment_energy_ts_kWh = segmentEnergyTs_kWh;
energy.segment_energy_stored_kWh = segmentEnergyStored_kWh;
energy.cumulative_energy_ts_kWh = [0; cumsum(segmentEnergyTs_kWh)];
energy.cumulative_energy_stored_kWh = ...
    [0; cumsum(segmentEnergyStored_kWh)];
energy.E_lap_ts_kWh = lapTs_kWh;
energy.E_lap_stored_kWh = lapStored_kWh;
energy.E_endurance_stored_kWh = enduranceStored_kWh;
energy.E_nominal_required_kWh = nominalRequired_kWh;
energy.SOC_end_estimated = socEnd;
socTolerance = 1e-12 * max(1, abs(powertrain.battery.SOC_min));
energy.can_finish_endurance_estimated = socEnd ...
    >= powertrain.battery.SOC_min - socTolerance;
end

function validateInputs(track, v_mps, vehicle, endurance)
requiredTrack = ["s_m", "ds_m", "kappa_1pm", "is_closed"];
if ~all(isfield(track, cellstr(requiredTrack))) ...
        || ~isscalar(track.is_closed) ...
        || ~ismember(double(track.is_closed), [0, 1])
    error("QSSLTS:EnergyTrack", "Track fields are invalid or incomplete.");
end

nPoint = numel(track.s_m);
nSegment = nPoint - 1 + double(logical(track.is_closed));
if numel(v_mps) ~= nPoint || numel(track.kappa_1pm) ~= nPoint ...
        || numel(track.ds_m) ~= nSegment || any(track.ds_m <= 0)
    error("QSSLTS:EnergyTrack", ...
        "Track, speed, and segment dimensions are inconsistent.");
end
if ~isfield(vehicle, "mass") ...
        || ~isfield(vehicle.mass, "total_kg") ...
        || ~isscalar(vehicle.mass.total_kg) ...
        || ~isfinite(vehicle.mass.total_kg) ...
        || vehicle.mass.total_kg <= 0
    error("QSSLTS:VehicleParameters", ...
        "A positive finite vehicle mass is required.");
end

requiredEndurance = ["num_laps", "safety_factor"];
if ~all(isfield(endurance, cellstr(requiredEndurance))) ...
        || ~isscalar(endurance.num_laps) ...
        || ~isfinite(endurance.num_laps) ...
        || endurance.num_laps <= 0 ...
        || endurance.num_laps ~= fix(endurance.num_laps) ...
        || ~isscalar(endurance.safety_factor) ...
        || ~isfinite(endurance.safety_factor) ...
        || endurance.safety_factor < 1
    error("QSSLTS:EnduranceConfig", ...
        "num_laps must be a positive integer and safety_factor >= 1.");
end
end
