function cap = calc_ts_power_cap(Vbus_V, powertrain)
%CALC_TS_POWER_CAP Calculate TSAC-outlet and mechanical power caps.
%   All current limits in this function are DC limits. Auxiliary power is
%   removed after the TSAC cap, before inverter and motor efficiencies.

arguments
    Vbus_V double {mustBeFinite, mustBeNonnegative}
    powertrain (1,1) struct
end

powertrain = validate_powertrain_config(powertrain);
Vbus_V = Vbus_V(:);
effectiveVoltage_V = min([Vbus_V, ...
    repmat(powertrain.rules.max_ts_voltage_V, numel(Vbus_V), 1), ...
    repmat(powertrain.battery.V_max_V, numel(Vbus_V), 1), ...
    repmat(powertrain.inverter.V_dc_max_V, numel(Vbus_V), 1)], [], 2);

rulePower_W = repmat(powertrain.rules.max_ts_power_W, numel(Vbus_V), 1);
ruleCurrent_W = effectiveVoltage_V * powertrain.rules.max_ts_current_A;
batteryPower_W = repmat( ...
    powertrain.battery.P_discharge_peak_W, numel(Vbus_V), 1);
batteryCurrent_W = effectiveVoltage_V ...
    * powertrain.battery.I_discharge_peak_A;
inverterPower_W = repmat( ...
    powertrain.inverter.P_dc_peak_W, numel(Vbus_V), 1);
inverterCurrent_W = effectiveVoltage_V ...
    * powertrain.inverter.I_dc_peak_A;

candidatePower_W = [rulePower_W, ruleCurrent_W, batteryPower_W, ...
    batteryCurrent_W, inverterPower_W, inverterCurrent_W];
candidateLabels = ["rule_power", "rule_current", "battery_power", ...
    "battery_current", "inverter_power", "inverter_current"];
[tsacPowerCap_W, index] = min(candidatePower_W, [], 2);
netInverterInput_W = max(tsacPowerCap_W ...
    - powertrain.battery.P_ts_aux_W, 0);
mechanicalPowerCap_W = netInverterInput_W ...
    * powertrain.inverter.eta_const * powertrain.motor.eta_const;

cap.effective_voltage_V = effectiveVoltage_V;
cap.power_candidates_W.rule_power = rulePower_W;
cap.power_candidates_W.rule_current = ruleCurrent_W;
cap.power_candidates_W.battery_power = batteryPower_W;
cap.power_candidates_W.battery_current = batteryCurrent_W;
cap.power_candidates_W.inverter_power = inverterPower_W;
cap.power_candidates_W.inverter_current = inverterCurrent_W;
cap.candidate_labels = candidateLabels;
cap.tsac_power_cap_W = tsacPowerCap_W;
cap.limiter = candidateLabels(index).';
cap.aux_power_W = powertrain.battery.P_ts_aux_W;
cap.inverter_input_power_cap_W = netInverterInput_W;
cap.mechanical_power_cap_W = mechanicalPowerCap_W;
cap.dc_current_limits_A.rule = powertrain.rules.max_ts_current_A;
cap.dc_current_limits_A.battery = ...
    powertrain.battery.I_discharge_peak_A;
cap.dc_current_limits_A.inverter = powertrain.inverter.I_dc_peak_A;
cap.tsac_dc_current_at_cap_A = tsacPowerCap_W ...
    ./ max(effectiveVoltage_V, eps);
cap.battery_efficiency_for_energy_model = powertrain.battery.eta_discharge;
end
