function envelope = calc_motor_voltage_envelope( ...
        motorSpeed_rpm, Vbus_V, powertrain)
%CALC_MOTOR_VOLTAGE_ENVELOPE Infer a conservative voltage-aware envelope.
%   This continuous scaling is an engineering inference, not a supplier
%   torque-speed curve at the requested bus voltage.

arguments
    motorSpeed_rpm double {mustBeFinite, mustBeNonnegative}
    Vbus_V double {mustBeFinite, mustBeNonnegative}
    powertrain (1,1) struct
end

powertrain = validate_powertrain_config(powertrain);
motorSpeed_rpm = motorSpeed_rpm(:);
Vbus_V = expandToSize(Vbus_V, numel(motorSpeed_rpm));
motor = powertrain.motor;

effectiveVoltage_V = min([Vbus_V, ...
    repmat(powertrain.rules.max_ts_voltage_V, numel(Vbus_V), 1), ...
    repmat(powertrain.inverter.V_dc_max_V, numel(Vbus_V), 1), ...
    repmat(motor.required_voltage_peak_power_V, numel(Vbus_V), 1)], [], 2);
peakPowerAvailable_W = motor.physical_peak_power_W ...
    .* min(effectiveVoltage_V / motor.required_voltage_peak_power_V, 1);
peakLoadSpeed_rpm = motor.Kv_peak_load_rpm_per_V .* effectiveVoltage_V;
stopSpeed_rpm = min(motor.max_mechanical_speed_rpm, ...
    motor.Kv_no_load_rpm_per_V .* effectiveVoltage_V);

omega_radps = motorSpeed_rpm * 2 * pi / 60;
omegaRegularization_radps = 1;
peakTorque_Nm = repmat(motor.peak_torque_Nm, numel(motorSpeed_rpm), 1);
powerTorque_Nm = peakPowerAvailable_W ...
    ./ max(omega_radps, omegaRegularization_radps);
linearFraction = (stopSpeed_rpm - motorSpeed_rpm) ...
    ./ max(stopSpeed_rpm - peakLoadSpeed_rpm, eps);
linearFraction = min(max(linearFraction, 0), 1);
linearTorque_Nm = motor.peak_torque_Nm .* linearFraction;
[availableTorque_Nm, index] = min( ...
    [peakTorque_Nm, powerTorque_Nm, linearTorque_Nm], [], 2);
candidateLabels = ["motor_torque", "motor_power", "motor_voltage"];

voltageScenario = repmat( ...
    "voltage_limited_engineering_inference", numel(Vbus_V), 1);
voltageScenario(effectiveVoltage_V ...
    >= motor.required_voltage_peak_power_V) = ...
    "datasheet_peak_power_voltage";

envelope.effective_voltage_V = effectiveVoltage_V;
envelope.voltage_scenario = voltageScenario;
envelope.peak_power_available_W = peakPowerAvailable_W;
envelope.peak_load_speed_rpm = peakLoadSpeed_rpm;
envelope.stop_speed_rpm = stopSpeed_rpm;
envelope.omega_regularization_radps = omegaRegularization_radps;
envelope.peak_torque_limit_Nm = peakTorque_Nm;
envelope.power_torque_limit_Nm = powerTorque_Nm;
envelope.linear_torque_limit_Nm = linearTorque_Nm;
envelope.available_torque_Nm = availableTorque_Nm;
envelope.available_power_W = availableTorque_Nm .* omega_radps;
envelope.limiter = candidateLabels(index).';
end

function value = expandToSize(value, count)
value = value(:);
if isscalar(value)
    value = repmat(value, count, 1);
elseif numel(value) ~= count
    error("QSSLTS:PowertrainQuerySize", ...
        "Vbus_V must be scalar or match motorSpeed_rpm.");
end
end
