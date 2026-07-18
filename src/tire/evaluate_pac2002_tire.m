function output = evaluate_pac2002_tire(model, input)
%EVALUATE_PAC2002_TIRE Evaluate steady-state PAC2002 forces and moments.
%   Inputs use SI units and a common vehicle-oriented tyre frame. Scalar
%   fields are expanded to the shape of Fz_N. The TIR model is reflected
%   when its fitted side and the requested mount side differ.

arguments
    model (1, 1) struct
    input (1, 1) struct
end

validateModel(model);
state = normalizeInput(input);
sections = model.sections;

modelOrientation = sideOrientation(model.model_side);
mountOrientation = sideOrientation(state.mount_side);
reflection = mountOrientation .* modelOrientation;
% PAC2002 uses the opposite slip-angle sign from the handling interface,
% where positive alpha must produce positive lateral force.
alpha = -reflection .* state.alpha_rad;
gamma = reflection .* state.gamma_rad;
turnSlip = reflection .* state.turn_slip_1pm;

Fz = state.Fz_N;
Fz0 = model.nominal_load_N;
FzEvaluation = max(Fz, eps(Fz0));
dfz = (FzEvaluation - Fz0) ./ Fz0;
kappa = state.kappa;
Vx = state.Vx_mps;

[Fx0, Kx] = pureLongitudinal(sections, FzEvaluation, ...
    dfz, kappa, gamma);
[Fy0, Kya, By, Cy, muy, SHy, SVy] = pureLateral( ...
    sections, FzEvaluation, Fz0, dfz, alpha, gamma);
FxModel = combinedLongitudinal(sections, Fx0, dfz, kappa, alpha);
FyModel = combinedLateral(sections, Fy0, FzEvaluation, dfz, ...
    kappa, alpha, gamma, muy);

effectiveRadius = effectiveRollingRadius( ...
    sections, model.unloaded_radius_m, FzEvaluation, Fz0);
MxModel = overturningMoment(sections, model.unloaded_radius_m, ...
    FzEvaluation, Fz0, gamma, FyModel);
MyModel = rollingMoment(sections, model.unloaded_radius_m, ...
    FzEvaluation, Fz0, FxModel, Vx, model.reference_speed_mps);
MzModel = aligningMoment(sections, model.unloaded_radius_m, ...
    FzEvaluation, Fz0, dfz, alpha, gamma, kappa, ...
    FxModel, FyModel, Kx, Kya, By, Cy, SHy, SVy, turnSlip);

speedScale = min(1, abs(Vx) ./ max(model.low_speed_threshold_mps, eps));
contact = Fz > 0;
FxModel = FxModel .* speedScale .* contact;
FyModel = FyModel .* speedScale .* contact;
MxModel = MxModel .* speedScale .* contact;
MyModel = MyModel .* speedScale .* contact;
MzModel = MzModel .* speedScale .* contact;
effectiveRadius(~contact) = model.unloaded_radius_m;

output.Fx_N = FxModel;
output.Fy_N = reflection .* FyModel;
output.Mx_Nm = reflection .* MxModel;
output.My_Nm = MyModel;
output.Mz_Nm = reflection .* MzModel;
output.effective_radius_m = effectiveRadius;
output.within_range = contact & ...
    inRange(Fz, model.ranges.Fz_N) & ...
    inRange(kappa, model.ranges.kappa) & ...
    inRange(state.alpha_rad, model.ranges.alpha_rad) & ...
    inRange(state.gamma_rad, model.ranges.gamma_rad);
end

function validateModel(model)
required = ["sections", "model_side", "nominal_load_N", ...
    "unloaded_radius_m", "reference_speed_mps", ...
    "low_speed_threshold_mps", "ranges"];
for index = 1:numel(required)
    if ~isfield(model, required(index))
        error("evaluate_pac2002_tire:InvalidModel", ...
            "PAC2002 model is missing field %s.", required(index));
    end
end
end

function state = normalizeInput(input)
required = ["Fz_N", "kappa", "alpha_rad", "gamma_rad", ...
    "turn_slip_1pm", "Vx_mps", "mount_side"];
for index = 1:numel(required)
    if ~isfield(input, required(index))
        error("evaluate_pac2002_tire:MissingInput", ...
            "Input is missing field %s.", required(index));
    end
end

targetSize = size(input.Fz_N);
state.Fz_N = expandNumeric(input.Fz_N, targetSize, "Fz_N");
state.kappa = expandNumeric(input.kappa, targetSize, "kappa");
state.alpha_rad = expandNumeric( ...
    input.alpha_rad, targetSize, "alpha_rad");
state.gamma_rad = expandNumeric( ...
    input.gamma_rad, targetSize, "gamma_rad");
state.turn_slip_1pm = expandNumeric( ...
    input.turn_slip_1pm, targetSize, "turn_slip_1pm");
state.Vx_mps = expandNumeric(input.Vx_mps, targetSize, "Vx_mps");
state.mount_side = expandSide(input.mount_side, targetSize);
end

function value = expandNumeric(value, targetSize, name)
if ~isnumeric(value) || ~isreal(value) || any(~isfinite(value), "all")
    error("evaluate_pac2002_tire:InvalidInput", ...
        "Input field %s must contain finite real numeric values.", name);
end
if isscalar(value)
    value = repmat(double(value), targetSize);
elseif ~isequal(size(value), targetSize)
    error("evaluate_pac2002_tire:InputShape", ...
        "Input field %s must be scalar or match the size of Fz_N.", name);
else
    value = double(value);
end
end

function side = expandSide(value, targetSize)
side = upper(string(value));
if isscalar(side)
    side = repmat(side, targetSize);
elseif ~isequal(size(side), targetSize)
    error("evaluate_pac2002_tire:InputShape", ...
        "Input field mount_side must be scalar or match Fz_N.");
end
if any(~ismember(side, ["LEFT", "RIGHT"]), "all")
    error("evaluate_pac2002_tire:InvalidMountSide", ...
        "mount_side values must be LEFT or RIGHT.");
end
end

function orientation = sideOrientation(side)
orientation = ones(size(side));
orientation(upper(string(side)) == "RIGHT") = -1;
end

function [Fx0, Kx] = pureLongitudinal(s, Fz, dfz, kappa, gamma)
p = s.LONGITUDINAL_COEFFICIENTS;
l = s.SCALING_COEFFICIENTS;
Cx = value(p, "PCX1", 1) * value(l, "LCX", 1);
mux = (value(p, "PDX1", 0) + value(p, "PDX2", 0) .* dfz) .* ...
    (1 - value(p, "PDX3", 0) .* gamma.^2) * value(l, "LMUX", 1);
Dx = mux .* Fz;
Kx = Fz .* (value(p, "PKX1", 0) + value(p, "PKX2", 0) .* dfz) .* ...
    exp(value(p, "PKX3", 0) .* dfz) * value(l, "LKX", 1);
Bx = Kx ./ nonzero(Cx .* Dx);
SHx = (value(p, "PHX1", 0) + value(p, "PHX2", 0) .* dfz) * ...
    value(l, "LHX", 1);
SVx = Fz .* (value(p, "PVX1", 0) + value(p, "PVX2", 0) .* dfz) * ...
    value(l, "LVX", 1) * value(l, "LMUX", 1);
kappaShifted = kappa + SHx;
Ex = (value(p, "PEX1", 0) + value(p, "PEX2", 0) .* dfz + ...
    value(p, "PEX3", 0) .* dfz.^2) .* ...
    (1 - value(p, "PEX4", 0) .* sign(kappaShifted)) * ...
    value(l, "LEX", 1);
Ex = min(Ex, 1);
Fx0 = magicFormula(Bx, Cx, Dx, Ex, kappaShifted) + SVx;
end

function [Fy0, Kya, By, Cy, muy, SHy, SVy] = pureLateral( ...
        s, Fz, Fz0, dfz, alpha, gamma)
p = s.LATERAL_COEFFICIENTS;
l = s.SCALING_COEFFICIENTS;
Cy = value(p, "PCY1", 1) * value(l, "LCY", 1);
muy = (value(p, "PDY1", 0) + value(p, "PDY2", 0) .* dfz) .* ...
    (1 - value(p, "PDY3", 0) .* gamma.^2) * value(l, "LMUY", 1);
Dy = muy .* Fz;
Kya = value(p, "PKY1", 0) * Fz0 .* ...
    sin(2 .* atan(Fz ./ nonzero(value(p, "PKY2", 1) * Fz0))) .* ...
    (1 - value(p, "PKY3", 0) .* abs(gamma)) * value(l, "LKY", 1);
By = Kya ./ nonzero(Cy .* Dy);
SHy = (value(p, "PHY1", 0) + value(p, "PHY2", 0) .* dfz) * ...
    value(l, "LHY", 1) + value(p, "PHY3", 0) .* gamma * ...
    value(l, "LKYG", 1);
SVy = Fz .* (value(p, "PVY1", 0) + value(p, "PVY2", 0) .* dfz) * ...
    value(l, "LVY", 1) * value(l, "LMUY", 1) + ...
    Fz .* (value(p, "PVY3", 0) + value(p, "PVY4", 0) .* dfz) .* ...
    gamma * value(l, "LKYG", 1) * value(l, "LMUY", 1);
alphaShifted = alpha + SHy;
Ey = (value(p, "PEY1", 0) + value(p, "PEY2", 0) .* dfz) .* ...
    (1 - (value(p, "PEY3", 0) + value(p, "PEY4", 0) .* gamma) .* ...
    sign(alphaShifted)) * value(l, "LEY", 1);
Ey = min(Ey, 1);
Fy0 = magicFormula(By, Cy, Dy, Ey, alphaShifted) + SVy;
end

function Fx = combinedLongitudinal(s, Fx0, dfz, kappa, alpha)
p = s.LONGITUDINAL_COEFFICIENTS;
l = s.SCALING_COEFFICIENTS;
B = value(p, "RBX1", 0) .* cos(atan(value(p, "RBX2", 0) .* kappa)) * ...
    value(l, "LXAL", 1);
C = value(p, "RCX1", 1);
E = min(value(p, "REX1", 0) + value(p, "REX2", 0) .* dfz, 1);
shift = value(p, "RHX1", 0);
numerator = magicCosine(B, C, E, alpha + shift);
denominator = magicCosine(B, C, E, shift);
Fx = Fx0 .* numerator ./ nonzero(denominator);
end

function Fy = combinedLateral(s, Fy0, Fz, dfz, kappa, alpha, gamma, muy)
p = s.LATERAL_COEFFICIENTS;
l = s.SCALING_COEFFICIENTS;
B = value(p, "RBY1", 0) .* ...
    cos(atan(value(p, "RBY2", 0) .* ...
    (alpha - value(p, "RBY3", 0)))) * value(l, "LYKA", 1);
C = value(p, "RCY1", 1);
E = min(value(p, "REY1", 0) + value(p, "REY2", 0) .* dfz, 1);
shift = value(p, "RHY1", 0) + value(p, "RHY2", 0) .* dfz;
gain = magicCosine(B, C, E, kappa + shift) ./ ...
    nonzero(magicCosine(B, C, E, shift));
peakShift = muy .* Fz .* (value(p, "RVY1", 0) + ...
    value(p, "RVY2", 0) .* dfz + value(p, "RVY3", 0) .* gamma) .* ...
    cos(atan(value(p, "RVY4", 0) .* alpha)) * value(l, "LVYKA", 1);
verticalShift = peakShift .* sin(value(p, "RVY5", 0) .* ...
    atan(value(p, "RVY6", 0) .* kappa));
Fy = gain .* Fy0 + verticalShift;
end

function Mx = overturningMoment(s, R0, Fz, Fz0, gamma, Fy)
p = s.OVERTURNING_COEFFICIENTS;
l = s.SCALING_COEFFICIENTS;
normalizedFy = Fy ./ Fz0;
normalizedFz = Fz ./ Fz0;
bracket = value(p, "QSX1", 0) * value(l, "LVMX", 1) - ...
    value(p, "QSX2", 0) .* gamma + ...
    value(p, "QSX3", 0) .* normalizedFy + ...
    value(p, "QSX4", 0) .* cos(value(p, "QSX5", 0) .* ...
    atan((value(p, "QSX6", 0) .* normalizedFz).^2)) .* ...
    sin(value(p, "QSX7", 0) .* gamma + value(p, "QSX8", 0) .* ...
    atan(value(p, "QSX9", 0) .* normalizedFy)) + ...
    value(p, "QSX10", 0) .* atan(value(p, "QSX11", 0) .* ...
    normalizedFz) .* gamma;
Mx = R0 .* Fz .* bracket * value(l, "LMX", 1);
end

function My = rollingMoment(s, R0, Fz, Fz0, Fx, Vx, referenceSpeed)
p = s.ROLLING_COEFFICIENTS;
l = s.SCALING_COEFFICIENTS;
speedRatio = abs(Vx) ./ max(referenceSpeed, eps);
coefficient = value(p, "QSY1", 0) + ...
    value(p, "QSY2", 0) .* Fx ./ Fz0 + ...
    value(p, "QSY3", 0) .* speedRatio + ...
    value(p, "QSY4", 0) .* speedRatio.^4;
My = -R0 .* Fz .* coefficient * value(l, "LMY", 1) .* sign(Vx);
end

function Mz = aligningMoment(s, R0, Fz, Fz0, dfz, alpha, gamma, ...
        kappa, Fx, Fy, Kx, Kya, By, Cy, SHy, SVy, turnSlip)
p = s.ALIGNING_COEFFICIENTS;
l = s.SCALING_COEFFICIENTS;
Sht = value(p, "QHZ1", 0) + value(p, "QHZ2", 0) .* dfz + ...
    (value(p, "QHZ3", 0) + value(p, "QHZ4", 0) .* dfz) .* gamma;
alphaTrail = alpha + Sht;
SHf = SHy + SVy ./ nonzero(Kya);
alphaResidual = alpha + SHf;
stiffnessRatio = Kx ./ nonzero(Kya);
alphaTrailEquivalent = equivalentAngle(alphaTrail, stiffnessRatio, kappa);
alphaResidualEquivalent = equivalentAngle( ...
    alphaResidual, stiffnessRatio, kappa);

Bt = (value(p, "QBZ1", 0) + value(p, "QBZ2", 0) .* dfz + ...
    value(p, "QBZ3", 0) .* dfz.^2) .* ...
    (1 + value(p, "QBZ4", 0) .* gamma + ...
    value(p, "QBZ5", 0) .* abs(gamma)) * ...
    value(l, "LKY", 1) / max(value(l, "LMUY", 1), eps);
Ct = value(p, "QCZ1", 1);
Dt = Fz .* (value(p, "QDZ1", 0) + value(p, "QDZ2", 0) .* dfz) .* ...
    (1 + value(p, "QDZ3", 0) .* gamma + ...
    value(p, "QDZ4", 0) .* gamma.^2) .* (R0 ./ Fz0) * ...
    value(l, "LTR", 1);
Et = (value(p, "QEZ1", 0) + value(p, "QEZ2", 0) .* dfz + ...
    value(p, "QEZ3", 0) .* dfz.^2) .* ...
    (1 + (value(p, "QEZ4", 0) + value(p, "QEZ5", 0) .* gamma) .* ...
    (2 / pi) .* atan(Bt .* Ct .* alphaTrailEquivalent));
trail = Dt .* cos(Ct .* atan(Bt .* alphaTrailEquivalent - ...
    Et .* (Bt .* alphaTrailEquivalent - ...
    atan(Bt .* alphaTrailEquivalent)))) .* cos(alpha);

Br = value(p, "QBZ9", 0) * value(l, "LKY", 1) / ...
    max(value(l, "LMUY", 1), eps) + value(p, "QBZ10", 0) .* By .* Cy;
Dr = Fz .* R0 .* ((value(p, "QDZ6", 0) + ...
    value(p, "QDZ7", 0) .* dfz) * value(l, "LRES", 1) + ...
    (value(p, "QDZ8", 0) + value(p, "QDZ9", 0) .* dfz) .* ...
    gamma * value(l, "LGAZ", 1)) * value(l, "LMUY", 1);
residual = Dr .* cos(atan(Br .* alphaResidualEquivalent)) .* cos(alpha);
lever = (value(p, "SSZ1", 0) + value(p, "SSZ2", 0) .* Fy ./ Fz0 + ...
    (value(p, "SSZ3", 0) + value(p, "SSZ4", 0) .* dfz) .* gamma) .* ...
    R0 * value(l, "LS", 1);
% This PAC2002 parameter set has no spin-slip force coefficients. QTZ1 is
% a gyroscopic-wheel term and is therefore not inferred from tyre data.
gyration = zeros(size(turnSlip));
Mz = -trail .* Fy + residual + lever .* Fx + gyration;
end

function angleEquivalent = equivalentAngle(angle, stiffnessRatio, kappa)
angleEquivalent = atan(sqrt(tan(angle).^2 + ...
    (stiffnessRatio .* kappa).^2)) .* sign(angle);
end

function radius = effectiveRollingRadius(s, R0, Fz, Fz0)
p = s.VERTICAL;
verticalStiffness = value(p, "VERTICAL_STIFFNESS", NaN);
if ~isfinite(verticalStiffness) || verticalStiffness <= 0
    error("evaluate_pac2002_tire:InvalidVerticalStiffness", ...
        "VERTICAL.VERTICAL_STIFFNESS must be positive and finite.");
end
nominalDeflection = Fz0 ./ verticalStiffness;
normalizedLoad = Fz ./ Fz0;
radius = R0 - nominalDeflection .* (value(p, "DREFF", 0) .* ...
    atan(value(p, "BREFF", 0) .* normalizedLoad) + ...
    value(p, "FREFF", 0) .* normalizedLoad);
end

function result = magicFormula(B, C, D, E, slip)
argument = B .* slip;
result = D .* sin(C .* atan(argument - E .* ...
    (argument - atan(argument))));
end

function result = magicCosine(B, C, E, slip)
argument = B .* slip;
result = cos(C .* atan(argument - E .* ...
    (argument - atan(argument))));
end

function denominator = nonzero(denominator)
small = abs(denominator) < eps;
denominator(small) = eps .* (1 - 2 .* (denominator(small) < 0));
end

function result = inRange(valueToCheck, limits)
result = valueToCheck >= limits(1) & valueToCheck <= limits(2);
end

function result = value(section, name, default)
if isfield(section, name)
    result = section.(name);
else
    result = default;
end
end
