function [muX, muY] = tire_constant_mu(Fz_N, tire)
%TIRE_CONSTANT_MU Return constant longitudinal/lateral friction values.
%   Fz_N is accepted to preserve a common tire-model interface.

arguments
    Fz_N double
    tire (1,1) struct
end

muX = tire.mu_x + zeros(size(Fz_N));
muY = tire.mu_y + zeros(size(Fz_N));
end
