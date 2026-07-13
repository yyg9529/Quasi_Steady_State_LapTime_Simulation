function provenance = tire_envelope_provenance(tire)
%TIRE_ENVELOPE_PROVENANCE Return serializable GGV-relevant tire inputs.
%   slip_force is a separate 7DOF constitutive model and may contain a
%   function handle. It neither generates nor identifies the QSS envelope.

    arguments
        tire (1, 1) struct
    end

    provenance = tire;
    if isfield(provenance, "slip_force")
        provenance = rmfield(provenance, "slip_force");
    end
end
