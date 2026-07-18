function app = launch_qsslts_app(visible)
%LAUNCH_QSSLTS_APP Launch the QSSLTS engineering GUI.

arguments
    visible (1,1) logical = true
end

projectRoot = fileparts(mfilename("fullpath"));
addpath(fullfile(projectRoot, "app"));
project_setup();

app = QssltsApp(visible);
if nargout == 0
    clear app
end
end
