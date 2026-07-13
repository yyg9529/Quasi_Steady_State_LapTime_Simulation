function projectRoot = project_setup()
%PROJECT_SETUP Add QSSLTS folders to the MATLAB path.
%   projectRoot = PROJECT_SETUP() returns the absolute repository root.

projectRoot = fileparts(mfilename("fullpath"));

folders = [
    fullfile(projectRoot, "src")
    fullfile(projectRoot, "data")
    fullfile(projectRoot, "examples")
    fullfile(projectRoot, "preprocessing")
];

for i = 1:numel(folders)
    if isfolder(folders(i))
        addpath(genpath(folders(i)));
    end
end

fprintf("QSS lap-time project initialized: %s\n", projectRoot);
end
