function info = resolve_mfeval_installation()
%RESOLVE_MFEVAL_INSTALLATION Select the newest installed MFeval release.

candidates = string(which("mfeval", "-all"));
if isempty(candidates)
    error("QSSLTS:MFevalUnavailable", ...
        "MFeval is not installed or is not available on the MATLAB path.");
end

versions = strings(size(candidates));
scores = -inf(size(candidates));
for index = 1:numel(candidates)
    contentsFile = fullfile(fileparts(candidates(index)), "Contents.m");
    if ~isfile(contentsFile)
        continue
    end
    token = regexp(fileread(contentsFile), ...
        "Version\s+([0-9]+(?:\.[0-9]+)+)", "tokens", "once");
    if isempty(token)
        continue
    end
    versions(index) = string(token{1});
    parts = sscanf(versions(index), "%d.%d.%d").';
    parts(end + 1:3) = 0;
    scores(index) = parts(1) * 1e6 + parts(2) * 1e3 + parts(3);
end

[bestScore, bestIndex] = max(scores);
if ~isfinite(bestScore)
    error("QSSLTS:MFevalVersionUnknown", ...
        "Could not determine the version of the installed MFeval copies.");
end

info.version = versions(bestIndex);
info.function_file = candidates(bestIndex);
info.root = string(fileparts(candidates(bestIndex)));
addpath(info.root, "-begin");
clear("mfeval");
rehash path;
if ~strcmpi(string(which("mfeval")), info.function_file)
    error("QSSLTS:MFevalActivationFailed", ...
        "Could not activate MFeval %s at %s.", ...
        info.version, info.function_file);
end
end
