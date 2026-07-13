function comparison = compare_limiter_usage(fastestResult, slowestResult)
%COMPARE_LIMITER_USAGE Compare exact distance-weighted limiter categories.

    arguments
        fastestResult (1, 1) struct
        slowestResult (1, 1) struct
    end

    fastest = summarize_limiter_usage(fastestResult);
    slowest = summarize_limiter_usage(slowestResult);
    limiter = union(fastest.limiter, slowest.limiter, "stable");
    fastest_percent_distance = lookupPercent(limiter, fastest);
    slowest_percent_distance = lookupPercent(limiter, slowest);
    delta_slowest_minus_fastest_percent = ...
        slowest_percent_distance - fastest_percent_distance;
    comparison = table(limiter, fastest_percent_distance, ...
        slowest_percent_distance, delta_slowest_minus_fastest_percent);
    comparison = sortrows(comparison, ...
        "fastest_percent_distance", "descend");
end

function percentage = lookupPercent(limiterNames, usage)
    percentage = zeros(numel(limiterNames), 1);
    for index = 1:numel(limiterNames)
        match = usage.limiter == limiterNames(index);
        if any(match)
            percentage(index) = usage.percent_distance(match);
        end
    end
end
