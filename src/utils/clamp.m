function y = clamp(x, lower, upper)
%CLAMP Limit x to the inclusive interval [lower, upper].

y = min(max(x, lower), upper);
end
