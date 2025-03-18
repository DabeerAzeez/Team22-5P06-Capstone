% Helper ternary function
function result = ternary(cond, valTrue, valFalse)
if cond
    result = valTrue;
else
    result = valFalse;
end
end