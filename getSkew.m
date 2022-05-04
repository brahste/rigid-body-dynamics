function skew = getSkew(vec)
% getSkew
% Takes a vector and returns it's skew-symmetric matrix
% Params: vec: (3x1 or 6x1 column vector) Target vector.
% Returns: skew: (3x3 or 6x6 matrix) skew symmetric matrix of target
% vector.

if length(vec) == 3
    skew = [0, -vec(3), vec(2); vec(3), 0, -vec(1); -vec(2), vec(1), 0];
elseif length(vec) == 6
    wskew = [0, -vec(6), vec(5); vec(6), 0, -vec(4); -vec(5), vec(4), 0];
    vskew = [0, -vec(3), vec(2); vec(3), 0, -vec(1); -vec(2), vec(1), 0];
    skew = [wskew, vskew; zeros(3), wskew];
end

end

