function T = getT(C, r)
% getT
% Constructs an interbody transformation matrix.
% Params: C: (3x3 matrix) Target rotation matrix.
%         r: (3x1 column vector) Target interbody position vector.
% Returns: T: (6x6 matrix) Interbody transformation matrix.
T = [C, -C * getSkew(r); zeros(3), C];
end

