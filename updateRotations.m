function [C0Bi, C10i, C21i, C32i, C43i, C54i, TGi, dTGi] = updateRotations(vGy, qGy)
% updateRotations
% Evaluates each rotation matrix at the an instantaneous angular position
% and constructs the global interbody transformation matrix and its time
% derivative.
% Params: vGy: (42x1 column vector) Relative free-joint velocities.
%         qGy: (42x1 column vector) Relative free-joint positions.
% Returns: C(n)(n-1)i: (3x3 rotation matrix) Matrix relating coordinates in
%          frame (n-1) to frame (n); n=0,1,2,3,4,5 where n=-1=B.
% Note: The use of 'i' at the end of rotations/transformations indicates 
%       that all of these quantities are 'i'nstantaneous.

global C0B C10 C21 C32 C43 C54
global th0 th1 th2 th3 th4 th5
global rB0 r01 r12 r23 r34 r45
global PG

% Update rotation matrices and convert them to a numerical value
C0Bi = double(subs(C0B, th0, qGy(2)));
C10i = double(subs(C10, th1, qGy(3)));
C21i = double(subs(C21, th2, qGy(4)));
C32i = double(subs(C32, th3, qGy(5)));
C43i = double(subs(C43, th4, qGy(6)));
C54i = double(subs(C54, th5, qGy(7)));

% Retrieve i.b. transformation matrices
T0Bi = getT(C0Bi, rB0);
T10i = getT(C10i, r01);
T21i = getT(C21i, r12);
T32i = getT(C32i, r23);
T43i = getT(C43i, r34);
T54i = getT(C54i, r45);

% Construct the global transformation matrix
TGi = [eye(6),zeros(6,36);
    T0Bi,eye(6),zeros(6,30);
    T10i*T0Bi,T10i,eye(6),zeros(6,24);
    T21i*T10i*T0Bi,T21i*T10i,T21i,eye(6),zeros(6,18);
    T32i*T21i*T10i*T0Bi,T32i*T21i*T10i,T32i*T21i,T32i,eye(6),zeros(6,12);
    T43i*T32i*T21i*T10i*T0Bi,T43i*T32i*T21i*T10i,T43i*T32i*T21i,...
    T43i*T32i, T43i,eye(6),zeros(6,6);
    T54i*T43i*T32i*T21i*T10i*T0Bi,T54i*T43i*T32i*T21i*T10i,...
    T54i*T43i*T32i*T21i,T54i*T43i*T32i,T54i*T43i,T54i,eye(6)];

% Calculate global relative interbody velocities
vGint = PG * vGy;

% Calculate time derivatives of each i.b. transformation matrix
dT0Bi = -getSkew(vGint(7:12)) * T0Bi;
dT10i = -getSkew(vGint(13:18)) * T10i;
dT21i = -getSkew(vGint(19:24)) * T21i;
dT32i = -getSkew(vGint(25:30)) * T32i;
dT43i = -getSkew(vGint(31:36)) * T43i;
dT54i = -getSkew(vGint(37:42)) * T54i;

% Construct the time derivative of the global tranformation matrix
dTGi = [eye(6),zeros(6,36);
    dT0Bi,eye(6),zeros(6,30);
    dT10i*dT0Bi,dT10i,eye(6),zeros(6,24);
    dT21i*dT10i*dT0Bi,dT21i*dT10i,dT21i,eye(6),zeros(6,18);
    dT32i*dT21i*dT10i*dT0Bi,dT32i*dT21i*dT10i,dT32i*dT21i,dT32i,eye(6),...
    zeros(6,12);
    dT43i*dT32i*dT21i*dT10i*dT0Bi,dT43i*dT32i*dT21i*dT10i,...
    dT43i*dT32i*dT21i,dT43i*dT32i,dT43i,eye(6),zeros(6,6);
    dT54i*dT43i*dT32i*dT21i*dT10i*dT0Bi,dT54i*dT43i*dT32i*dT21i*dT10i,...
    dT54i*dT43i*dT32i*dT21i,dT54i*dT43i*dT32i,dT54i*dT43i,dT54i,eye(6)];

end

