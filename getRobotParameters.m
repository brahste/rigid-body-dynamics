function getRobotParameters()

%global cmB cm0 cm1 cm2 cm3 cm4 cm5
global cB c0 c1 c2 c3 c4 c5

global MB M0 M1 M2 M3 M4 M5
global mB m0 m1 m2 m3 m4 m5
global MG PG
global C0B C10 C21 C32 C43 C54
global th0 th1 th2 th3 th4 th5
global rB0 r01 r12 r23 r34 r45 r56
global Bpf

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Rotation related quantities
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

syms th0 th1 th2 th3 th4 th5;

%Rotation matricies for each joint -- Cnm rotates coords from
%frame m to frame n.

%Rotation from Base frame to frame 0
C0B = [cos(th0), sin(th0), 0; -sin(th0), cos(th0), 0; 0, 0, 1]*...
    [1,0,0;0,-1,0;0,0,-1];
%Rotation from frame 0 to frame 1
C10 = [cos(th1), sin(th1), 0; -sin(th1), cos(th1), 0; 0, 0, 1]*...
    [0,0,-1;0,1,0;1,0,0]*[1,0,0;0,0,1;0,-1,0];
%Rotation from frame 1 to frame 2
C21 = [cos(th2), sin(th2), 0; -sin(th2), cos(th2), 0; 0, 0, 1]*...
    [-1,0,0;0,-1,0;0,0,1]*[1,0,0;0,0,1;0,-1,0];
%------''----------- 2 -> 3
C32 = [cos(th3), sin(th3), 0; -sin(th3), cos(th3), 0; 0, 0, 1]*...
    [1,0,0;0,0,-1;0,1,0];
%------''----------- 3 -> 4
C43 = [cos(th4), sin(th4), 0; -sin(th4), cos(th4), 0; 0, 0, 1]*...
    [-1,0,0;0,-1,0;0,0,1]*[1,0,0;0,0,-1;0,1,0];
%------''----------- 4 -> 5
C54 = [cos(th5), sin(th5), 0; -sin(th5), cos(th5), 0; 0, 0, 1]*...
    [1,0,0;0,0,-1;0,1,0]*[0,0,-1;0,1,0;1,0,0];

% Define initial rotation matrices
C0Bin = double(subs(C0B,th0,0));
C10in = double(subs(C10,th1,0));
C21in = double(subs(C21,th2,0));
C32in = double(subs(C32,th3,0));
C43in = double(subs(C43,th4,0));
C54in = double(subs(C54,th5,0));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Link related quantities
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Link lengths (m)
% Bottom of manipulator to zeroth joint (AP 4075 0003). The zeroth joint
% corresponds to the origin of both frame-B and frame-0.
lbase = 0.1565;
% Zeroth joint to first joint -- link 0 (AM 7575 0005)
l0 = 0.11975;
% First joint to second joint -- link 1 (KR 822:1)
l1 = 0.2050;
% Second joint to third joint -- link 2 (KR 822:2)
l2 = 0.2050;
% Third joint to fourth joint -- link 3 (AM 7558 0003)
l3 = 0.2073;
% Third joint to fourth joint shoulder offset
l3offset = 0.0098;
% Fourth joint to fifth joint -- link 4 (KR 823:1)
l4 = 0.1038;
% Fifth joint to sixth joint -- link 5 (KR 823:2)
l5 = 0.1038;

%Link connection vectors -- rMN is the vector from joint m to joint n
%Inertial frame and 0th frame have the same origin
rB0 = [0; 0; 0];
r01 = l0 * [0; 0; -1];
r12 = l1 * [0; -1; 0];
r23 = l2 * [0; 0; 1];
r34 = [0; -l3; -l3offset];
r45 = l4 * [0; 0; -1];
r56 = l5 * [1; 0; 0];

%Center of mass vectors from frame origin (m): Here the CM from the base of
%the arm to the nth link (leftmost vector) is subtracted by the vector 
%from the base to the nth joint (rightmost vector).
cmB = [-0.302609; 1.12552; lbase-70.6408]*1e-3;
cm0 = C0Bin * ([10.8787; 2.0965e-3; 226.174]*1e-3 - [0; 0; lbase]);
cm1 = (C10in*C0Bin) *...
    ([-14.4691; 1.27792e-3; 369.622]*1e-3 - [0; 0; lbase+l0]);
cm2 = (C21in*C10in*C0Bin) *...
    ([-14.4691; -1.27792e-3; 591.377]*1e-3 - [0; 0; lbase+l0+l1]);
cm3 = (C32in*C21in*C10in*C0Bin) *...
    ([16.848; -4.42603e-3; 760.246]* 1e-3 - [0; 0; lbase+l0+l1+l2]);
cm4 = (C43in*C32in*C21in*C10in*C0Bin) *...
    ([-4.54468; 15.2575e-3; 956.346]*1e-3 -...
    [l3offset; 0; lbase+l0+l1+l2+l3]);
cm5 = (C54in*C43in*C32in*C21in*C10in*C0Bin) *...
    ([24.0447; 15.2575e-3; 1036.66]*1e-3 -...
    [l3offset; 0; lbase+l0+l1+l2+l3+l4]);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Mass related quantities
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Link masses (kg)
mB = Bpf * (2292.668 * 1e-3);
m0 = 583.341 * 1e-3;
m1 = 912.766 * 1e-3;
m2 = 912.766 * 1e-3;
m3 = 593.781 * 1e-3;
m4 = 367.799 * 1e-3;
m5 = 367.799 * 1e-3;

%Moments of inertia (kg m^2) about frame origin
JB = [1.671E+07, 2.492E+04, 7.640E+04; 2.492E+04, 1.657E+07, 4.182E+05;
    7.640E+04, -4.182E+05, 2.597E+06] * 1e-9;
J0 = [3.176E+7, 22.253, -1.669E+6; 22.253, 3.171E+7, -269.712;
    -1.669E+6, -269.712, 7.815E+5] * 1e-9;
J1 = [1.307E+08, -11.438, 4.224E+06; -11.438, 1.308E+08, -463.315;
    4.224E+06, -463.315, 1.296E+06] * 1e-9;
J2 = [3.253E+08, 10.64, 8.474E+06; 10.64, 3.254E+08, 658.102;
    8.474E+06, 658.102, 1.296E+06] * 1e-9;
J3 = [3.472E+08, 7.454, -7.459E+06; 7.454, 3.472E+08, 2106.952;
    -7.459E+06, 2106.952, 6.336E+05] * 1e-9;
J4 = [3.371E+08, 17.258, 1.750E+06; 17.258, 3.371E+08, -5302.782;
    1.750E+06, -5302.782, 3.111E+05] * 1e-9;
J5 = [3.960E+08, -126.468, -9.017E+06; -126.468,3.962E+08, -5880.65;
    -9.017E+06, -5880.65, 5.162E+05] * 1e-9;

%First moments of mass from frame origin: Product of CM and total mass
cB = cmB * mB;
c0 = cm0 * m0;
c1 = cm1 * m1;
c2 = cm2 * m2;
c3 = cm3 * m3;
c4 = cm4 * m4;
c5 = cm5 * m5;

%Mass matricies (see A.15)
MB = [eye(3)*mB, -getSkew(cB)*0.99; getSkew(cB)*0.99, JB];
M0 = [eye(3)*m0, -getSkew(c0)*0.99; getSkew(c0)*0.99, J0];
M1 = [eye(3)*m1, -getSkew(c1)*0.99; getSkew(c1)*0.99, J1];
M2 = [eye(3)*m2, -getSkew(c2)*0.99; getSkew(c2)*0.99, J2];
M3 = [eye(3)*m3, -getSkew(c3)*0.99; getSkew(c3)*0.99, J3];
M4 = [eye(3)*m4, -getSkew(c4)*0.99; getSkew(c4)*0.99, J4];
M5 = [eye(3)*m5, -getSkew(c5)*0.99; getSkew(c5)*0.99, J5];

%Global mass matrix
MG = blkdiag(MB,M0,M1,M2,M3,M4,M5);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Projection related quantities
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Projection matrix: All axes aligned such that a positive rotation is c.c.w
%about the z-axis
P = [0; 0; 0; 0; 0; 1];
PG = blkdiag(ones(6,1),P,P,P,P,P,P);

save robotParameters