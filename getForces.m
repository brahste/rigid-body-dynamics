function [FGint, FGext, FGi] = getForces(vGy, C0Bi, C10i, C21i, C32i,...
    C43i, C54i, TGi)
% getForces
% Calculates forces and torques on each link
% Params: vGy: (42x1 column vector) Relative free-joint velocities.
%         C(n)(n-1)i: (3x3 rotation matrix) Matrix relating coordinates in
%          frame (n-1) to frame (n); n=0,1,2,3,4,5 where n=-1=B.
%         TGi: (42x42 matrix) Global transformation matrix.
% Returns: FGint: (42x1 column vector) Global interbody force (control)
%          FGext: (42x1 column vector) Global extrnal force   (gravtity)
%          FGi: (42x1 column vector) Global inertial force    (fictitous)

global cB c0 c1 c2 c3 c4 c5
global M0 M1 M2 M3 M4 M5
global gcB gc0 gc1 gc2 gc3 gc4 gc5
global PG
global gravmod inermod

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Control forces
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Global interbody force (C.40)
FGint = [zeros(5,1); gcB; zeros(5,1); gc0; zeros(5,1); gc1; zeros(5,1); 
    gc2; zeros(5,1); gc3; zeros(5,1); gc4; zeros(5,1); gc5];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Gravitational forces
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

a_g = [0;0;-9.80665]; %Acceleration due to gravity in m/s^2

%Center of mass vectors in base (inertial) frame

%Here I multiply the first moment of mass cn vector (rightmost vectors) 
%by the transpose of the rotation matrix C(n)(m)i such that the orientation
%of the first moments in the inertial frame are represented by 'cN_B'

c0_B = C0Bi.' * c0;
c1_B = (C10i*C0Bi).' * c1;
c2_B = (C21i*C10i*C0Bi).' * c2;
c3_B = (C32i*C21i*C10i*C0Bi).' * c3;
c4_B = (C43i*C32i*C21i*C10i*C0Bi).' * c4;
c5_B = (C54i*C43i*C32i*C21i*C10i*C0Bi).' * c5;

%Take the cross product of 'cmN_B' with the force due to gravity 'l0m * g'
%which is defined on line ~19 to be in the negative z-direction. This
%returns the torque due to gravity about joint n on each respective link.
f0ext = [0; 0; dot(-c0_B,a_g) / (norm(c0_B) * norm(a_g))];
f1ext = [0; 0; dot(-c1_B,a_g) / (norm(c1_B) * norm(a_g))];
f2ext = [0; 0; dot(-c2_B,a_g) / (norm(c2_B) * norm(a_g))];
f3ext = [0; 0; dot(-c3_B,a_g) / (norm(c3_B) * norm(a_g))];
f4ext = [0; 0; dot(-c4_B,a_g) / (norm(c4_B) * norm(a_g))];
f5ext = [0; 0; dot(-c5_B,a_g) / (norm(c5_B) * norm(a_g))];

g0ext = cross(c0_B,a_g);
g1ext = cross(c1_B,a_g);
g2ext = cross(c2_B,a_g);
g3ext = cross(c3_B,a_g);
g4ext = cross(c4_B,a_g);
g5ext = cross(c5_B,a_g);

%Global external force (C.42)
FGext = gravmod * [zeros(6,1); f0ext; g0ext; f1ext; g1ext; f2ext; g2ext; 
    f3ext; g3ext; f4ext; g4ext; f5ext; g5ext];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Fictitous (non-linear) forces
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Calculate global absolute velocities
vG = TGi * PG * vGy;

% Extract components
v0 = vG(7:12);
v1 = vG(13:18);
v2 = vG(19:24);
v3 = vG(25:30);
v4 = vG(31:36);
v5 = vG(37:42);

%Inertial forces about joint n (A.24)
%FiB = getSkew(VB).' * MB * VB;
Fi0 = getSkew(v0).' * M0 * v0;
Fi1 = getSkew(v1).' * M1 * v1;
Fi2 = getSkew(v2).' * M2 * v2;
Fi3 = getSkew(v3).' * M3 * v3;
Fi4 = getSkew(v4).' * M4 * v4;
Fi5 = getSkew(v5).' * M5 * v5;

%Global non-linear force (C.52)
FGi = inermod * [zeros(6,1);Fi0; Fi1; Fi2; Fi3; Fi4; Fi5];

end