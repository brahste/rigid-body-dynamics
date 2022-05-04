function statederiv = getInstantaneousAccel(t, instate)
% getInstananeousAccel
% Calculates the derivative of the input state for the simulation contained
% in the script file 'getAccelMain.m'.
% Params: t: (float) time step passed to integrator.
%         instate: (84x1 column vector) Relative free-joint
%         velocities (1:42) and positions (43:84).
% Returns: statederiv: (84x1 column vector) Relative free-joint
%          accelerations (1:42) and velocities (43:84).

global MG PG Maf

% Extract relative free-joint positions from input state
qGy = instate(8:14);

% Extract relative free-joint velocities from input state
vGy = instate(1:7);

% Update all rotation matricies using the input state and construct
% variables for the global interbody transfomation matrix and its time
% derivative
[C0Bi,C10i,C21i,C32i,C43i,C54i,TGi,dTGi] = updateRotations(vGy,qGy);

% Evaluate the global forces using the input state
[FGint,FGext,FGi] = getForces(vGy,C0Bi,C10i,C21i,C32i,C43i,C54i,TGi);

% Apply connections and projections to global mass matrix (B.94)
Msystem = Maf + (PG.' * TGi.' * MG * TGi * PG);

% Apply connections and projections to global interbody force (B.62 & B.96)
Fcontrol = -PG.' * FGint;

% Apply connections and projections to global external force (B.63 & B.97)
Fexternal = PG.' * TGi.' * FGext;

% Apply connections and projections to global inertial force (B.64 & B.98)
Finertial = PG.' * TGi * FGi;

% Evaluate additional non-linear term
Fnonlinear = -PG.' * TGi.' * MG * dTGi * PG * vGy;

% Compute the acceleration
aGy = Msystem \ (Fcontrol + Fexternal + Finertial + Fnonlinear);
aGy = double(aGy);

% Return the derivative of the input state
statederiv = [aGy; vGy];
end