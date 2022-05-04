% Author: Braden Stefanuk
% Date: Mar 27, 2019
% This is the main script for the simulation of Kinova's 7DOF Spherical
% arm following the 'global formulation' developed by Sincarsin & Hughes 
% (1989).

% 1) Clear everything
clear variables
clear global
close all

% 2) Get all of the variables related to the composition and geometry
% of the manipulator.

% Set a base body prefactor: if large -> motion of base is negligible
%                            if unity -> motion of base is unaffected                            
global Bpf
Bpf = 417289; %(kg) Mass of ISS

getRobotParameters()
load('robotParameters')

% 3) Set the initial conditions of the simulation.

% Global relative free-joint positions, 42x1 column vector
%qGy_in = zeros(42,1);
qGy_in = zeros(7,1);

% Global relative free-joint velocities, 42x1 column vector
%vGy_in = zeros(42,1);
vGy_in = zeros(7,1);

% Construct 84x1 input state
instate = [vGy_in; qGy_in];

% Set control torques
global gcB gc0 gc1 gc2 gc3 gc4 gc5 gravmod inermod

gcB = -15;
gc0 = -15;
gc1 = -15;
gc2 = -15;
gc3 = -15;
gc4 = -3.4;
gc5 = -3.4;

% Gravity modifier
gravmod = 0;
inermod = 1;

% Set a mass matrix additive factor---temporary solution to bypass singular
% mass matrix issue (as of Mar. 28, 2019)
global Maf
Maf = 0.5e1 * eye(7);

% 6) Run simulating function through an integrator.

% Define time steps
dt = 0.05;
tmax = 10.00;
tsteps = 0:dt:tmax;

% Set options to pass to integrator
options = odeset('RelTol',1e-2);

% Integrate & run simulation
tic
time = 0;
for i=1:length(tsteps)-1
    time = time + dt;
    tspan = [0: dt/2: dt];
    [tout, output] = ode45(@getInstantaneousAccel, tspan, instate, options);
    %instate = output(3,1:84)';  
    % Set output to third row because each loop computes from 0,dt/2,dt
    instate = output(3,1:14)';
    
    result(i,:) = instate;
    t(i) = time;
    if time > (tmax/2)
    %    gravmod = 0;
        gcB = 0;
        gc0 = 0;
        gc1 = 0;
        gc2 = 0;
        gc3 = 0;
        gc4 = 0;
        gc5 = 0;
    end
end
toc

% 7) Validate the simulation

getPlots(t, result, [mB, m0, m1, m2, m3, m4, m5],...
    [JB, J0, J1, J2, J3, J4, J5])
