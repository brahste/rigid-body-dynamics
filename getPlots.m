function getPlots(t, result, ms, Js)
% Plot the kinetic energy

% Extract global relative free-joint velocities from the simulation result
vGy_t = result(:,1:7);
qGy_t = result(:,8:14);
[nrows, ncols] = size(vGy_t);

Krot = zeros(length(t), 7);
% Compute global relative interbody velocities at each time step
for k = 1:1:length(t)
    for n = 1:1:ncols
        Krot(k,n) = 0.5 * Js(n) * (vGy_t(k,n))^2;
    end
end

fig_w = figure('Name','Rotational Free-joint Velocities');
plot(t, vGy_t)
legend('B','0','1','2','3','4','5','Location','northwest');
grid on
ylabel('wGy (t) (rad/s)');
xlabel('t (s)');
%savefig('RotationalVel')
%matlab2tikz('RotationalVelTikzNog.tex')


fig_th = figure('Name','Rotational Free-joint Positions');
plot(t, qGy_t)
legend('B','0','1','2','3','4','5','Location','northwest')
grid on
ylabel('thGy (t) (rad)');
xlabel('t (s)');
%savefig('RotationalPos')
%matlab2tikz('RotationalPosTikzNog.tex')

fig_Krot = figure('Name','Rotational Kinetic Energy');
plot(t, Krot)
legend('B','0','1','2','3','4','5','Location','northwest')
grid on
ylabel('Krot (t) g (mm rad/s)^2');
xlabel('t (s)');
%savefig('RotationalK')
%matlab2tikz('RotationalKTikzNog.tex')

%fig_Ktotal = figure('Name', 'Total Kinetic Energy');
%plot(t, sum(Krot.') + sum(Ktran.'))

end

