time_step = .001;
J = 1;
tau = 10;
b = 1;
w_initial = 0;

% Pick solver: 'ode1' (Euler), 'ode4' (RK4 fixed step)
solver = 'ode1';

model = 'MEEN_432_Proj1Part1';
load_system(model);

set_param(model, 'SolverType', 'Fixed-step', ...
                     'Solver', solver, ...
                     'FixedStep', num2str(time_step));



% Now run
out = sim(model);

time = out.tout;
w_dot = out.w_dot.data;    
w_k   = out.w_k.data;   
theta = out.theta.data;

comp_time = out.SimulationMetadata.TimingInfo.ExecutionElapsedWallTime;

% Analytic solution (vectorized)
true_w = (tau/b) * (1 - exp(-b*time/J)) + w_initial * exp(-b*time/J);

% Error
error = w_k - true_w;   
max_error = max(error)

% Plot
figure;
plot(time, true_w, 'LineWidth', 1.5);
hold on;
plot(time, w_k, '--', 'LineWidth', 1.5);
plot(time, error, ':', 'LineWidth', 1.5);

xlabel('Time (s)');
ylabel('Values');
legend('true w', 'w_k', 'error', 'theta');
title('Simulation Outputs');
grid on;


