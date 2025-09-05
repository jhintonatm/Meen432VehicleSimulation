time_step = 0.01;
J = 1;
tau = 1;
b = 1;
w_initial = 1;

out = sim('MEEN_432_Proj1Part1');

time = out.tout;
w_dot = out.w_dot.data;    
w_k   = out.w_k.data;      

% Analytic solution (vectorized)
true_w = (tau/b) * (1 - exp(-b*time/J)) + w_initial * exp(-b*time/J);

% Error
error = w_k - true_w;   

figure;
plot(time, w_dot, 'LineWidth', 1.5);
hold on;
plot(time, w_k, '--', 'LineWidth', 1.5);
plot(time, error, ':', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Values');
legend('w\_dot', 'w\_k', 'error');
title('Simulation Outputs');
grid on;

