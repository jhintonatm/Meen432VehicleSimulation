time_step=1
sim(MEEN_432_Proj1Part1)
time = out.tout;           % Time array
w_dot = out.w_dot.data;    % Data from w_dot timeseries
w_k   = out.w_k.data;      % Data from w_k timeseries




   

figure;
plot(time, w_dot, 'LineWidth', 1.5);
hold on;
plot(time, w_k, '--', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Values');
legend('w\_dot', 'w\_k');
title('Simulation Outputs');
grid on;

