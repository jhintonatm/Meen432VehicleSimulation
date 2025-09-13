time_steps = [0.001, 1];
solvers = {'ode1', 'ode4', 'ode45'};

j1 = 1;
j2 = 1;
tau_applied = 10;
tau_load = 1;
b1 = 1;
b2 = 1;
w_0 = 0;

model = 'MEEN_432_Proj1Part1';
load_system(model);

results = struct();

% --- Run simulations for all combinations ---
for s = 1:numel(solvers)
    for t = 1:numel(time_steps)
        solver = solvers{s};
        dt = time_steps(t);

        % configure solver
        set_param(model, 'SolverType', 'Fixed-step', ...
                         'Solver', solver, ...
                         'FixedStep', num2str(dt));

        % measure simulation CPU time
        tic;
        out = sim(model);
        sim_time = toc;

        % always take time from the signal itself
        time = out.w.Time;
        w = out.w.Data;

        % analytic solution using that time vector
        true_w = ((tau_applied - tau_load) / (b1 + b2)) .* ...
                 (1 - exp(-(b1 + b2) .* time / (j1 + j2))) + ...
                 w_0 .* exp(-(b1 + b2) .* time / (j1 + j2));

        % store results
        results.(solver)(t).time = time;
        results.(solver)(t).w = w;
        results.(solver)(t).true_w = true_w;
        results.(solver)(t).dt = dt;
        results.(solver)(t).sim_time = sim_time;
    end
end

%% --- Plot version 1: Each solver gets its own subplot ---
figure;
for s = 1:numel(solvers)
    subplot(1, numel(solvers), s);
    hold on;
    for t = 1:numel(time_steps)
        r = results.(solvers{s})(t)
        plot(r.time, r.w, '--', 'LineWidth', 1.5, ...
            'DisplayName', sprintf('dt=%.1f (%.4f s)', r.dt, r.sim_time));
    end
    xlabel('Time (s)');
    ylabel('w');
    title(sprintf('Solver: %s', solvers{s}));
    legend show;
    grid on;
end
sgtitle('Comparison by Solver (different dt on same plot)');

%% --- Plot version 2: Each timestep gets its own subplot ---
figure;
for t = 1:numel(time_steps)
    subplot(1, numel(time_steps), t);
    hold on;
    for s = 1:numel(solvers)
        r = results.(solvers{s})(t);
        plot(r.time, r.w, '--', 'LineWidth', 1.5, ...
            'DisplayName', sprintf('%s (%.4f s)', solvers{s}, r.sim_time));
    end
    xlabel('Time (s)');
    ylabel('w');
    title(sprintf('dt = %.3f s', time_steps(t)));
    legend show;
    grid on;
end
sgtitle('Comparison by Time Step (different solvers on same plot)');
