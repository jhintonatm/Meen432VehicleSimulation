clear; close all; clc;

params = vehicleParams();

dt = 0.1;               
cycles = {"cycles/urban.csv","cycles/highway.csv"};
cycleNames = {"Urban","Highway"};

results = struct();

for c = 1:numel(cycles)
    fprintf('Loading cycle: %s\n', cycles{c});
    [t_ref, v_ref] = loadEPACycle(cycles{c}); 
    simOut = simulateVehicle(t_ref, v_ref, params, dt);
    results(c).name = cycleNames{c};
    results(c).t = simOut.t;
    results(c).v = simOut.v;
    results(c).torqueMotor = simOut.Tm;
    results(c).torqueWheel = simOut.Tw;
    results(c).P_elec = simOut.P_elec;
    results(c).energyConsumed = simOut.energyConsumed; 
    results(c).energyRegen = simOut.energyRegen;       
    results(c).SOC = simOut.SOC;
    
    fprintf('%s: Energy Consumed = %.3f kWh, Energy Recovered = %.3f kWh, SOC_final = %.2f%%\n\n',...
        cycleNames{c}, simOut.energyConsumed/3.6e6, simOut.energyRegen/3.6e6, simOut.SOC*100);
end

% ---- Plots ----
for c = 1:numel(results)
    figure('Name',results(c).name,'NumberTitle','off');
    subplot(3,1,1);
    plot(results(c).t, results(c).v*3.6); hold on; % km/h
    title([results(c).name ' - Speed (km/h)']);
    ylabel('v (km/h)');
    grid on;
    subplot(3,1,2);
    plot(results(c).t, results(c).torqueMotor); hold on;
    title('Motor Torque (Nm)');
    ylabel('T_m (Nm)');
    grid on;
    subplot(3,1,3);
    plot(results(c).t, results(c).P_elec/1000); hold on; % kW
    title('Battery Power (kW) (positive -> discharge)');
    ylabel('P_{batt} (kW)');
    xlabel('Time (s)');
    grid on;
end



function params = vehicleParams()
    % Vehicle & drivetrain parameters (example values; tune as needed)
    params.m = 1500;                % mass (kg)
    params.rw = 0.30;               % wheel radius (m)
    params.Cd = 0.28;               % drag coefficient
    params.Af = 2.2;                % frontal area (m^2)
    params.rho = 1.225;             % air density (kg/m^3)
    params.Cr = 0.015;              % rolling resistance coeff
    params.g = 9.81;                % gravity (m/s^2)
    params.eta_trans = 0.95;        % transmission efficiency (motoring)
    params.eta_trans_regen = 0.90;  % transmission efficiency (regen path)
    params.gear = 9.0;              % overall gear ratio (transmission * final drive)
    % Motor limits
    params.Tm_max = 220;            % max motor torque (Nm)
    params.P_max = 90000;           % max electrical power (W)
    params.omega_nom = 8000 * 2*pi/60; % nominal motor speed rad/s (for power curve)
    params.motor_efficiency = 0.92; % motoring efficiency
    params.regen_efficiency = 0.6;  % regen efficiency (fraction of mechanical braking returned)
    % Battery
    params.batt_Wh = 50e3;          % battery capacity (Wh) e.g. 50 kWh -> 50,000 Wh
    params.batt_J = params.batt_Wh * 3600; % J
    params.SOC0 = 0.8;              % start state of charge (0-1)
    params.V_nom = 350;             % nominal battery voltage (V) (used only if needed)
    % controller gains for driver
    params.Kp = 500;                % proportional gain maps speed error to wheel torque (Nm per m/s)
    params.Ki = 40;                 % integral gain
    params.maxBrakeTorque = -4000;  % max braking torque at motor side (negative)
end

function [t_ref, v_ref] = loadEPACycle(csvfile)
    
    if exist(csvfile,'file')
        data = readmatrix(csvfile);
        if size(data,2) < 2
            error('EPA CSV must have at least two columns: time_s, speed_mph');
        end
        t_ref = data(:,1);
        speed_mph = data(:,2);
    else
        warning('Cycle file %s not found. Using synthetic short cycle.', csvfile);
        t_ref = (0:1:600)';
        speed_mph = 25 + 10*sin(2*pi*t_ref/90);
    end
    v_ref = speed_mph * 0.44704; % convert mph to m/s
end

function simOut = simulateVehicle(t_ref, v_ref, params, dt)
    t_end = t_ref(end);
    t = (0:dt:t_end)';
    
    v_ref_sim = interp1(t_ref, v_ref, t, 'linear', 'extrap');
    N = numel(t);
    
    v = zeros(N,1);         
    x = zeros(N,1);         
    Tm = zeros(N,1);        
    Tw = zeros(N,1);        
    power_batt = zeros(N,1);
    energyConsumed = 0;     
    energyRegen = 0;        
    SOC = zeros(N,1);
    SOC(1) = params.SOC0;
    integral_err = 0;
    % Simulation loop
    for k = 1:N-1
        % finding the speed error
        err = v_ref_sim(k) - v(k);
        integral_err = integral_err + err*dt;
        % Controller -> desired wheel torque (Nm)
        % Use PI on speed to generate wheel force (approx) then convert to motor torque
 
        Tw_cmd = (params.Kp*err + params.Ki*integral_err); % Nm at wheel
        % Convert to motor torque command (account for gear and transmission)
        Tm_cmd = Tw_cmd / (params.gear * params.eta_trans);
        % Enforce motor torque and power limits
 
        omega_m = v(k)/params.rw * params.gear;
     
        Tm_cmd = max(min(Tm_cmd, params.Tm_max), params.maxBrakeTorque); 
      
        if Tm_cmd > 0
            P_mech = Tm_cmd * omega_m;
            P_elec = P_mech / params.motor_efficiency;
            if P_elec > params.P_max
                P_elec = params.P_max;
                P_mech = P_elec * params.motor_efficiency;
                Tm_cmd = P_mech / max(omega_m,1e-6);
            end
        else
            % regenerative braking: limit recovered power by P_max and regen efficiency
            P_mech = Tm_cmd * omega_m; % negative
            P_elec = P_mech * params.regen_efficiency; % negative magnitude smaller
            if abs(P_elec) > params.P_max
                P_elec = -params.P_max;
                P_mech = P_elec / params.regen_efficiency;
                Tm_cmd = P_mech / max(omega_m,1e-6);
            end
        end
        % Save motor torque
        Tm(k) = Tm_cmd;
        % Wheel torque after gear and transmission
        if Tm_cmd >= 0
            Tw_k = Tm_cmd * params.gear * params.eta_trans;
        else
            
            Tw_k = Tm_cmd * params.gear * params.eta_trans_regen;
        end
        Tw(k) = Tw_k;
        
        F_drive = Tw_k / params.rw;
        
        F_aero = 0.5 * params.rho * params.Cd * params.Af * v(k)^2;
        F_roll = params.Cr * params.m * params.g * sign(v(k)+1e-6);
        
        a = (F_drive - F_aero - F_roll)/params.m;
        
        v(k+1) = max(v(k) + a*dt, 0); 
        x(k+1) = x(k) + v(k)*dt;
        
        P_mech = Tm_cmd * omega_m;
        
        if P_mech >= 0
            
            P_batt = P_mech / params.motor_efficiency;
        else
            
            P_batt = P_mech * params.regen_efficiency;
        end
        
        power_batt(k) = P_batt; 
        
        E_step = P_batt * dt; 
        if E_step >= 0
            energyConsumed = energyConsumed + E_step;
        else
            energyRegen = energyRegen + abs(E_step);
        end
        SOC(k+1) = max(0, min(1, SOC(k) - E_step/params.batt_J));
    end
    %saving the ouputs
    simOut.t = t;
    simOut.v = v;
    simOut.Tm = Tm;
    simOut.Tw = Tw;
    simOut.P_elec = power_batt;
    simOut.energyConsumed = energyConsumed;
    simOut.energyRegen = energyRegen;
    simOut.SOC = SOC(end);
end
