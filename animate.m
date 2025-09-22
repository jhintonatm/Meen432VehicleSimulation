% This animates the patch vehicle going around the generated track
function v_rot = rotate(v, theta)
    % Rotate 2D vector v by angle theta (radians)
    R = [cos(theta), -sin(theta);
         sin(theta),  cos(theta)];
    v_rot = R * v;
end
%% From a Kinematic Model
simout = sim("Project_2_Kinematic_Model.slx");
car_X = simout.X.Data;
car_Y = simout.Y.Data;
car_psi = simout.psi.Data;
car_time = simout.tout;

path_X = path.xpath;
path_Y = path.ypath;

%% From no model
% xpath = path.xpath;
% ypath = path.ypath;
% theta = path.tpath;

%% Animating the patch
fh = figure();
fh.WindowState = 'maximized';
hold on
plot(path.xpath,path.ypath,'--r'); axis equal; % plots center line
plot(path.xinpath, path.yinpath, 'b'); axis equal; % plots inside border 
plot(path.xoutpath, path.youtpath,'b'); axis equal; % plots outside border
axis([min(path.xoutpath) , max(path.xoutpath) , min(path.youtpath) , max(path.youtpath)]) % limits axis for better viewing
xlabel('X Distance (m)') % x axis label
ylabel('Y Distance (m)') % y axis label
title('Project 2 Track') % title
grid
h = animatedline; % handle of animatedline

L = 15;
width = 5;
for i = 1:length(car_X)
    x = car_X(i);
    y = car_Y(i);
    psi = car_psi(i);
    
  
    car = [-L/2 -width/2; -L/2 width/2; L/2 width/2; L/2 -width/2]; % array of the vertices of the car
    rcar = rotate(car', 0)'; 
    a = polyshape(rcar + [x,y]);
    ap = plot(a); % plotting the shape of the car
    drawnow limitrate
    ap.FaceColor = 'k'; % makes car black
    drawnow limitrate 
    pause(0.05);
    delete(ap)
end
function stats = raceStat(car_X, car_Y, car_time, path)
    % Compute race statistics and check path adherence

    % 1. Total Distance Traveled
    dx = diff(car_X);
    dy = diff(car_Y);
    total_distance = sum(sqrt(dx.^2 + dy.^2));

    % 2. Total Time and Average Speed
    total_time = car_time(end) - car_time(1);
    avg_speed = total_distance / total_time;


    for i = 1:length(car_X)
        % Find the closest centerline point to the car
        dx = path.xpath - car_X(i);
        dy = path.ypath - car_Y(i);
        dist = sqrt(dx.^2 + dy.^2);
        [min_dist, ~] = min(dist);
    end

    % 5. Package into output struct
    stats = struct();
    stats.total_distance = total_distance;
    stats.total_time = total_time;
    stats.avg_speed = avg_speed;


    % 6. Display basic info
    fprintf('--- Race Statistics ---\n');
    fprintf('Total Distance Traveled: %.2f m\n', total_distance);
    fprintf('Total Time: %.2f s\n', total_time);
    fprintf('Average Speed: %.2f m/s\n', avg_speed);

end
race = raceStat(car_X, car_Y, car_time, path)
hold off
