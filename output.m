function output(T, QQ, YY, WW, RR, DD)

switch YY
    case 1
        controller = 'linear PD control with gravity compensation';
    case 2
        controller = 'PID control with gravity compensation';
    case 3
        controller = 'Trajectory tracking control law - Z axis PID controller';
    case 4
        controller = 'Sliding mode 1st order (sign)';
    case 5
        controller = 'Super-twisting';
    otherwise
        controller = 'Invalid controller selection';
end

switch WW
    case 1
        solver = 'Fixed-step Runge-Kutta 4th order';
    case 2
        solver = 'ODE Runge-Kutta (variable step)';
    otherwise
        solver = 'Invalid solver selection';
end

switch RR
    case 1
        reference = 'Z step reference, X & Y = 0';
    case 2
        reference = 'Spiral trajectory';
    otherwise
        reference = 'Invalid reference selection';
end

switch DD
    case 0
        disturbance = 'without disturbance';
    case 1
        disturbance = 'single wind gust at T/2';
    case 2
        disturbance = 'four wind gusts (i) at 5+i*T/4, same direction';
    case 3
        disturbance = 'four wind gusts (i) at 5+i*T/4, alternating direction';
    otherwise
        disturbance = 'Invalid reference selection';
end

fprintf('QUADROTOR HELICOPTER MODEL SIMULATION \n');
fprintf('Simulation runtime: %d sec.\n', T);
fprintf('Running MODEL %d \n', QQ);
fprintf('Selected controller:  YY = %d - %s \n', YY, controller);
fprintf('Selected solver:      WW = %d - %s \n', WW, solver);
fprintf('Selected reference:   RR = %d - %s \n', RR, reference);
fprintf('Selected disturbance: DD = %d - %s \n', DD, disturbance);
fprintf('\nRunning...\n');

end

