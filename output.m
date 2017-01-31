function output(T, QQ, YY, WW, RR, DD, SF, EE, SAT, mm, Ixx, Iyy, Izz, b, d, l, AngVel_limit)

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
        controller = 'Super-twisting (2nd order sliding mode) algorithm';
    case 6
        controller = '1-SM trajectory tracking control law';
    case 7
        controller = 'Super-twisting trajectory tracking control law';
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
    case 3
        reference = 'Sinusoidal function';
    otherwise
        reference = 'Invalid reference selection';
end

switch DD
    case 0
        disturbance = 'Without disturbance';
    case 1
        disturbance = 'Single wind gust at T/2';
    case 2
        disturbance = 'Four wind gusts (i) at 5+i*T/4, same direction';
    case 3
        disturbance = 'Four wind gusts (i) at 5+i*T/4, alternating direction';
    case 4
        disturbance = 'Rapid alternating wave disturbance';
    otherwise
        disturbance = 'Invalid reference selection';
end

switch SF
    case 0
        smoothing = 'Z reference w/o smoothing filter';
    case 1
        smoothing = 'Z reference w/ smoothing filter 1st order';
    case 2
        smoothing = 'Z reference w/ smoothing filter 2nd order';
    case 3
        smoothing = 'Z reference w/ nonlinear saturated smoothing filter';
    otherwise
        smoothing = 'Invalid smoothing filter selection';
end

switch EE
    case 0
        estimator = 'No error derivative estimator used';
    case 1
        estimator = 'Linear error derivative estimator';
    case 2
        estimator = 'Super-twisting error derivative estimator';
    otherwise
        estimator = 'Invalid error derivative estimator selection';
end

switch SAT
    case true
        saturation = 'Motor saturation enabled';
    case false
        saturation = 'Motor saturation disabled';
    otherwise
        saturation = 'Error! Check settings';
end


fprintf('QUADROTOR HELICOPTER MODEL SIMULATION \n \n');
fprintf('Using quadrotor data: \n     mm = %.2f, Ixx = %.4f, Iyy = %.4f, Izz = %.4f \n', mm, Ixx, Iyy, Izz);
fprintf('Using motor data: \n     b = %.4e, d = %.4e, l = %.4f \n', b, d, l);
fprintf('Motor saturation limit: \n     AngVel_limit = %d rad/s\n\n', AngVel_limit);
fprintf('Simulation runtime: %d sec.\n', T);
fprintf('Running MODEL %d \n', QQ);
fprintf('Selected controller:  YY = %d - %s \n', YY, controller);
fprintf('Selected solver:      WW = %d - %s \n', WW, solver);
fprintf('Selected reference:   RR = %d - %s \n', RR, reference);
fprintf('Selected smoothing:   SF = %d - %s \n', SF, smoothing);
fprintf('Selected estimator:   EE = %d - %s \n', EE, estimator);
fprintf('Selected disturbance: DD = %d - %s \n', DD, disturbance);
fprintf('Selected saturation:  SAT = %d - %s \n', SAT, saturation);
fprintf('\nRunning calculations... ');

end

