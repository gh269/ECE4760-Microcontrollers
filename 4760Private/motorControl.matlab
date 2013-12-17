% motor control test
% includes
% -- motory dynamics
% -- PID controller rectification and saturation
% -- IR speed sensor model with new rps once/revolution

clear all


Fs = 100 ;
dt = 1/Fs ;
t=0:dt:5 ;

m_input = zeros(size(t)) ; % init motor input array
m_output = zeros(size(t)) ;% starts at zero rps
c_error = zeros(size(t)) ;% starts at zero rps


% motor model is a 2-pole low pass when under power
% but is simple 1-pole lowpass when no power is applied
motor_cutoff_freq = 0.5; %Hz
motor_off_cutoff = 0.2; %Hz
[b_motor_on, a_motor_on] = butter(2, motor_cutoff_freq/(Fs/2)) ;
[b_motor_off, a_motor_off] = butter(1, motor_off_cutoff/(Fs/2)) ;

% Sensor model is a reflective IR detector which records one edge/revolution
% sensor_pos is the integral of the speed
% the speed feedback is updated every time sensor_pos increases by one
% full revolution
sensor_pos = zeros(size(t)) ; %units are one full revolution
sensor_output = zeros(size(t)) ;% starts at zero rps
raw_sensor = 0 ;

% control input revolutions/sec
rps = 10 ;
% up-step
c_input =  rps * (t>.1);
% another upstep
c_input(t>1.25) = 3 * rps ;
% down-step at t=3
c_input(t>3.0) = rps ;

% PID parameters
A = 20 ;
B = 0.02 ; pid_integral = 0;
C = 30 ;
% maximum motor input drive
m_max = 50 ;

for i = 4:length(t)
   
    %sensor update
    % integrate to get new shaft position
    sensor_pos(i) = sensor_pos(i-1) + dt * m_output(i-1) ;
    % if one full rotation has occured, then update sensor
    % otherwise extrapolate from last measurements
    if fix(sensor_pos(i)) > fix(sensor_pos(i-1))
      raw_sensor = m_output(i-1) ;
      sensor_output(i) = raw_sensor ;
    else
      sensor_output(i) = raw_sensor ;
    end
   
    % control error
    c_error(i) = c_input(i) - sensor_output(i) ;
   
    % PID algorithm for a system with NO negative output
    % get integral error term
    if sign(c_error(i)) == sign(c_error(i-1))
        pid_integral = pid_integral + c_error(i) ;
    else
        pid_integral = 0;
    end
   
    % sum the PID terms,  zero neg output, and limit max
    m_input(i) = ...
            A * c_error(i) + ...
            B * pid_integral  + ...
            C * (c_error(i)-c_error(i-1))  ;
    m_input(i) = min(m_input(i) * (m_input(i)>0), m_max) ;
   
    % compute motor response to input
    if m_input(i)>0
      m_output(i) = ...
            b_motor_on(1)*m_input(i) + ...
            b_motor_on(2)*m_input(i-1) + ...
            b_motor_on(3)*m_input(i-2) - ...
            a_motor_on(2)*m_output(i-1) - ...
            a_motor_on(3)*m_output(i-2) ;
    else
      m_output(i) = ...
            b_motor_off(1)*m_input(i) + ...
            b_motor_off(2)*m_input(i-1) - ...
            a_motor_off(2)*m_output(i-1)  ;
    end
   
end

figure(1); clf;
plot(t, c_input);
set (gca, 'ylim', [0, 55])
hold on
plot(t, m_output,'r');
plot(t, m_input, 'g') ;
legend('command','motor rps', 'motor drive')