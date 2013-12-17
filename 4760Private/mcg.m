clear all

Fs = 100; % Hz
dt = 1/Fs;
t = 0:dt:5;

m_input = zeros(size(t));
m_output = zeros(size(t));
c_error = zeros(size(t));

motor_cutoff_freq = 0.5; % Hz;
motor_off_cutoff = 0.2; % Hz

sensor_pos = zeros(size(t));
sensor_output = zeros(size(t));

raw_sensor = 0;

rps = 10;
% up
c_input = rps* (t > .1);
% up
c_input (t > 1.25) = 3 * rps;
% down
c_input ( t > 3.0) = rps;

A = 20;
B = 0.02;
pid_integral = 0;
m_max = 50;

for i = 4 : length(t)
	%sensor update, integrate for new shaft position
	sensor_pos(i) = sensor_pos(i-1) + dt * m_output(i-1);
	%if one full rotation has occured, update sensor
	% else extrapolate from last measurements
	if fix(sensor_pos(i)) > fix(sensor_pos(i-1))

		raw_sensor = m_output(i-1);
		sensor_output(i) = raw_sensor;
	else
		sensor_output(i) = raw_sensor;
	end

	%control error
	c_error(i) = c_input(i) - sensor_output(i);
	%PID algorithm for a system with no negative output
	% get integral error term 
	if sign(c_error(i)) == sign(c_error(i-1))
		pid_integral = pid_integral + c_error(i);
	else

		pid_integral = 0;
	end	



	m_input(i) = ...
		A * c_error(i) + ...
		B * pid_integral + ...
		C * ( c_error(i) - c_error(i-1));
	m_input 

end
