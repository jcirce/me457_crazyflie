motordata = readtable('trajectory-stablemotordata.csv','NumHeaderLines',1);  
statedata = readtable('trajectory-stable.csv', 'NumHeaderLines',1);

t = motordata(:, 1);
time = table2array(t);
state_time = table2array(statedata(:,1));


m1 = motordata(:, 2); %motor pwm from .csv
m2 = motordata(:, 3);
m3 = motordata(:, 4);
m4 = motordata(:, 5);
roll_raw = motordata(:,6);
pitch_raw = motordata(:,7);

motor1 = table2array(m1);
motor2 = table2array(m2);
motor3 = table2array(m3);
motor4 = table2array(m4);
roll = table2array(roll_raw);
pitch = table2array(pitch_raw);

motors = horzcat(motor1, motor2, motor3, motor4);

% linear map motor 0-65535 pwm to 0-256
scale = 256/65355; 
pwm = motors*scale;

%from bitcraze.io
thrust = (0.409e-3)*pwm.^2 + (140.5e-3)*pwm - 0.099; %kg

stabilizer_roll = table2array(statedata(:,2));
stabilizer_pitch = table2array(statedata(:,3));
stateestimate_rate_roll = table2array(statedata(:,4))*180.0/(pi*1000.0);
stateestimate_rate_pitch = table2array(statedata(:,5))*180.0/(pi*1000.0);
controller_pitchrate = table2array(statedata(:,6));
controller_rollrate = table2array(statedata(:,7));
roll_alt = table2array(statedata(:,8));
pitch_alt = table2array(statedata(:,9));

%plotters

tiledlayout(3,1)

% nexttile
% plot(state_time, stabilizer_roll, 'r')
% hold on
% plot(state_time, stabilizer_pitch, 'b')
% ylim([-25 25])
% title('Stabilizer Roll and Pitch')

nexttile
plot(state_time, stateestimate_rate_pitch, 'b')
hold on
plot(state_time, stateestimate_rate_roll, 'r')

plot(state_time, controller_pitchrate, 'c')
hold on
plot(state_time, controller_rollrate, 'color', '#D95319')

title('state estimate and controller output roll and pitch rates')

nexttile
plot(time, roll, 'r')
hold on
plot(time, pitch, 'b')
ylim([-25 25])
hold on
plot(state_time, stabilizer_roll, 'm')
hold on
plot(state_time, -1*stabilizer_pitch, 'k') %inverted
title('desired & measured roll and pitch')

nexttile
plot(time, thrust(:,1))
hold on
plot(time, thrust(:,2))
hold on
plot(time, thrust(:,3))
hold on
plot(time, thrust(:,4))
title('thrusts for each motor')

