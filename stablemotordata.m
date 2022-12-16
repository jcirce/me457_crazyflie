data = readtable('trajectory-stablemotordata.csv','NumHeaderLines',1);  

t = data(:, 1);
time = table2array(t);
zeroed_time = (time - 1311687)/1000;

m1 = data(:, 2); %motor pwm from .csv
m2 = data(:, 3);
m3 = data(:, 4);
m4 = data(:, 5);
roll_raw = data(:,6);
pitch_raw = data(:,7);

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

%convert thrust to torques L M N 
%92 mm length
arm = 92/2000; %m lever arm for moments
yaw_arm = sqrt(2)*arm;

L = zeros(length(motor1));
M = zeros(length(motor1));
N = zeros(length(motor1));

for i = 1:length(motor1)
    L(i) = arm*(thrust(i,1) - thrust(i,2) - thrust(i,3) + thrust(i,4));
    M(i) = arm*(-1*thrust(i,1) + thrust(i,2) - thrust(i,3) + thrust(i,4));
    N(i) = yaw_arm*(-1*thrust(i,1) - thrust(i,2) + thrust(i,3) + thrust(i,4));
end 

moments = [L(:,1) M(:,1) N(:,1)];

I = [16.571710 0.830806 0.718277; 
     0.830806 16.655602 1.800197; 
     0.718277 1.800197 29.261652]*10e-6; %kg*m^2
%from Forster, Hamer, D'Andrea - System Identification of ... 

%state = [p; q; r; phi; theta; psi];
% state = roll rate, pitch rate, yaw rate, roll, pitch, yaw
state = zeros(length(motor1),6);
ts = 0.001; %check this
%%

%in the loop
for i = 500:length(motor1)
    %pull states
    phi = state(i,1);   %roll
    theta = state(i,2); %pitch
    psi = state(i,3);   %yaw
    p = state(i,4);     %roll rate
    q = state(i,5);     %pitch rate
    r = state(i,6);     %yaw rate
    ell = moments(i,1); %L from motors
    m = moments(i,2);   %M from motors
    n = moments(i,3);   %N from motors
    
    %omega = p q r, angular velocity
    omega = [p; 
             q; 
             r];
    cor = [0 -r q; 
           r 0 -p; 
           -q p 0];
    loop_moments = [ell;
                    m;
                    n];
    
    %phidot thetadot psidot    
    euler_dot = [1 sind(phi)*tand(theta) cosd(phi)*tand(theta);
                 0 cosd(phi)            -sind(phi);
                 0 sind(phi)*secd(theta)  cosd(phi)*secd(theta)] * omega;
    
    omega_dot = inv(I) * (loop_moments - cor*I*omega);
    
    xdot = vertcat(euler_dot, omega_dot);
    %all 6 states

    %inner loop runs for each state
    for j = 1:6
    %RK4 integration
        k1 = xdot(j);
        %k2 = xdot(j+(ts/2)*k1);
        %k3 = xdot(j) + (ts/2)*k2;
        %k4 = xdot(j) + ts*k3;
        %state(i+1,j) = state(i,j) + (ts/6)*(k1 + 2*k2 + 2*k3 + k4);
        state(i+1,j) = state(i,j) + xdot(j)*ts;
    %end inner loop
    end
end
    
% %%
% t1 = 1:1505;
% t2 = 1:1504;
% plot(t2, roll, 'k')
% hold on
% plot(t2, pitch, 'r')
% hold on
% plot(t1, state(:,1), 'g')
% hold on
% plot(t1, state(:,2),'b')
    
            
            