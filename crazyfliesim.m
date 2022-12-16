start_time = 0;
end_time = 10;
dt =  0.5;
times = start_time:dt:end_time;
N = numel(times);

state = zeros(6,1);
omega = zeros(3,1);

I = [1 2 3; 4 5 6; 7 8 9;];
moments = [1; 2; 3];


for t = times
    omegadot = angular_acceleration(moments, omega, I);
    omega = omega + dt*omegadot;
end


function omegadot = angular_acceleration(moments, omega, I)
    omegadot = (moments - cross(omega, I*omega))/I; 
end 

    %phi_dot = p + (q*sin(phi) + r*cos(phi)) * (sin(theta)/cos(theta));
    %theta_dot = q*cos(phi) - r*sin(phi);
    %psi_dot = (q*sin(phi) + r*cos(phi)) / cos(theta);

%     p_dot = (1/I(1,1))*(ell + (I(2,2) - I(3,3))*q*r);
%     q_dot = (1/I(2,2))*(m + (I(3,3) - I(1,1))*p*r);
%     r_dot = (1/I(3,3))*(n + (I(1,1) - I(2,2))*p*q);
%     

mass = 27/1000; %kg
g = 9.81; %kg/m^2
F_gravity = [-mass*g*sin(theta);
             mass*g*sin(phi)*cos(theta);
             mass*g*cos(phi)*cos(theta)]; %in body frame