function [] = ulift(Kp, Ki, Kd)


%% PID controller coefficients
% Kp = proportional
% Ki = integral
% Kd = derivative


%% physical parameters
mass = 1500;                    % mass of train
height = 20;                         % diplacement between point A and B (meters)
gravity = 9.8;                        % acceleration due to gravity
b = 1000;                       % coefficient of dynamic friction

%% simulation time
Time = 50;                     % simulation time
dt = 0.01;                      % simulation resolution
N = Time/dt +1;                 % number of samples


%% simulation variables
t=0:dt:Time;                    % time vectors
Velocity=zeros(1,N);               % angular velocity in rads/sec
acceleration=zeros(1,N);                   % acceleration
acceleration=zeros(1,N);               % accelration
displacement=zeros(1,N);        % displacement
Force_net=zeros(1,N);           % net force acting on pendulum
error = height*ones(1,N);            % error in displacement
error_I=0;                      % integral of error
error_D=0;                      % derivative of error
F_drive=0;                          % toruque, to be calculated by PID controller
h=[linspace(0,20,20/dt) 20*ones(1,50/dt-20/dt)];

%% calculation
for n=2:N-1
  %% Process
  acceleration(n) = Force_net(n-1)/mass;                               % lift acceleration 
  
  Velocity(n) = Velocity(n-1) + 0.5*(acceleration(n-1)+acceleration(n))*dt;                     % velocity
  displacement(n) = displacement(n-1) + 0.5*(Velocity(n-1)+Velocity(n))*dt;       % displacement
  F_reaction =b*Velocity(n);                                               % reaction force        
  F_gravity = mass*gravity;                                       % gravitaition force
  
  %% Feedback error
  error(n) = h(n)-displacement(n);                         % error in displacement

  %% Control
  error_I = error_I + 0.5*(error(n)+error(n-1))*dt;         % integral
  error_D = (error(n)-error(n-1))/dt;                       % derivative
  F_drive = Kp*error(n) + Ki*error_I + Kd*error_D;          % driving force
   if n < 500 
    if F_drive < mass*gravity
        F_drive = mass*gravity;
    end
   end
   if (F_drive > 20000)
      F_drive=20000;                                             % maximum limit of force 20kN
  end
  if (F_drive < -20000)
      F_drive=-20000;                                            % minimum limit of force -20kN
  end
  if ((F_drive - F_reaction-F_gravity)/mass  > 2.5)
      F_drive = F_reaction + F_gravity + 2.5*mass;                                        % maximum limit of force 2.5ms2
  end
  if((F_drive - F_reaction-F_gravity)/mass <- 2.5)
          F_drive = F_reaction + F_gravity -2.5*mass;
  end

  Force_net(n) = F_drive - F_reaction-F_gravity;                        % net force
end
%% Plotting 

plot(t,displacement);
ylabel('displacement (meters)');
xlabel('time (s)');
title('Displacement vs Time');
hold on
plot(t(1:end-1),h);
ylabel('displacement (meters)');
xlabel('time (s)');
title('Displacement vs Time');
hold off
figure;
plot(t,acceleration);
ylabel('acceleration(meters per second squares)');
xlabel('time (s)');
title('Acceleration vs Time');


% lift(13000,193,32000)
