clear

disp("Running FinalProject_Overshoot")
%% setup time interval of segments
global Tf N
Tf = [1 1 1 1 1 1 1 1];%Segment time of the motion Tf in seconds from rest to rest
k = 1;%The number of trajectory reference configurations per 0.01 seconds:
N = Tf.*k./0.01;%The number of points

%% initial configs
Tsb = [1 0 0 0;
         0 1 0 0;
         0 0 1 0.0963;
         0 0 0 1];
Tb0 = [1 0 0 0.1662
        0 1 0 0
        0 0 1 0.0026
        0 0 0 1];
T0e = [1 0 0 0.033
        0 1 0 0
        0 0 1 0.6546
        0 0 0 1];
%Tbe = Tb0*T0e;
%% configs for TrajectoryGenerator
Tse_init = [0 0 1 0
            0 1 0 0
            -1 0 0 0.5
            0 0 0 1];

Tsc_init = [1 0 0 1
            0 1 0 0
            0 0 1 0.025
            0 0 0 1];
        
Tsc_final = [0 1 0 0
            -1 0 0 -1
            0 0 1 0.025
            0 0 0 1];   
        
Tce_grasp = [-sqrt(2)/2 0 sqrt(2)/2 0;
            0 1 0 0;
            -sqrt(2)/2 0 -sqrt(2)/2 0;
            0 0 0 1];


Tce_standoff = [-sqrt(2)/2 0 sqrt(2)/2 0;
                0 1 0 0;
                -sqrt(2)/2 0 -sqrt(2)/2 0.1;
                0 0 0 1];

% Body Jacobian
B1 = [0 0 1 0 0.033 0]';
B2 = [0 -1 0 -0.5076 0 0]';
B3 = [0 -1 0 -0.3526 0 0]';
B4 = [0 -1 0 -0.2176 0 0]';
B5 = [0 0 1 0 0 0]';
Blist = [B1 B2 B3 B4 B5];
%% F = pinv( H(0) )
l = 0.47/2;
r = 0.0475;
w = 0.3/2;
F = r/4.*[-1/(l+w) 1/(l+w) 1/(l+w) -1/(l+w)
    1 1 1 1
    -1 1 -1 1];%b frame
%% run TrajectoryGenerator
[Tse_N] = TrajectoryGenerator(Tse_init,Tsc_init,Tsc_final,Tce_grasp,Tce_standoff,k);
% Tse_N is 1x800 config cell, need to convert to 800x13 matrix
gripper_state =0;%%default
forcsv = [];
% for i = 1:8
%     if i >= 3 && i <= 7
%         gripper_state =1;
%     else
%         gripper_state =0;
%     end
% end
    for j = 1:length(Tse_N)

            forcsv = [forcsv
                Tse_N{1,j}(1,1) Tse_N{1,j}(1,2) Tse_N{1,j}(1,3)...
                Tse_N{1,j}(2,1) Tse_N{1,j}(2,2) Tse_N{1,j}(2,3)...
                Tse_N{1,j}(3,1) Tse_N{1,j}(3,2) Tse_N{1,j}(3,3)...
                Tse_N{1,j}(1,4) Tse_N{1,j}(2,4) Tse_N{1,j}(3,4)...
                gripper_state];
    end


gripper_open_start_point = N(1)+N(2);
gripper_open_end_point =N(1)+N(2)+N(3)+N(4)+N(5)+N(6);
for i = gripper_open_start_point:gripper_open_end_point
    forcsv(i,13) = 1;
end
writematrix(forcsv ,'Overshoot_Trajectory Generation Gripper.csv')

%% FeedbackControl
Kp = 5*eye(6);
Ki = 90*eye(6);
dt = 0.01;

cfg = [0.2 -0.2 0.2 0 0 0 0 0 0 0 0 0];% chassis phi x y, j12345 w1234
cfg_cvs = zeros(length(Tse_N)-1,13); %store every k-th configuration 


X = Tse_init;%  X = Tsb*Tb0*T0e;
F6 = zeros(6,4); 
F6(3:5,1:4) = F;% full matrix
u_max = 10000;
Xerr_rec = [];% prepare the Xerr file
for i = 1:length(Tse_N)-1 %loop of N-1 times
    Xd = Tse_N{i};
    Xd_n = Tse_N{i+1};
    [V,Xerr_k] = FeedbackControl(X, Xd, Xd_n, Kp, Ki, dt);% calculate the control law 
    Xerr_k = Xerr_k';
    Xerr_rec = [Xerr_rec % record the Xerr file
                Xerr_k];
    %generate the wheel and joint controls 
    thetalist = cfg(4:8); 
    Jb_arm = JacobianBody(Blist,thetalist);% eframe
    T_0e = FKinBody(T0e,Blist,thetalist');
    Jb_base = Adjoint(TransInv(T_0e)*TransInv(Tb0))*F6;% eframe
    Je = [Jb_base Jb_arm];
    u_control = pinv(Je,1e-2)*V;
    
    cfg = NextState(cfg,u_control, dt, u_max);%new configuration;
    cfg_cvs(i,1:12)  = cfg;
	cfg_cvs(i,13) = forcsv(i,13);
    %store every displaystyle k th configuration for later animation
    
    %find X for next iteration
    phi = cfg(1);
    x = cfg(2);
    y = cfg(3);
    Tsb = [cos(phi)    -sin(phi)   0   x
            sin(phi)     cos(phi)   0   y
            0            0          1   0.0963
            0            0          0   1];
	thetalist = cfg(4:8); %next iteration's joint angles
	T__0e = FKinBody(T0e,Blist,thetalist');
    X = Tsb*Tb0*T__0e;
end
disp("Generating animation csv file.")
cfg_cvs = [cfg 0
            cfg_cvs];% add the first line
writematrix(cfg_cvs ,'Overshoot_Trajectory Generation.csv')

disp("Writing error plot data.")
figure(1)
t = linspace(0,length(Xerr_rec)/100,length(Xerr_rec))';
plot1 = plot(t,Xerr_rec);
writematrix(Xerr_rec ,'Overshoot_Xerr data.csv')
title("Error");
xlabel("Time, seconds");
ylabel("Error,m/s,rad/s");
legend([plot1(1),plot1(2),plot1(3),plot1(4),plot1(5),plot1(6)],'\omega_{x}','\omega_{y}','\omega_{z}','v_{x}','v_{y}','v_{z}');

%% milestone 1 Kinematics Simulator
function Configuration_Next = NextState(Configuration_Now, control, dt, u_max)
%configuration: 1:3 chassis 4:8 arm 9:12 wheel
%control: 1:4 wheel speedd 5:9 joint speed

for i =1:9
    if control(i) > u_max
        control(i) = u_max;
    elseif control(i) <= u_max
        if control(i) >= -u_max
            control(i) = control(i);
        elseif control(i) < -u_max
                control(i) = -u_max;
        end
    end
end

u = control(1:4);
joint = control(5:9);

l = 0.47/2;
r = 0.0475;
w = 0.3/2;
F = r/4.*[-1/(l+w) 1/(l+w) 1/(l+w) -1/(l+w)
    1 1 1 1
    -1 1 -1 1];
Vb = F*u;

%change odometry(in b' frame) to b frame
if Vb(1)==0
    dqb = [0;Vb(2);Vb(3)];
else
    dqb = [Vb(1)
        (Vb(2)*sin(Vb(1))+Vb(3)*(cos(Vb(1))-1))/Vb(1)
        (Vb(3)*sin(Vb(1))+Vb(2)*(1-cos(Vb(1))))/Vb(1)];
end
% transform dqb(in b frame) to space frame.
phik = Configuration_Now(1);
T = [1 0 0
    0 cos(phik) -sin(phik)
    0 sin(phik) cos(phik)];
dq = T*dqb;

%update configuration
Configuration_Next = zeros(1,12);
Configuration_Next(1:3) = Configuration_Now(1:3) +dq'*dt;%chassis
Configuration_Next(4:8) = Configuration_Now(4:8) +joint'*dt;%joint
Configuration_Next(9:12) = Configuration_Now(9:12) + u'*dt;% wheel


end


%% milestone 2 Reference Trajectory Generation
function [Tse_N] = TrajectoryGenerator(Tse_init,Tsc_init,Tsc_final,Tce_grasp,Tce_standoff,k)
global Tf N
method = 3;% The time-scaling method

%eight-segment trajectory 1: initial configuration to a "standoff" 
Xstart = Tse_init;
Xend = Tsc_init*Tce_standoff;
traj_1 = CartesianTrajectory(Xstart,Xend,Tf(1),N(1),method);
%eight-segment trajectory 2: move the gripper down to the grasp position
Xstart = Xend;
Xend = Tsc_init*Tce_grasp;
traj_2 = CartesianTrajectory(Xstart,Xend,Tf(2),N(2),method);
%eight-segment trajectory 3: Closing of the gripper.
Xstart = Xend;
Xend = Tsc_init*Tce_grasp;
traj_3 = CartesianTrajectory(Xstart,Xend,Tf(3),N(3),method);
%eight-segment trajectory 4:move the gripper back up to the "standoff"
Xstart = Xend;
Xend = Tsc_init*Tce_standoff;
traj_4 = CartesianTrajectory(Xstart,Xend,Tf(4),N(4),method);
%eight-segment trajectory 5:move the gripper to a "standoff"above the final
Xstart = Xend;
Xend = Tsc_final*Tce_standoff;
traj_5 = CartesianTrajectory(Xstart,Xend,Tf(5),N(5),method);
%eight-segment trajectory 6: move the gripper to the final 
Xstart = Xend;
Xend = Tsc_final*Tce_grasp;
traj_6 = CartesianTrajectory(Xstart,Xend,Tf(6),N(6),method);
%eight-segment trajectory 7:Opening of the gripper
Xstart = Xend;
Xend = Tsc_final*Tce_grasp;
traj_7 = CartesianTrajectory(Xstart,Xend,Tf(7),N(7),method);
%eight-segment trajectory 8:move the gripper back to the "standoff"
Xstart = Xend;
Xend = Tsc_final*Tce_standoff;
traj_8 = CartesianTrajectory(Xstart,Xend,Tf(8),N(8),method);

Tse_N= [traj_1 traj_2 traj_3 traj_4 traj_5 traj_6 traj_7 traj_8];

end

%% milestone 3 Feedforward Control
function [V,Xerr_k] = FeedbackControl(X, Xd, Xd_n, Kp, Ki, dt)
%X = Tse
%Xd = Tse,d
%Xd_n = Tse,d,next

%V: ee twist in frame e
%Feed forward
Xerr_i = 0;
Vd = 1/dt*MatrixLog6(TransInv(Xd)*Xd_n);
Vd = se3ToVec(Vd); %to vector
%Proportional
Xerr_k = MatrixLog6(TransInv(X)*Xd);
Xerr_k = se3ToVec(Xerr_k); %to vector
%integral
Xerr_i = Xerr_i + Xerr_k *dt;

V = Adjoint(TransInv(X)*Xd)*Vd ...
    + Kp * Xerr_k...
    + Ki * Xerr_i;

end