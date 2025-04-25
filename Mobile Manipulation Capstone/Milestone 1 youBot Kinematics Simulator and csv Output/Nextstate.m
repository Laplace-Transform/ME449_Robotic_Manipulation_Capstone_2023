clear
clc
dt = 0.01;
u_max = 5;
control = [10 10 10 10 0 0 0 0 0 0]';
cfg = [0 0.2 0.2 0 0 0 0 0 0 0 0 0];
cfg_cvs =[];
for i = 1:100
    cfg = NextState(cfg,control, dt, u_max);%new configuration;
    cfg_cvs(i,1:12)  = cfg;
	cfg_cvs(i,13) = 0;
end
cfg_cvs = [cfg 0
            cfg_cvs];% add the first line
writematrix(cfg_cvs ,'Trajectory Generation.csv')

function Configuration_Next = NextState(Configuration_Now, control, dt, u_max)
%configuration: 1:3 chassis 4:8 arm 9:12 wheel
%control: 1:4 wheel speedd 5:9 joint speed
u = control(1:4);
joint = control(5:9);

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
Configuration_Next(9:12) = Configuration_Now(9:12) + u'*dt;
end