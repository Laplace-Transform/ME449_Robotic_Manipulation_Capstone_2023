function [Tse_N] = TrajectoryGenerator(Tse_init,Tsc_init,Tsc_final,Tce_grasp,Tce_standoff,k)
global Tf N
Tf = [1 1 1 1 1 1 1 1];%Segment time of the motion Tf in seconds from rest to rest
N = Tf.*k./0.01;%The number of points
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