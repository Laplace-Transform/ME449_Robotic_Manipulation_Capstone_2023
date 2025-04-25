clear
global Tf N
Tb0 = [1 0 0 0.1662
        0 1 0 0
        0 0 1 0.0026
        0 0 0 1];
T0e = [1 0 0 0.033
        0 1 0 0
        0 0 1 0.6546
        0 0 0 1];
Tbe = Tb0*T0e;
%% 
Tse_init = [1 0 0 0
            0 1 0 0
            0 0 1 0.0963+0.0026+0.6546
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
                -sqrt(2)/2 0 -sqrt(2)/2 0.5;
                0 0 0 1];
k = 1;

[Tse_N] = TrajectoryGenerator(Tse_init,Tsc_init,Tsc_final,Tce_grasp,Tce_standoff,k);

%%
gripper_state =0;%%default
forcsv = [];
% for i = 1:8
%     if i >= 3 && i <= 7
%         gripper_state =1;
%     else
%         gripper_state =0;
%     end


    for j = 1:length(Tse_N)

            forcsv = [forcsv
                Tse_N{1,j}(1,1) Tse_N{1,j}(1,2) Tse_N{1,j}(1,3)...
                Tse_N{1,j}(2,1) Tse_N{1,j}(2,2) Tse_N{1,j}(2,3)...
                Tse_N{1,j}(3,1) Tse_N{1,j}(3,2) Tse_N{1,j}(3,3)...
                Tse_N{1,j}(1,4) Tse_N{1,j}(2,4) Tse_N{1,j}(3,4)...
                gripper_state];
    end
% end

gripper_open_start_point = N(1)+N(2);
gripper_open_end_point =N(1)+N(2)+N(3)+N(4)+N(5)+N(6);
for i = gripper_open_start_point:gripper_open_end_point
    forcsv(i,13) = 1;
end
    
    
    
writematrix(forcsv ,'Trajectory Generation.csv')
