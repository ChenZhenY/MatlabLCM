%% Setting up the lcm type and cores
% To skip the compile process, I provide compiled lcm and my_types.jar
% here. If you want to use latest version in RobotSoftware, you should
% compile RS and find these two files in
% ${RobotSoftwareDirname}/lcm-types/java

% This file is supposed to run in /MatlabLCM. If you run in other dir, make
% sure add the path of lcm.jar and my_types.jar
clear clc

javapath = javaclasspath('-all');
if ~contains(javapath, 'java/lcm.jar')
     javaaddpath java/lcm.jar
end
if ~contains(javapath, 'java/my_types.jar')
    javaaddpath java/my_types.jar
end

publisher = lcm.lcm.LCM.getSingleton();
humanoid_state_msg = lcmtypes.humanoid_state_info_lcmt();
debug_visual_msg   = lcmtypes.debug_visualization_lcmt();

%% arrow msg initialization
num_arrows = 2;
debug_visual_msg.arrow_count=num_arrows;
debug_visual_msg.arrow_elements=num_arrows;
debug_visual_msg.arrow_position_elements=num_arrows*3;
debug_visual_msg.arrow_color_elements=num_arrows*4;

debug_visual_msg.arrow_base_positions = zeros(num_arrows*3,1);
debug_visual_msg.arrow_directions = zeros(num_arrows*3,1);
debug_visual_msg.arrow_nom = [30, 30];
debug_visual_msg.arrow_ids = ["GRF 1", "GRF 2"]; 
debug_visual_msg.arrow_colors = zeros(num_arrows*4,1);

for i=1:num_arrows
    debug_visual_msg.arrow_colors(4*(i-1)+1)= 1;
    debug_visual_msg.arrow_colors(4*(i-1)+2)= 0;
    debug_visual_msg.arrow_colors(4*(i-1)+3)= 0;
    debug_visual_msg.arrow_colors(4*(i-1)+4)= 0.7;
end

%% Loading and Processing mat.file
% matfile: x y z roll pitch yaw right(Hipz Hipx Hipy Knee Ankle)7-11 left(Hipz Hipx Hipy Knee Ankle)12-16 rightArm leftArm

data = load("JointInput/example1_interpolate.mat");
%data = load("JointInput/example1.mat");

t_all = length(data.unity.time);
ctrl_all = data.unity.state;

sim_freq = 250; % current default freq is 2kHz, interpolate 250Hz
t_step = 1/sim_freq;

% Note: all linear xyz follow the visualized coordinates
% rotation frame follow the left-hand rule in unity(direction cw)

for i = 1:t_all
    raw_pos = [ctrl_all(i,1), ctrl_all(i,2), ctrl_all(i,3)];
    humanoid_state_msg.body_pos = raw_pos.';
    humanoid_state_msg.jpos(:)  = ctrl_all(i,7:24);
    rotangle = ctrl_all(i, 4:6);
    % humanoid_state_msg.body_ori_quat_visual(:) = angle2quat(-rotangle(1), -rotangle(2), -rotangle(3), 'XZY');
    rotM = rotationMatrix(-rotangle(3),'y')*rotationMatrix(-rotangle(2), 'z')*rotationMatrix(-rotangle(1),'x')*rotationMatrix(pi/2, 'y');
    humanoid_state_msg.body_ori_quat_visual(:) = rotm2quat(rotM);

    debug_visual_msg.arrow_base_positions(1)= i/500;
    debug_visual_msg.arrow_base_positions(2)= 1;
    debug_visual_msg.arrow_base_positions(3)= 1;
    debug_visual_msg.arrow_directions(1) = 30;
    debug_visual_msg.arrow_directions(2) = 0;
    debug_visual_msg.arrow_directions(3) = 0;
%     debug_visual_msg.arrow_base_positions(1)= raw_pos(1);
%     debug_visual_msg.arrow_base_positions(2)= raw_pos(2);
%     debug_visual_msg.arrow_base_positions(3)= raw_pos(3);
%     debug_visual_msg.arrow_directions(1) = raw_pos(1) ;
%     debug_visual_msg.arrow_directions(2) = raw_pos(2)+ i/10;
%     debug_visual_msg.arrow_directions(3) = raw_pos(3);

%     debug_visual_msg.arrow_base_positions(3*(i-1)+1)= 0.02*t;
%     debug_visual_msg.arrow_base_positions(3*(i-1)+2)= 0.03*t;
%     debug_visual_msg.arrow_base_positions(3*(i-1)+3)=0 ;
%     debug_visual_msg.arrow_directions(3*(i-1)+1)= t;
%     debug_visual_msg.arrow_directions(3*(i-1)+2)= t;
%     debug_visual_msg.arrow_directions(3*(i-1)+3)= t;
%     debug_visual_msg.arrow_directions(4) = -t;

    publisher.publish('humanoid_visualization_info', humanoid_state_msg);
    publisher.publish('debug_visualization', debug_visual_msg);
    pause(t_step);
end