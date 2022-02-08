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

%% sphere and arrow msg initialization
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

num_sphere = 1;
debug_visual_msg.sphere_count = num_sphere;
debug_visual_msg.sphere_elements = num_sphere;
debug_visual_msg.sphere_position_elements = 3*num_sphere;
debug_visual_msg.sphere_color_elements = 4*num_sphere;
debug_visual_msg.sphere_positions = zeros(3*num_sphere,1);
debug_visual_msg.sphere_radii = zeros(num_sphere, 1);
debug_visual_msg.sphere_colors = zeros(4*num_sphere,1);
for i=1:num_sphere
    debug_visual_msg.sphere_colors(4*(i-1)+1)= 1;
    debug_visual_msg.sphere_colors(4*(i-1)+2)= 0;
    debug_visual_msg.sphere_colors(4*(i-1)+3)= 0;
    debug_visual_msg.sphere_colors(4*(i-1)+4)= 0.7;
    debug_visual_msg.sphere_radii(i) = 0;  % 0.3 set this number when need to use 
end

%% Loading and Processing mat.file
% matfile: x y z roll pitch yaw right(Hipz Hipx Hipy Knee Ankle)7-11 left(Hipz Hipx Hipy Knee Ankle)12-16 rightArm leftArm

data = load("JointInput/unity_foot.mat");
%data = load("JointInput/example1_interpolate.mat");
%data  = load("JointInput/GRF.mat");
%data = load("JointInput/example1.mat");

t_all = length(data.unity.time);
ctrl_all = data.unity.state;
% cop   = data.unity.cop;
grf   = data.unity.U;
foot_pos = data.unity.pfoot;

grf_mag = 250; % magnititude of grf
grf_vis = 40;  % visualization scale
grf     = grf*grf_vis/grf_mag; 
sim_freq = 1000; % current default freq is 2kHz, interpolate 250Hz
t_step = 1/sim_freq;

% Note: all linear xyz follow the visualized coordinates
% rotation frame follow the left-hand rule in unity(direction cw)

for i = 1:t_all
    raw_pos = [ctrl_all(i,1), ctrl_all(i,2), ctrl_all(i,3)];
    humanoid_state_msg.body_pos = raw_pos.';
    humanoid_state_msg.jpos(:)  = ctrl_all(i,7:24);
    rotangle = ctrl_all(i, 4:6);
    rotM = rotationMatrix(-rotangle(3),'y')*rotationMatrix(-rotangle(2), 'z')*rotationMatrix(-rotangle(1),'x')*rotationMatrix(pi/2, 'y');
    humanoid_state_msg.body_ori_quat_visual(:) = rotm2quat(rotM);

%     debug_visual_msg.arrow_base_positions(1)= i/500;
%     debug_visual_msg.arrow_base_positions(2)= -0.3;
%     debug_visual_msg.arrow_base_positions(3)= 1;
%     debug_visual_msg.arrow_directions(1) = 30;
%     debug_visual_msg.arrow_directions(2) = -0.3;
%     debug_visual_msg.arrow_directions(3) = 30;

    debug_visual_msg.arrow_base_positions(1)= foot_pos(i,1);
    debug_visual_msg.arrow_base_positions(2)= foot_pos(i,2);
    debug_visual_msg.arrow_base_positions(3)= foot_pos(i,3);
    debug_visual_msg.arrow_directions(1) = grf(i,1);
    debug_visual_msg.arrow_directions(2) = grf(i,2);
    debug_visual_msg.arrow_directions(3) = grf(i,3);
    debug_visual_msg.arrow_base_positions(4)= foot_pos(i,4);
    debug_visual_msg.arrow_base_positions(5)= foot_pos(i,5);
    debug_visual_msg.arrow_base_positions(6)= foot_pos(i,6);
    debug_visual_msg.arrow_directions(4) = grf(i,7);
    debug_visual_msg.arrow_directions(5) = grf(i,8);
    debug_visual_msg.arrow_directions(6) = grf(i,9);

%     debug_visual_msg.sphere_positions(1) = i/500;
%     debug_visual_msg.sphere_positions(2) = 0;
%     debug_visual_msg.sphere_positions(3) = 0.6;

    publisher.publish('humanoid_visualization_info', humanoid_state_msg);
    publisher.publish('debug_visualization', debug_visual_msg);
    pause(t_step);
end