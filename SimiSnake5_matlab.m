clear all
close all
clc
vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5); %connenet vrep at this port

if (clientID>-1)
    disp('Connected to remote API server');
    %create some joint pos
    Joint_pos1 = [-2*pi/3,0,0,pi/4,0,0,0,0];
    Joint_pos2 = [2*pi/3,pi,pi/8,pi/4,pi/16,pi,0,0];
    Joint_pos3 = [0,0,0,0,0,0,0,0];
    Joint_poses(:,:,:,:,:,:,:,:,1) = Joint_pos1;
    Joint_poses(:,:,:,:,:,:,:,:,2) = Joint_pos2;
    Joint_poses(:,:,:,:,:,:,:,:,3) = Joint_pos3;
    
%     disp(Joint_pos(:,1));
%get the list of joint poses from simulink
[rows columns] = size([Joint_poses]);

    %joints handles
    h=[0,0,0,0,0,0,0,0];
    % vrep api to command the specific joint of the snake model in vrep
    % joint name should be match the model
    [r,h(1)]=vrep.simxGetObjectHandle(clientID,'snake_joint_v1',vrep.simx_opmode_blocking);
    [r,h(2)]=vrep.simxGetObjectHandle(clientID,'snake_joint_h1',vrep.simx_opmode_blocking);
    [r,h(3)]=vrep.simxGetObjectHandle(clientID,'snake_joint_v2',vrep.simx_opmode_blocking);
    [r,h(4)]=vrep.simxGetObjectHandle(clientID,'snake_joint_h2',vrep.simx_opmode_blocking);
    [r,h(5)]=vrep.simxGetObjectHandle(clientID,'snake_joint_v3',vrep.simx_opmode_blocking);
    [r,h(6)]=vrep.simxGetObjectHandle(clientID,'snake_joint_h3',vrep.simx_opmode_blocking);
    [r,h(7)]=vrep.simxGetObjectHandle(clientID,'snake_joint_v4',vrep.simx_opmode_blocking);
    [r,h(8)]=vrep.simxGetObjectHandle(clientID,'snake_joint_h4',vrep.simx_opmode_blocking);
%     op = vrep.simGetJointForce(h(1), vrep.simxGetJointForce(h(1)));
    

    % loop through the poses from simulink model
    for j=1:columns
        %joint pose of each joint
        joint_pos = Joint_poses(:,j);

        %loop through all the 8 joints
        for i=1:8
%             op = vrep.simGetJointForce(h(i), vrep.simxGetJointForce(h(i)));
            %command each joint to a trarget pose in vrep using its api
            vrep.simxSetJointTargetPosition(clientID, h(i),joint_pos(i),vrep.simx_opmode_streaming);
        end
        pause(10);
    end
    
else
    disp('Failed connecting to remote API server');
end
    vrep.delete(); % call the destructor!
    
    disp('Program ended');