function x=testPublisher( u )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
global  jointpub jointmsg counter tftree tfStampedMsg tfStampedMsg2 tfStampedMsg3 tfStampedMsg4 tfStampedMsgcm1 tfStampedMsgcm2 tfStampedMsgcm3 tfStampedMsgcm4

if u==0 

    %% TF publisher
    tftree = rostf;
    tfStampedMsg = rosmessage('geometry_msgs/TransformStamped');
    tfStampedMsg.ChildFrameId = 'TF1';
    tfStampedMsg.Header.FrameId = 'world';
    
    tfStampedMsg2 = rosmessage('geometry_msgs/TransformStamped');
    tfStampedMsg2.ChildFrameId = 'TF2';
    tfStampedMsg2.Header.FrameId = 'world';
    
    tfStampedMsg3 = rosmessage('geometry_msgs/TransformStamped');
    tfStampedMsg3.ChildFrameId = 'TF3';
    tfStampedMsg3.Header.FrameId = 'world';
    
    tfStampedMsg4 = rosmessage('geometry_msgs/TransformStamped');
    tfStampedMsg4.ChildFrameId = 'TF4';
    tfStampedMsg4.Header.FrameId = 'world';
    
    tfStampedMsgcm1 = rosmessage('geometry_msgs/TransformStamped');
    tfStampedMsgcm1.ChildFrameId = 'TFcm1';
    tfStampedMsgcm1.Header.FrameId = 'world';
    
    tfStampedMsgcm2 = rosmessage('geometry_msgs/TransformStamped');
    tfStampedMsgcm2.ChildFrameId = 'TFcm2';
    tfStampedMsgcm2.Header.FrameId = 'world';
    
    tfStampedMsgcm3 = rosmessage('geometry_msgs/TransformStamped');
    tfStampedMsgcm3.ChildFrameId = 'TFcm3';
    tfStampedMsgcm3.Header.FrameId = 'world';
    
    tfStampedMsgcm4 = rosmessage('geometry_msgs/TransformStamped');
    tfStampedMsgcm4.ChildFrameId = 'TFcm4';
    tfStampedMsgcm4.Header.FrameId = 'world';
    
    %% Joint State Publisher
    %Use here the correct topic name --see bringup launch file--
    jointpub = rospublisher('/fourdof_joint_states', 'sensor_msgs/JointState');
    jointmsg = rosmessage(jointpub);
    
    % specific names of the joints --see urdf file--
    jointmsg.Name={ 'q1_joint','q2_joint','q3_joint','q4_joint'};
    
    for i=1:4
    jointmsg.Velocity(i)=0.0;
    jointmsg.Effort(i)=0.0;
    end
    
    counter=0;
    
end
%% JOINT STATE MSG
    T1=1.0;
    T2=1.0;
    T3=1.0;
    T4=1.0;
    w1=2*pi/T1;
    w2=2*pi/T2;
    w3=2*pi/T3;
    w4=2*pi/T4;

    jointmsg.Header.Stamp=tfStampedMsg.Header.Stamp;
    jointmsg.Header.Seq=counter;
    counter=counter+1;
    
    %Use the joint limits of the specific robot --see urdf file--
    q1=pi*sin(w1*u);
    q2=pi*sin(w2*u)*0.05;
    q3=pi*sin(w3*u)*0.05;
    q4=pi*sin(w4*u);
    jointmsg.Position=[q1,q2,q3,q4];
%     showdetails(jointmsg);
    send(jointpub,jointmsg);
    
%% TF MSG
    [T1_w,T2_w,T3_w,T4_w,Tcm1_w,Tcm2_w,Tcm3_w,Tcm4_w]=SimpleRobotPlot([0 0 0 0 q1 q2 q3 -q4 u 0.3 0.34 0.583 0.12 0.08 0.115 0.082 0.108 0.123 0.167 0.028 0.05 pi/4 pi/6 0.32]);
    tfStampedMsg.Header.Stamp = rostime('now');
    tfStampedMsg.Header.Seq=counter;
    tfStampedMsg.Transform.Translation.X = double(T1_w(1,4));
    tfStampedMsg.Transform.Translation.Y = double(T1_w(2,4));
    tfStampedMsg.Transform.Translation.Z = double(T1_w(3,4));

    rotm = double(T1_w(1:3,1:3));
    quatrot = rotm2quat(rotm);%四元数表示一个旋转，
    tfStampedMsg.Transform.Rotation.W = quatrot(1);
    tfStampedMsg.Transform.Rotation.X = quatrot(2);
    tfStampedMsg.Transform.Rotation.Y = quatrot(3);
    tfStampedMsg.Transform.Rotation.Z = quatrot(4);
    sendTransform(tftree, tfStampedMsg);
    %************************************************************
    
    tfStampedMsg2.Header.Stamp = rostime('now');
    tfStampedMsg2.Header.Seq=counter;
    tfStampedMsg2.Transform.Translation.X = double(T2_w(1,4));
    tfStampedMsg2.Transform.Translation.Y = double(T2_w(2,4));
    tfStampedMsg2.Transform.Translation.Z = double(T2_w(3,4));

    rotm = double(T2_w(1:3,1:3));
    quatrot = rotm2quat(rotm);%四元数表示一个旋转，
    tfStampedMsg2.Transform.Rotation.W = quatrot(1);
    tfStampedMsg2.Transform.Rotation.X = quatrot(2);
    tfStampedMsg2.Transform.Rotation.Y = quatrot(3);
    tfStampedMsg2.Transform.Rotation.Z = quatrot(4);
    sendTransform(tftree, tfStampedMsg2);
    %************************************************************
    
    tfStampedMsg3.Header.Stamp = rostime('now');
    tfStampedMsg3.Header.Seq=counter;
    tfStampedMsg3.Transform.Translation.X = double(T3_w(1,4));
    tfStampedMsg3.Transform.Translation.Y = double(T3_w(2,4));
    tfStampedMsg3.Transform.Translation.Z = double(T3_w(3,4));

    rotm = double(T3_w(1:3,1:3));
    quatrot = rotm2quat(rotm);%四元数表示一个旋转，
    tfStampedMsg3.Transform.Rotation.W = quatrot(1);
    tfStampedMsg3.Transform.Rotation.X = quatrot(2);
    tfStampedMsg3.Transform.Rotation.Y = quatrot(3);
    tfStampedMsg3.Transform.Rotation.Z = quatrot(4);
    sendTransform(tftree, tfStampedMsg3);
    %************************************************************
       
    tfStampedMsgcm1.Header.Stamp = rostime('now');
    tfStampedMsgcm1.Header.Seq=counter;
    tfStampedMsgcm1.Transform.Translation.X = double(Tcm1_w(1,4));
    tfStampedMsgcm1.Transform.Translation.Y = double(Tcm1_w(2,4));
    tfStampedMsgcm1.Transform.Translation.Z = double(Tcm1_w(3,4));

    rotm = double(Tcm1_w(1:3,1:3));
    quatrot = rotm2quat(rotm);%四元数表示一个旋转，
    tfStampedMsgcm1.Transform.Rotation.W = quatrot(1);
    tfStampedMsgcm1.Transform.Rotation.X = quatrot(2);
    tfStampedMsgcm1.Transform.Rotation.Y = quatrot(3);
    tfStampedMsgcm1.Transform.Rotation.Z = quatrot(4);
    sendTransform(tftree, tfStampedMsgcm1);
    %************************************************************
    
    tfStampedMsgcm2.Header.Stamp = rostime('now');
    tfStampedMsgcm2.Header.Seq=counter;
    tfStampedMsgcm2.Transform.Translation.X = double(Tcm2_w(1,4));
    tfStampedMsgcm2.Transform.Translation.Y = double(Tcm2_w(2,4));
    tfStampedMsgcm2.Transform.Translation.Z = double(Tcm2_w(3,4));

    rotm = double(Tcm2_w(1:3,1:3));
    quatrot = rotm2quat(rotm);%四元数表示一个旋转，
    tfStampedMsgcm2.Transform.Rotation.W = quatrot(1);
    tfStampedMsgcm2.Transform.Rotation.X = quatrot(2);
    tfStampedMsgcm2.Transform.Rotation.Y = quatrot(3);
    tfStampedMsgcm2.Transform.Rotation.Z = quatrot(4);
    sendTransform(tftree, tfStampedMsgcm2);
    %************************************************************
    
    tfStampedMsgcm3.Header.Stamp = rostime('now');
    tfStampedMsgcm3.Header.Seq=counter;
    tfStampedMsgcm3.Transform.Translation.X = double(Tcm3_w(1,4));
    tfStampedMsgcm3.Transform.Translation.Y = double(Tcm3_w(2,4));
    tfStampedMsgcm3.Transform.Translation.Z = double(Tcm3_w(3,4));

    rotm = double(Tcm3_w(1:3,1:3));
    quatrot = rotm2quat(rotm);%四元数表示一个旋转，
    tfStampedMsgcm3.Transform.Rotation.W = quatrot(1);
    tfStampedMsgcm3.Transform.Rotation.X = quatrot(2);
    tfStampedMsgcm3.Transform.Rotation.Y = quatrot(3);
    tfStampedMsgcm3.Transform.Rotation.Z = quatrot(4);
    sendTransform(tftree, tfStampedMsgcm3);
    %************************************************************
    
     tfStampedMsgcm4.Header.Stamp = rostime('now');
    tfStampedMsgcm4.Header.Seq=counter;
    tfStampedMsgcm4.Transform.Translation.X = double(Tcm4_w(1,4));
    tfStampedMsgcm4.Transform.Translation.Y = double(Tcm4_w(2,4));
    tfStampedMsgcm4.Transform.Translation.Z = double(Tcm4_w(3,4));

    rotm = double(Tcm4_w(1:3,1:3));
    quatrot = rotm2quat(rotm);%四元数表示一个旋转，
    tfStampedMsgcm4.Transform.Rotation.W = quatrot(1);
    tfStampedMsgcm4.Transform.Rotation.X = quatrot(2);
    tfStampedMsgcm4.Transform.Rotation.Y = quatrot(3);
    tfStampedMsgcm4.Transform.Rotation.Z = quatrot(4);
    sendTransform(tftree, tfStampedMsgcm4);    
    %************************************************************
    
    %Use the above example to plot the TF of each joint and each center 
    %of mass using the Homogenous transformations obtained from 
    %D-H parameters. You will need the joint positions which are generated
    %with the JOINT STATE MSG publisher (see above)

    x=u;

end

