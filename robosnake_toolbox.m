
robot = robotics.RigidBodyTree;
L = .25;                  %link length
t1= sym('t1');
t2= sym('t2');
t3= sym('t3');
t4= sym('t4');
t5= sym('t5');
t6= sym('t6');
t7= sym('t7');
t8= sym('t8');
t9= sym('t9');
t10= sym('t10');
t11= sym('t11');
t12= sym('t12');
t13= sym('t13');

dhparams = [ 0 0 L 0;
            0 -pi/2 0 0;
           0 pi/2 0 0;
           0 -pi/2 L 0;
           0 pi/2 0 0;
           0 -pi/2 L 0;
           0 pi/2 0 0;
           0 -pi/2 L 0;
           0 pi/2 0 0;
           0 -pi/2 L 0;
           0 pi/2 0 0;
           0 -pi/2 L 0;
           0 pi/2 0 0;
           0 0 L 0;]

body1 = robotics.RigidBody( 'body1' ) ;
jnt1 = robotics.Joint('jnt1','revolute');
setFixedTransform(jnt1,dhparams(1,:),'dh');
body1.Joint=jnt1;
addBody(robot,body1,'base')

body2 =robotics.RigidBody('body2');
jnt2 =robotics.Joint('jnt2','revolute');
body3 = robotics.RigidBody('body3');
jnt3 = robotics.Joint('jnt3','revolute');
body4 = robotics.RigidBody('body4');
jnt4 = robotics.Joint('jnt4','revolute');
body5 = robotics.RigidBody('body5');
jnt5 = robotics.Joint('jnt5','revolute');
body6 = robotics.RigidBody('body6');
jnt6 = robotics.Joint('jnt6','revolute');
body7 = robotics.RigidBody('body7');
jnt7 = robotics.Joint('jnt7','revolute');
body8 =robotics.RigidBody('body8');
jnt8 =robotics.Joint('jnt8','revolute');
body9 = robotics.RigidBody('body9');
jnt9 = robotics.Joint('jnt9','revolute');
body10 = robotics.RigidBody('body10');
jnt10 = robotics.Joint('jnt10','revolute');
body11 = robotics.RigidBody('body11');
jnt11 = robotics.Joint('jnt11','revolute');
body12 = robotics.RigidBody('body12');
jnt12 = robotics.Joint('jnt12','revolute');
body13 = robotics.RigidBody('body13');
jnt13 = robotics.Joint('jnt13','revolute');
body14 = robotics.RigidBody('body14');
jnt14 = robotics.Joint('jnt14','revolute');


setFixedTransform(jnt2,dhparams(2,:),'dh');
setFixedTransform(jnt3,dhparams(3,:),'dh');
setFixedTransform(jnt4,dhparams(4,:),'dh');
setFixedTransform(jnt5,dhparams(5,:),'dh');
setFixedTransform(jnt6,dhparams(6,:),'dh');
setFixedTransform(jnt7,dhparams(7,:),'dh');
setFixedTransform(jnt8,dhparams(8,:),'dh');
setFixedTransform(jnt9,dhparams(9,:),'dh');
setFixedTransform(jnt10,dhparams(10,:),'dh');
setFixedTransform(jnt11,dhparams(11,:),'dh');
setFixedTransform(jnt12,dhparams(12,:),'dh');
setFixedTransform(jnt13,dhparams(13,:),'dh');
setFixedTransform(jnt14,dhparams(14,:),'dh');

body2.Joint=jnt2;
body3.Joint=jnt3;
body4.Joint=jnt4;
body5.Joint=jnt5;
body6.Joint=jnt6;
body7.Joint=jnt7;
body8.Joint=jnt8;
body9.Joint=jnt9;
body10.Joint=jnt10;
body11.Joint=jnt11;
body12.Joint=jnt12;
body13.Joint=jnt13;
body14.Joint=jnt14;
 
addBody(robot,body2,'body1');
addBody(robot,body3,'body2');
addBody(robot,body4,'body3');
addBody(robot,body5,'body4');
addBody(robot,body6,'body5');
addBody(robot,body7,'body6');
addBody(robot,body8,'body7');
addBody(robot,body9,'body8');
addBody(robot,body10,'body9');
addBody(robot,body11,'body10');
addBody(robot,body12,'body11');
addBody(robot,body13,'body12');
addBody(robot,body14,'body13');

%------------------------------------------
 config = homeConfiguration(robot)
 
 config(1).JointPosition = 0;
 config(2).JointPosition = 0;
 config(3).JointPosition = 0;
 config(4).JointPosition = 0;
 config(5).JointPosition = 0;
 config(6).JointPosition = 0;
 config(7).JointPosition = 0;
 config(8).JointPosition = 0;
 config(9).JointPosition = 0;
 config(10).JointPosition = 0;
 config(11).JointPosition = 0;
 config(12).JointPosition = 0;
 config(13).JointPosition = 0;
 config(14).JointPosition = 0;
 
% figure
  show(robot,config);

  
  
  
  
  

  
  
  