rx=20;
ry=20;
rz=5.0;
omega=0.2;
t=0:0.1:500;
t=t';
x=rx*cos(omega*t);
y=ry*sin(omega*t);
z=rz*sin(2*omega*t);
r=[x y z];
qx=2*sin(0.01*t);
qy=0.1*t;
qz=0+.1*t;
q=[qx qy qz];
pd=r+q;
plot3(pd(:,1),pd(:,2),pd(:,3));
hold on
plot3(qx,qy,qz);



plot3(pd1(:,1),pd1(:,2),pd1(:,3));
hold on;
plot3(Target_Pos(:,1),Target_Pos(:,2),Target_Pos(:,3));
plot3(Tracker.Id1.State(:,1),Tracker.Id1.State(:,2),Tracker.Id1.State(:,3));
plot3(Target.Id1.Est1(:,1),Target.Id1.Est1(:,2),Target.Id1.Est1(:,3));

e1=pd1-Tracker.Id1.State(:,1:3);
e2=Target_Pos-Target.Id1.Est1(:,1:3);