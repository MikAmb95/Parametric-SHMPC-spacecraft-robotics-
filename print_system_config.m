
% Characteristics of the robot---------------------------------------------
a1=2; %length of the link #1 [m]
a2=2; %length of the link #2 [m]
a3=2; %length of the link #3 [m]
ac1=a1/2; %CoM link #1 [m]
ac2=a2/2; %CoM link #2 [m]
ac3=a3/2; %CoM link #3 [m]
m1=2; %mass link #1 [kg]
m2=2; %mass link #2 [kg]
m3=2; %mass link #3 [kg]
mb=5; %mass link #1 [kg]
mp=1; %mass EE [kg]
Izz1=1/12*m1*a1^2; %moment of inertia link #1 [kg*m^2]
Izz2=1/12*m2*a2^2; %moment of inertia link #2 [kg*m^2]
Izz3=1/12*m3*a3^2; %moment of inertia link #3 [kg*m^2]
W1=0.5; % (the base is modeled as a rectangular) side #1 [m]
W2=0.5; % (the base is modeled as a rectangular) side #2 [m]
Izzb=1/12*mb*(4*W1^2+4*W2^2); % moment of inertia base [kg*m^2]
ex=W1*0-0.1*0; %where the robot is connected to the base (x-direction)
ey=W2*0-0.1*0; %where the robot is connected to the base (y-direction)


global obs_diam obs_x obs_y 

x = X; %take the X_sol of the main script to do some manipulation

%Un-pack the results
xc=x(1,:); %x_base
yc=x(2,:); %y_base
psi=x(3,:); %orient_base
q1=x(4,:); %q1 robot
q2=x(5,:); %q2 robot
q3=x(6,:); %q3 robot


%From here there this the direct kinematics of the robot to have the x-y of
%the robotic arm 

%x-y of the base
xb=xc+ex*cos(psi)+ey*sin(psi); 
yb=yc-ey*cos(psi)+ex*sin(psi); 

%x-y of the first link of the robotic arm
xe1=xc+a1*cos(psi+q1)+ex*cos(psi)+ey*sin(psi); 
ye1=yc+a1*sin(psi+q1)-ey*cos(psi)+ex*sin(psi);

%x-y of the second link of the robotic arm
xe2=xc+a1*cos(psi+q1)+ex*cos(psi)+ey*sin(psi)+a2*cos(psi+q1+q2);
ye2=yc+a1*sin(psi+q1)-ey*cos(psi)+ex*sin(psi)+a2*sin(psi+q1+q2);

%x-y of the third (end-effector) link of the robotic arm
xe3=xc+a3*cos(psi+q1+q2+q3)+a1*cos(psi+q1)+ex*cos(psi)+ey*sin(psi)+a2*cos(psi+q1+q2);
ye3=yc+a3*sin(psi+q1+q2+q3)+a1*sin(psi+q1)-ey*cos(psi)+ex*sin(psi)+a2*sin(psi+q1+q2);


figure(4)

hold on;grid on;box on
xlim([-1 4])
ylim([-1.5 4])
lW = 3; c1 = [0.68, 0.85, 0.9]; c2 = [1, 0.6, 0.2];


%build to visualize
ang=0:0.005:2*pi;
r = obs_diam/2;  % obstacle radius
xp_obs=r*cos(ang);
yp_obs=r*sin(ang);

plot(obs_x+xp_obs,obs_y+yp_obs,'--r'); % plot circle


printMotion = true; %true if you want to have the motion

for i = 1:5:size(x,2)

x11=xc(i)+W1.*cos(psi(i))-W2.*sin(psi(i));
y11=yc(i)+W1.*sin(psi(i))+W2.*sin(psi(i)+pi/2);
x21=xc(i)-W1.*cos(psi(i))+W2.*cos(psi(i)+pi/2);
y21=yc(i)-W1.*sin(psi(i))+W2.*sin(psi(i)+pi/2);
x31=xc(i)+W1.*cos(psi(i))+W2.*sin(psi(i));
y31=yc(i)+W1.*sin(psi(i))-W2.*sin(psi(i)+pi/2);
x41=xc(i)-W1.*cos(psi(i))-W2.*cos(psi(i)+pi/2);
y41=yc(i)-W1.*sin(psi(i))-W2.*sin(psi(i)+pi/2);
Line1 = line([xb(i);xe1(i)],[yb(i);ye1(i)],'color',c1,'LineWidth',lW);
Line2 = line([xe1(i);xe2(i)],[ye1(i);ye2(i)],'color',c1,'LineWidth',lW);
Line3 = line([xe2(i);xe3(i)],[ye2(i);ye3(i)],'color',c1,'LineWidth',lW);
Line4 = line([x11;x31],[y11;y31],'color',c2,'LineWidth',lW);
Line5 = line([x41;x31],[y41;y31],'color',c2,'LineWidth',lW);
Line6 = line([x21;x41],[y21;y41],'color',c2,'LineWidth',lW);
Line7 = line([x21;x11],[y21;y11],'color',c2,'LineWidth',lW);


pause(0.1)

if i == size(x,2)-1; break 
end

if printMotion == false
delete(Line1);
delete(Line2);
delete(Line3);
delete(Line4);
delete(Line5);
delete(Line6);
delete(Line7);
end



end

hold on
p1 = plot(0,0,'or',2.2,0.3,'*g',obs_x+xp_obs,obs_y+yp_obs,'--k','LineWidth',2); % plot circle

legend(p1,{'x_0','x_{des}','obstacle'})
