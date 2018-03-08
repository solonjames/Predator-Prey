function predator_prey
 close all
 g = 9.81;
 mr = 100; % Mass of predator, in kg
 my = 10.; % Mass of prey, in kg
 Frmax = 1.3*mr*g; % Max force on predator, in Newtons
 Fymax = 1.4*my*g; % Max force on prey, in Newtons
 c = 0.2; % Drag coeft, in N s/m
 initial_w = [150,1000,0,1000,0,0,0,0]; % Initial position/velocity
 force_table_predator = rand(51,2)-0.5;
 force_table_prey = rand(51,2)-0.5;
 options = odeset('Events',@event,'RelTol',0.001);
 [time_vals,sol_vals] = ode45(@(t,w)...
eom(t,w,mr,my,Frmax,Fymax,c,force_table_predator,force_table_prey), ...
 [0:1:250],initial_w,options);
 animate_projectiles(time_vals,sol_vals);

end

function F = compute_f_groupname(t,Frmax,Fymax,amiapredator,pr,vr,py,vy)
% Test time and place Enter the time and room for your test here
% Group members: list the names of your group members here
% t: Time
% Frmax: Max force that can act on the predator
% Fymax: Max force that can act on the prey
% amiapredator: Logical variable - if amiapredator is true,
% the function must compute forces acting on a predator.
% If false, code must compute forces acting on a prey.
% pr - 2D vector with current position of predator eg pr = [x_r;y_r]
% vr - 2D vector with current velocity of predator eg vr= [vx_r;vy_r]
% py - 2D vector with current position of prey py = [x_prey;y_prey]
% vy - 2D vector with current velocity of prey py = [vx_prey;vy_prey]
% NB:pr,vr,py,vy are all COLUMN VECTORS
% F - 2D vector specifying the force to be applied to the object
% that you wish to control F = [Fx;Fy]
% The direction of the force is arbitrary, but if the
% magnitude you specify exceeds the maximum allowable
% value its magnitude will be reduced to this value
% (without changing direction)
if (amiapredator)
 F=[-Frmax;0];
else
 F=[Fymax;0];
end
end

function dwdt = eom(t,w,mr,my,Frmax,Fymax,c,forcetable_r,forcetable_y)
% Extract the position and velocity variables from the vector w
% Note that this assumes the variables are stored in a particular order in w.
 pr=w(1:2); vr=w(5:6); py=w(3:4); vy=w(7:8);
 mr = 100; my = 10; g = 9.81;
% Compute all the forces on the predator
 amiapredator = true;
 Fr = compute_f_groupname(t,Frmax,Fymax,amiapredator,pr,vr,py,vy);
% The force table varies between +/- 0.5 so this makes the random force
% vary between +/- 0.2*mr*g
 Frrand = 0.4*mr*g*compute_random_force(t,forcetable_r);
 Frvisc = -vr*norm(vr)*c;
 Frgrav = -mr*g*[0;1];
 Frtotal = Fr+Frrand+Frvisc+Frgrav;
 
% Write similar code below to call your compute_f_groupname function to
% compute the force on the prey, determine the random forces on the prey,
% and determine the viscous forces on the prey
 amiapredator = false;
 Fy = compute_f_groupname(t,Frmax,Fymax,amiapredator,pr,vr,py,vy);
 Fyrand = 0.4*my*g*compute_random_force(t,forcetable_y);
 Fyvisc = -vy*norm(vy)*c;
 Fygrav = -my*g*[0;1];
 Fytotal = Fy+Fyrand+Fyvisc+Fygrav;
 
 ar=Frtotal/mr;
 ay=Fytotal/my;
 
 dwdt = [vr(1); vr(2); vy(1); vy(2); ar(1); ar(2); ay(1); ay(2)];
end

function [event,stop,direction] = event(t,w)
% Event function to stop calculation when predator catches prey
% Write your code here… For the event variable, use the distance between
% predator and prey. You could add other events to detect when predator/prey leave
% the competition area as well. See the MATLAB manual for how to detect and
% distinguish between multiple events if you want to do this

%stopping if Predator Catches Prey

pr=w(1:2);
py=w(3:4);

%checks first if either predator or prey have hit the ground, then moves 
%on to check distance between objects
if(pr(2)<0)
    event=0;
elseif py(2)<0
    event=0;
else
    %computes distance - 1
    event=sqrt((pr(1) - py(1))^2 + (pr(2) - py(2))^2) - 1;
end

stop = 1;
direction=0;

end 

function animate_projectiles(t,sols)
figure
xmax = max(max(sols(:,3)),max(sols(:,1)));
xmin = min(min(sols(:,3)),min(sols(:,1)));
ymax = max(max(sols(:,4)),max(sols(:,2)));
ymin = min(min(sols(:,4)),min(sols(:,2)));
dx = 0.1*(xmax-xmin)+0.5;
dy = 0.1*(ymax-ymin)+0.5;
for i = 1:length(t)
 clf
 plot(sols(1:i,3),sols(1:i,4),'LineWidth',2,'LineStyle',...
 ':','Color',[0 0 1]);
 ylim([ymin-dy ymax+dy]);
 xlim([xmin-dx xmax+dx]);
 hold on
 plot(sols(1:i,1),sols(1:i,2),'LineWidth',2,'LineStyle',':',...
 'Color',[1 0 0]);
 plot(sols(i,1),sols(i,2),'ro','MarkerSize',11,'MarkerFaceColor','r');
 plot(sols(i,3),sols(i,4),'ro','MarkerSize',5,'MarkerFaceColor','g');
 pause(0.1);
end
end

function F = compute_random_force(t,force_table)
% Computes value of fluctuating random force at time t, where 0<t<250.
% The variable force_table is a 251x2 matrix of pseudo-random
% numbers between -0.5 and 0.5, computed using
% force_table = rand(51,2)-0.5;
% NB – THE FORCE TABLE MUST BE DEFINED OUTSIDE THIS FUNCTION
% If you define it in here it fries the ode45 function
F = [interp1(0:5:250,force_table(:,1),t);...
 interp1(0:5:250,force_table(:,2),t)];
end