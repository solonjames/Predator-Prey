function predator_prey
 close all
 clear all
 clc
 g = 9.81;
 mr = 100; % Mass of predator, in kg
 my = 10.; % Mass of prey, in kg
 Frmax = 1.3*mr*g; % Max force on predator, in Newtons
 Fymax = 1.4*my*g; % Max force on prey, in Newtons
 c = 0.2; % Drag coeft, in N s/m
 initial_w = [0,1000,150,1000,0,0,0,0]; % Initial position/velocity
 force_table_predator = rand(51,2)-0.5;
 force_table_prey = rand(51,2)-0.5;
 options = odeset('Events',@event,'RelTol',0.001);
 % StartTime;
 StartTime = clock; 
 [time_vals,sol_vals] = ode113(@(t,w)...
eom(t,w,mr,my,Frmax,Fymax,c,force_table_predator,force_table_prey, StartTime), ...
 [0:1:250],initial_w,options);
 animate_projectiles(time_vals,sol_vals);
 time=max(time_vals)
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

    % Redefining the variables because your variables are illogical.
    v_hunter = vr;
    p_hunter = pr;
    v_prey = vy;
    p_prey = py;
    
    % Defining helpful constants.
    deg2rad = pi / 180;
    rad2deg = 180 / pi;
    m_prey = 10;
    m_hunter = 100;
    g = 9.81;
    
    % Defining variables needed for control.
    % PREY SETTINGS
    % 0 = near-invincible prey
    % 1 = dive-bomber
    % 2 = sinusoidal
    % 3 = random sinusoidal
    % 4 = up
    % PREDATOR SETTINGS
    % 0 = black magic
    % 1 = basic predator
    preySetting = 4;
    predatorSetting = 4;
    
    
    persistent preyMode;
    persistent predatorMode;
    predatorDiveHeight = 20;
    direction = 0;
    relativeSpeedMaxRatio = 1.2;
    preyCruisingAltitude = 75;
    preyPanicLevel = preyCruisingAltitude/2;
    persistent oldTime
    if (amiapredator)
        % Predator code.
        switch predatorSetting
            case 0
                % David's black magic predator.
                    disp(['Time step: ', num2str(t - oldTime)]);
                    if t == 0
                        oldTime = 0;
                    else
                        oldTime = t; 
                    end
                
                    % CONSTANTS
                    dt_delta = t - oldTime; % Works: 0.5 or 0.125
                
                
                    delta = p_prey + dt_delta * v_prey - p_hunter;
                    direction = atan2(delta(2), delta(1));
                    F_desired = getForce(Frmax, direction) + [0; 100 * 9.81] - v_hunter * 30; %magic constant is 30
                    if norm(F_desired) > Frmax
                        disp(['Statistics: Frmax = ', num2str(Frmax), ' F_temp = ', num2str(norm(F_desired)), ' v = ', num2str(norm(v_hunter))]);
                        F = F_desired / norm(F_desired) * Frmax;
                    else
                        F = F_desired;
                    end
                    if p_hunter(2) < 100
                        F = [0; Frmax];
                    elseif(norm(p_hunter-p_prey) < 5)
                        difference = p_prey-p_hunter;
                        direction = atan2(difference(2),difference(1));
                        F=getForce(Frmax,direction);
                    end
            case 1
                % Basic predator code.
                delta = p_prey + 0.5 * v_prey - p_hunter;
                direction = atan2(delta(2), delta(1));
                F = getForce(Frmax, direction);
            case 2
                % Orbital ATK.
                F_gravity = g * m_hunter;
                F_allowance = Frmax - F_gravity;
                delta = p_prey - p_hunter;
                distance = norm(delta);
                direction = atan2(delta(2), delta(1));
                if distance < 100
                    relative_v = dot(v_hunter, v_prey) / dot(v_prey, v_prey) * v_prey;
                    if norm(relative_v) > 2
                        direction = atan2(-relative_v(2), -relative_v(1));
                    end
                    F = getForce(F_allowance, direction) + [0; F_gravity];
                else
                    F = getForce(F_allowance, direction) + [0; F_gravity];
                end
            case 3
                % Bower bot
                F_gravity = g * m_hunter;
                F_allowance = Frmax - F_gravity;
                v_max = 8.29;
                delta = p_prey - p_hunter - v_hunter / 8.28;
                direction = atan2(delta(2), delta(1));
                F = getForce(F_allowance, direction) + [0; F_gravity];
            case 4
                % Improved magical bot.
                % Constants.
                dt = 0.15;
                magicConstant = 30;
                
                % Calculates predicted direction to future target.
                predictedDelta = p_prey + dt * v_prey - p_hunter;
                direction = atan2(predictedDelta(2), predictedDelta(1));
                
                % Calculates desired force.
                F_desired = getForce(Frmax, direction) + [0; 100 * 9.81] - v_hunter * magicConstant;
                
                % If desired force exceeds allowed limit, it's normalized and multiplied by Frmax.
                if norm(F_desired) > Frmax
                    F = F_desired / norm(F_desired) * Frmax;
                else
                    F = F_desired;
                end
                
                % Prevents the predator from hitting the ground.
                if p_hunter(2) < 200
                    F = [0; Frmax];
                end
        end
    else
        % Prey code.
        switch preySetting
            case 0
                % Calculates whether the prey is to the left or right of
                % the predator's velocity vector.
                a = p_hunter;
                b = p_hunter + v_hunter;
                c = p_prey;
                orientation = (c(1) - a(1)) * (b(2) - a(2)) - (c(2) - a(2)) * (b(1) - a(1));
                
                % Moves normally away from the predator's velocity vector.
                if orientation < 0
                    direction = atan2(v_hunter(2), v_hunter(1)) + 90 * deg2rad;
                else
                    direction = atan2(v_hunter(2), v_hunter(1)) - 90 * deg2rad;
                end
                
                % If far away from the predator or close to the ground,
                % moves up.
                if norm(p_prey - p_hunter) > 100 || p_prey(2) < 150
                    direction = 90 * deg2rad;
                end
                F = getForce(Fymax, direction);
            case 1
                if t == 0
                    preyMode = 'horizontal_escape';
                end

                switch(preyMode)
                    case 'horizontal_escape'
                        if (v_prey(1) >= v_hunter(1) && v_prey(1) >= 0) || (v_prey(1) <= v_hunter(1) && v_prey(1) <= 0)
                            % If prey is currently outrunning predator, keep doing so.
                            if v_prey(1) >= 0
                                direction = 0 * deg2rad;
                            else
                                direction = 180 * deg2rad;
                            end
                        else
                            % If prey cannot outrun predator, switch to dive mode.
                            direction = -90 * deg2rad;
                            preyMode = 'dive';
                            disp(['Switch to dive @ t = ', num2str(t)]);
                        end
                        % Makes sure to end dive if predator isn't following and prey is about to hit the ground.
                        if checkArcStart()
                            direction = 90 * deg2rad;
                        end
                    case 'dive'
                        direction = -90 * deg2rad + 5*cos(t);
                        if checkArcStart()
                            direction = 90 * deg2rad;
                        end
                    case 'end_dive'
                        direction = 43.00 * deg2rad;

                        %runs if turning is done
                        if v_prey(2) > 0 
                            preyMode = 'level flight';
                            disp(['Switch to level flight @ t = ', num2str(t)]);
                        end

                    case 'level flight'

                        direction = acos(m_prey*g/Fymax);

                        if p_prey(2) < preyPanicLevel

                            direction = acos(m_prey*g/Fymax) + sqrt(preyPanicLevel - p_prey(2))/preyPanicLevel...
                                * (pi/2 - acos(m_prey*g/Fymax));

                        end

                        if p_prey(2) > 1.5*preyCruisingAltitude

                            preyMode = 'dive';
                            disp(['Switch to dive @ t = ', num2str(t)]);

                        end

                end
                F = getForce(Fymax, direction);
            case 2
                % Sinusoidal prey.
                F_gravity = g * m_prey;
                F_allowance = Fymax - F_gravity;
                direction = t * 10 * deg2rad;
                F = getForce(F_allowance, direction) + [0; F_gravity];
            case 3
                % Sinusoidal prey 2.
                F_gravity = g * m_prey;
                F_allowance = Fymax - F_gravity;
                direction = t * 10 * deg2rad + sin(t) * 5;
                F = getForce(F_allowance, direction) + [0; F_gravity];
            case 4
                % UP prey
                F = [-Fymax / sqrt(2); Fymax / sqrt(2)];
        end
    end
    
    function isArcStarting = checkArcStart()
        safetyFactor = 0.5;
        if Fymax * (1 - safetyFactor) * (p_prey(2) - preyCruisingAltitude) < 0.5 * m_prey * v_prey(2)^2
            disp(['Switch to end_dive @ t = ', num2str(t)]);
            preyMode = 'end_dive';
            isArcStarting = true;
        else
            isArcStarting = false;
        end
    end

    function radius = radiusOfCurvature()
        if (v_hunter(1) * v_prey(2) - v_hunter(2) * v_prey(1)) ~=0
            radius = 100;
        else
            radius = norm(v_prey) * abs( ( (p_prey(2)-p_hunter(2)) * v_hunter(1) -...
                (p_prey(1)-p_hunter(1)) * v_hunter(2))...
                / (v_hunter(1) * v_prey(2) - v_hunter(2) * v_prey(1)));
        end
    end
    
    % Calculates force based on magnitude and direction.
    function force = getForce(magnitude, direction)
        force = [magnitude * cos(direction); magnitude * sin(direction)];
    end
end

function dwdt = eom(t,w,mr,my,Frmax,Fymax,c,forcetable_r,forcetable_y,StartTime)
 TimeElapsed = clock - StartTime;
 if TimeElapsed(end)>60 %Set it to a value that you want (I chose 10 seconds)
     return
 end

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
 
 delta = pr - py;
 distance = sqrt(delta(1)^2 + delta(2)^2);
 
 dwdt = [vr(1); vr(2); vy(1); vy(2); ar(1); ar(2); ay(1); ay(2)];
end

function [event0,stop,direction] = event(t,w)
% Event function to stop calculation when predator catches prey
% Write your code here� For the event variable, use the distance between
% predator and prey. You could add other events to detect when predator/prey leave
% the competition area as well. See the MATLAB manual for how to detect and
% distinguish between multiple events if you want to do this

%stopping if Predator Catches Prey

pr=w(1:2);
py=w(3:4);

%checks first if either predator or prey have hit the ground, then moves 
%on to check distance between objects
if(pr(2)<0)
    event0=0;
elseif py(2)<0
    event0=0;
else
    %computes distance - 1
    event0=sqrt((pr(1) - py(1))^2 + (pr(2) - py(2))^2) - 1;
end

stop = 1;
direction=0;

end 

function animate_projectiles(t,sols)
    handle = figure;
    xmax = max(max(sols(:,3)),max(sols(:,1)));
    xmin = min(min(sols(:,3)),min(sols(:,1)));
    ymax = max(max(sols(:,4)),max(sols(:,2)));
    ymin = min(min(sols(:,4)),min(sols(:,2)));
    dx = 0.1*(xmax-xmin)+0.5;
    dy = 0.1*(ymax-ymin)+0.5;
    i = 1;
    while(i <= length(t))
        if ~ishandle(handle)
            disp('Closed by user.');
            return
        end
        clf
        plot(sols(1:i,3),sols(1:i,4),'LineWidth',2,'LineStyle',...
        ':','Color',[0 0 1]);
        ylim([ymin-dy ymax+dy]);
        xlim([xmin-dx xmax+dx]);
        daspect([1 1 1]);
        hold on
        plot(sols(1:i,1),sols(1:i,2),'LineWidth',2,'LineStyle',':',...
        'Color',[1 0 0]);
        plot(sols(i,1),sols(i,2),'ro','MarkerSize',11,'MarkerFaceColor','r');
        plot(sols(i,3),sols(i,4),'ro','MarkerSize',5,'MarkerFaceColor','g');
        title(['t = ', num2str(i)]);
        if i == length(t)
            i = 1;
        else
            i = i + 1;
        end
        
                
%       Draws arrows to visualize velocity on both entities.
%       Draws a force vector on the entity under control.
        hunter = [sols(i,1) sols(i,2)];
        prey = [sols(i,3) sols(i,4)];
        quiver(hunter(1),hunter(2),sols(i,5),sols(i,6),20)
        quiver(prey(1),prey(2),sols(i,7),sols(i,8),20)
        
        pause(1/60);
    end
end

function F = compute_random_force(t,force_table)
% Computes value of fluctuating random force at time t, where 0<t<250.
% The variable force_table is a 251x2 matrix of pseudo-random
% numbers between -0.5 and 0.5, computed using
% force_table = rand(51,2)-0.5;
% NB � THE FORCE TABLE MUST BE DEFINED OUTSIDE THIS FUNCTION
% If you define it in here it fries the ode45 function
F = [interp1(0:5:250,force_table(:,1),t);...
 interp1(0:5:250,force_table(:,2),t)];
end