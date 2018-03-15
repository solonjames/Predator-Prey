function F = compute_f_group21(t,Frmax,Fymax,amiapredator,pr,vr,py,vy)
    % Test time and place: Friday, March 16, 2018 @ 10:00 AM in Room 92
    % Group members: David Charatan, Sarah Branse, Peter Huson, Solon James
    
    % Giving the variables more logical names.
    v_hunter = vr;
    p_hunter = pr;
    v_prey = vy;
    p_prey = py;
    
    % Defining helpful constants.
    deg2rad = pi / 180;
    m_prey = 10;
    m_hunter = 100;
    g = 9.81;
    
    % Defining variables needed for control.
    preySetting = 0;
    predatorSetting = 0;
    
    F = [0; 0];
    if (amiapredator)
        % Predator code.
        switch predatorSetting
            case 0
                % Magical predator bot (primary).
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
                minHeight = 100;
                if -v_hunter(2)^2 / (2 * Frmax / m_hunter - g) + p_hunter(2) < minHeight
                    F = [0; Frmax];
                end
                
                % BACKUP: If the other team makes our bot crash into the
                % ground, we can change this to be 100% sure we don't crash
                % into the ground.
                %if p_hunter(2) < 200
                %    F = [0; Frmax];
                %end
            case 1
                % BACKUP: Basic predator code.
                % This bot moves directly toward the prey.
                delta = p_prey + 0.5 * v_prey - p_hunter;
                direction = atan2(delta(2), delta(1));
                F = getForce(Frmax, direction);
            case 2
                % BACKUP: Improved basic predator code.
                % This bot first subtracts gravity, then uses the remaining
                % force to move toward the prey.
                F_gravity = g * m_hunter;
                F_allowance = Frmax - F_gravity;
                delta = p_prey - p_hunter;
                direction = atan2(delta(2), delta(1));
                F = getForce(F_allowance, direction) + [0; F_gravity];
        end
    else
        % Prey code.
        switch preySetting
            case 0
                % Magical prey bot (primary).
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
                % BACKUP: Sinusoidal prey.
                F_gravity = g * m_prey;
                F_allowance = Fymax - F_gravity;
                direction = t * 10 * deg2rad;
                F = getForce(F_allowance, direction) + [0; F_gravity];
            case 2
                % BACKUP: 2-layer sinusoidal prey.
                F_gravity = g * m_prey;
                F_allowance = Fymax - F_gravity;
                direction = t * 10 * deg2rad + sin(t) * 5;
                F = getForce(F_allowance, direction) + [0; F_gravity];
            case 3
                % BACKUP: Orbital/sinusoidal randomness prey.
                delta = [500; 1000] - p_prey;
                direction = atan2(delta(2), delta(1)) + sin(t);
                F = getForce(Fymax, direction);
            case 4
                % BACKUP: Normal prey.
                direction = atan2(v_hunter(2), v_hunter(1)) - 90 * deg2rad;
                F = getForce(Fymax, direction);
                if p_prey(2) < 100
                    F = [0; Fymax];
                end
        end
    end
    
    % Calculates force based on magnitude and direction.
    function force = getForce(magnitude, direction)
        force = [magnitude * cos(direction); magnitude * sin(direction)];
    end
end