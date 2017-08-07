% ee_work_envelope:
%   Used to calculate the work envelope of an articulated robot arm
%   based on the number of points reachable (unique, and total), with
%   the points rounded to a 3D grid (nearest 5cm) for consistency.
%   It is assumed that the link lengths are all equal (= reach / DOF)
%   DOF:
%     Number of degrees of freedom / joints
%   reach:
%     Total reach of arm, in cm
%   angleIncrement:
%     The value to increment the angles by while
%     iterating. Larger values will result in skipped points on
%     the grid, while smaller values will require more computation time.
%   maxAngle:
%     The maximum angle that a joint can rotate to, from its center
%     position. Applied to all the joints, for consistency.
function ee_work_envelope(DOF, reach, angleIncrement, maxAngle)
    tic; % Start timer (for benchmarking)      
    
    draw_figure();
    
    coordinates = iterate_angles_first(DOF, angleIncrement, maxAngle, reach/DOF);
    
    iterate_coordinates(coordinates);
    colormap(autumn);
    colorbar; 
    
    view(0,90);
    
    toc;
end
 
% draw_figure:
%   Prepares the graphical window
function draw_figure
    global az el;
    
    figure; % launches default window
    
    title('3D Diagram of Work Envelope');
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    grid on;
    
    axis([-50 50 -50 50 -30 60]);
    
    h = rotate3d;
    h.ActionPostCallback = @rotate_event;
    h.Enable = 'on';
    
    uicontrol('Style','text','String','Azimuth','Position', [0 15 50 20]);
    uicontrol('Style','text','String','Elevation','Position', [52 15 50 20]);
    az = uicontrol('Style','edit','String','0','Position', ... 
        [0 0 50 20],'Callback',@az_type_event);
    el = uicontrol('Style','edit','String','0','Position', ... 
        [51 0 50 20],'Callback',@el_type_event);
end

% iterate_angles_first:
%   A version of the iterate_angles function (see below), which is only to
%   be used with the first DOF.
%   Its loop uses parallel workers, and it also has code for the progress bar.
function coordinates = iterate_angles_first(dof, angInc, maxAng, linkLength)

    % The following is a 3-dimensional matrix representing the coordinates. 
    % Add 31 to each of the coordinates (x, y, and z) to get the matrix 
    % index (i.e. x=-30 has an index of x=1, and x=30 has x=61, in the matrix). The value 
    % contained in the matrix represents how many instances of that point was 
    % calculated (i.e. counting multiple solutions).
    coordinates = zeros(61, 61, 61);

    % First joint only iterates from -maxAng degrees to 0 degrees,
    % to optimize the computation. The values are reflected
    % to the other side later, on line 138
    parfor O_int = 0 : int8(maxAng/angInc)
        O = -double(O_int)*angInc;
        coordinates_partial = zeros(61, 61, 61);
            
        T = get_transformation_matrix(1, dof, O, linkLength); % Calculate next transform matrix
        
        if dof > 1 % If not last joint, keep going
            coordinates_partial = iterate_angles(2, T, coordinates_partial, dof, angInc, maxAng, linkLength);
        else % If last joint, calculate end-effector position and round to
             % grid (nearest 5cm)
            x = round(T(1,4)/5, 0) * 5 + 31;
            y = round(T(2,4)/5, 0) * 5 + 31;
            z = round(T(3,4)/5, 0) * 5 + 31;
            coordinates_partial(x, y, z) = coordinates_partial(x, y, z) + 1;
        end
        
        coordinates = coordinates + coordinates_partial;
    end
end
 
% iterate_angles:
%   Recursive function that iterates the joint angles with angleInc
%   increments, to find all the possible joint combinations and
%   their respective end-effector positions.
%   jointNum:
%     Indicates which joint is being called for; is necessary for
%     the recursive function. 1 = first joint, dof = last joint
%   prev_T:
%     The transformation matrix up until the joint in question.
function coordinates_partial = iterate_angles(jointNum, prev_T, coordinates_partial, dof, angInc, maxAng, linkLength)
    min = maxAng;
    max = maxAng;
    
    for O = -min : angInc : max
        T = prev_T * get_transformation_matrix(jointNum, dof, O, linkLength); % Calculate next transform matrix
        
        if jointNum < dof % If not last joint, keep going
            coordinates_partial = iterate_angles(jointNum + 1, T, coordinates_partial, dof, angInc, maxAng, linkLength);
        else % If last joint, calculate end-effector position and round to
             % grid (nearest 5cm)
            x = round(T(1,4)/5, 0) * 5 + 31;
            y = round(T(2,4)/5, 0) * 5 + 31;
            z = round(T(3,4)/5, 0) * 5 + 31;
            coordinates_partial(x, y, z) = coordinates_partial(x, y, z) + 1;
        end
    end
end

function iterate_coordinates(coordinates)
    pointCount = 0;
    points = zeros(4, 500000);
    i = 1;
    
    coordinates = coordinates + flip(coordinates,2);
 
    for x = -30 : 5 : 30
        for y = -30 : 5 : 30
            for z = -30 : 5 : 30
                v = coordinates(x+31, y+31, z+31);
                if v > 0
                    points(1,i) = x;
                    points(2,i) = y;
                    points(3,i) = z;
                    points(4,i) = log(v);
                    i = i + 1;
                    pointCount = pointCount + v;
                end
            end
        end
    end
    fprintf('%i unique, %i total points found\n', i-1, pointCount);
 
    scatter3(points(1,:), points(2,:), points(3,:), 40, points(4,:), 'o', 'filled');
end



function T = get_transformation_matrix(jointNum, dof, Q, d)
    switch jointNum
        case 1
            T = T_base(Q,d);
        case 2
            T = T_turn_side(Q,d);
        case 3
            if dof <= 6
                T = T_repeat_side(Q,d);
            else
                T = T_turn_up(Q,d);
            end
        case 4
            if dof <= 5
                T = T_repeat_side(Q,d);
            elseif dof == 6
                T = T_turn_up(Q,d);
            else
                T = T_turn_side(Q,d);
            end
        case 5
            if dof == 6
                T = T_turn_side(Q,d);
            else
                T = T_turn_up(Q,d);
            end
        case 6
            if dof == 6
                T = T_turn_up(Q,d);
            else
                T = T_turn_side(Q,d);
            end
        case 7
            T = T_turn_up(Q,d);
        case 8
            T = T_turn_side(Q,d);
        case 9
            T = T_turn_up(Q,d);
    end
end

function T = T_base(Q,d)
    T = [cos(Q) -sin(Q) 0 0; sin(Q) cos(Q) 0 0; 0 0 1 d; 0 0 0 1;];
end

function T = T_turn_side(Q,d)
    T = [cos(Q) -sin(Q) 0 -d*sin(Q); 0 0 -1 0; sin(Q) cos(Q) 0 d*cos(Q); 0 0 0 1;];
end

function T = T_repeat_side(Q,d)
    T = [cos(Q) -sin(Q) 0 -d*sin(Q); sin(Q) cos(Q) 0 d*cos(Q); 0 0 1 0; 0 0 0 1;];
end

function T = T_turn_up(Q,d)
    T = [cos(Q) -sin(Q) 0 0; 0 0 1 d; -sin(Q) -cos(Q) 0 0; 0 0 0 1;];  
end
 


% The functions below make the UI interactive
 
function rotate_event(~,evd)
    global az el;
    newView = round(evd.Axes.View);
    az.String = newView(1);
    el.String = newView(2);
end
 
function az_type_event(source,~)
    [~, el] = view;
    view(str2double(source.String), el);
end
 
function el_type_event(source,~)
    [az, ~] = view;
    view(az, str2double(source.String));
end



