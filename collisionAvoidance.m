%% Find path with no collision
function qMatrix = collisionAvoidance(robot, qStart, qEnd, face, vertex, faceNormals)
    qWaypoints = [qStart; qEnd]; 
    isCollision = true; 
    checkedTillWaypoint = 1;  % Track up to where the path has been checked
    qMatrix = []; % Initialise q matrix
    maxAttempts = 5;  % Maximum number of waypoint attempts
    attemptCount = 0;  % Count how many waypoints have been attempted

    while isCollision
        startWaypoint = checkedTillWaypoint;  % Start checking from last successful waypoint
        for i = startWaypoint:size(qWaypoints, 1) - 1
            % Interpolate between waypoints
            qMatrixJoin = InterpolateWaypointRadians(qWaypoints(i:i+1,:), deg2rad(10));
            
            % Check if the interpolated path is collision-free
            if ~IsCollision(robot, qMatrixJoin, face, vertex, faceNormals)
                % No collision, proceed with movement
                qMatrix = [qMatrix; qMatrixJoin]; %#ok<AGROW>
                isCollision = false;
                checkedTillWaypoint = i + 1; % Increment waypoint
                
                % Try joining to the final goal
                qMatrixJoin = InterpolateWaypointRadians([qMatrix(end,:); qEnd], deg2rad(10));
                if ~IsCollision(robot, qMatrixJoin, face, vertex, faceNormals)
                    % No collision till the goal
                    qMatrix = [qMatrix; qMatrixJoin];
                    disp('Reached goal without collision.');
                    isCollision = false;
                    return;  % Exit as the goal is reached without collision
                end
            else
                % Collision detected, pick a random waypoint that is collision-free
                disp('Collision detected. Avoiding...');
                qRand = getRandomWaypoint(robot.model.qlim);
                attemptCount = attemptCount + 1;  % Increment the attempt counter
                
                % If maximum attempts reached, abandon the path planning
                if attemptCount >= maxAttempts
                    disp('Abandoning path planning: too many collision attempts.');
                    qMatrix = [];  % Return an empty qMatrix to signal failure
                    return;
                end
                
                % Insert the random waypoint
                qWaypoints = [qWaypoints(1:i,:); qRand; qWaypoints(i+1:end,:)];
                isCollision = true;
                break;  % Restart the loop with the new random waypoint
            end
        end
    end
end

%% FineInterpolation
% Use results from Q2.6 to keep calling jtraj until all step sizes are
% smaller than a given max steps size
function qMatrix = FineInterpolation(q1,q2,maxStepRadians)
if nargin < 3
    maxStepRadians = deg2rad(1);
end
    
steps = 2;
while ~isempty(find(maxStepRadians < abs(diff(jtraj(q1,q2,steps))),1))
    steps = steps + 1;
end
qMatrix = jtraj(q1,q2,steps);
end

%% InterpolateWaypointRadians
% Given a set of waypoints, finely intepolate them
function qMatrix = InterpolateWaypointRadians(waypointRadians,maxStepRadians)
if nargin < 2
    maxStepRadians = deg2rad(1);
end

qMatrix = [];
for i = 1: size(waypointRadians,1)-1
    qMatrix = [qMatrix ; FineInterpolation(waypointRadians(i,:),waypointRadians(i+1,:),maxStepRadians)]; %#ok<AGROW>
end
end