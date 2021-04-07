function ~ = crossTheBridge()

% Creates a symbolic math object
syms u

% Establishes the distance between the wheels
d = 0.235; %meters

% Creates functions for parametric movement in the x and y directions
rx =  4*(0.3960*cos(2.65*(0.2*u+1.4)));
ry =  4*(-0.99*sin(0.2*u+1.4));

% Creates a vector function
r = [rx ry 0];

% Asserts certain characteristics of u to aid in simplification of symbolic
% expressions
assume(u, {'real', 'positive'});

% Finds the derivative of the x and y position, as well as of the position
% vector function. This represents velocity.
rPrimeX = simplify(diff(rx, u));
rPrimeY = simplify(diff(ry, u));
rPrime = [rPrimeX rPrimeY 0];

% Finds the magnitude of the velocity, otherwise known as speed
magRPrime = norm(rPrime);

% Determines the unit tangent vector by normalizing the velocity vector
T_hat = simplify(rPrime./norm(rPrime));

% Finds the derivative of the unit tangent vector
dT_hat = simplify(diff(T_hat, u));

% Crosses the unit tangent with its own derivative to create a function
% that represents the angular velocity over time.
omega = simplify(cross(T_hat, dT_hat));

% Finds the magnitude of the angular velocity, while also preserving the
% sign. A standard "norm()" function does not preserve the sign, so the
% dot product of omega and a unit vector in the k direction
magOmega = simplify(dot(omega, [0 0 1]));

% Determines the speeds of each of the wheels using the speed of the robot
% and the angular velocity
V_L = simplify(magRPrime - magOmega*(d/2));
V_R = simplify(magRPrime + magOmega*(d/2));

% Turns the symbolic wheel velocity functions into calculable functions
leftVelocity = @(t) double(subs(V_L, u, t));
rightVelocity = @(t) double(subs(V_R, u, t));

% Finds the initial position and heading of the robot to set it in the
% right place. Does this by converting the symbolic functions to calculable
% ones and finding their values at time 0.
bridgeStart = double(subs(r, u, 0));
startingTHat = double(subs(T_hat, u, 0));

% Creates a pointer that sends the velocities to the ROS node
pubvel = rospublisher('/raw_vel');

% Initializes a message holder that communicates the velocities to the ROS
% node
message = rosmessage(pubvel);

% Sets the initial velocity of each wheel to 0 m/s and sends that to the
% ROS node
message.Data = [0, 0];
send(pubvel, message);

% Places the Neato at the correct starting position and angles it in the
% correct starting direction
placeNeato(bridgeStart(1), bridgeStart(2), startingTHat(1), startingTHat(2), 2);

% Allows the Neato time to fall onto the bridge before beginning the
% movement of the wheels.
pause(2);

% Begins tracking elapsed time
rostic;

while 1
    
    %Sets a variable equal to the current elapsed time
    elapsed = rostoc;
    
    if elapsed < 16
        
        % Sets the wheel velocities to the correct speeds at the current
        % time
        left = leftVelocity(elapsed);
        right = rightVelocity(elapsed);
        
        % Sends velocity data to the ROS node
        message.Data = [left, right];
        send(pubvel, message);
    else
        
        % Stops the robot moving when it should ideally reach the end of
        % the bridge (often stops just a hair too late)
        message.Data = [0,0];
        send(pubvel, message);
        break
    end
end

end