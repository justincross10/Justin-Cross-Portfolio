% Justin Cross: CRSKAT005
% Save a City from an Asteroid
% 03/04/2025

%% Rocket model
% State variables and generalized coordinates for rocket
syms x y th alpha dx dy dth ddx ddy ddth F

qr = [x; y; th]; % Degrees of freedom for position
dqr = [dx; dy; dth]; % Generalised coordinates of velocity
ddqr = [ddx; ddy; ddth]; % Generalised coordinates of acceleration

syms L1 L2 mr g I
% Constants for rocket
L1 = 5; %(Width m)
L2 = 15; %(Height m)
mr = 1000; %(Mass kg)
g = 9.81; 

I = (1/12)*mr*(L1^2 + L2^2) + mr*(L2/2 - 3.5)^2; % Moment of inertia of rocket using parallel axis theorem to calculate rotational kinetic energy

% Rotations for rocket to represent rocket coordinates in inertial frame
R_R01 = Rotz(th);
R_R10 = transpose(R_R01);

% Position of rocket
rR_body = [x;y;0]; % Position of rocket in body frame
rR_in = R_R10*rR_body; % Position of rocket in inertial frame

% Linear velocity of rocket required for translational kinetic energy
dR = jacobian(rR_in, qr)*dqr;

% Angular velocity of rocket required for rotational kinetic energy
wR = [0;0;dth];

% Kinetic Energy of rocket required for mass matrix
T_R = simplify(0.5*mr*transpose(dR)*dR) + simplify(0.5*I*transpose(wR)*wR);

% Potential Energy of rocket required for gravity matrix
V_R = mr*g*rR_in(2);

% Modelling the thrust force as a generalised force
% Force rotations for force direction to be represented in inertial frame
R_F01 = Rotz(alpha+th); % Consider rocket rotation and thrust angle
R_F10 = transpose(R_F01);

% Position of force
rF_body = [0;-3.5;0]; % Position of force in body frame
rF_in = rR_in + R_R10*rF_body; % Position of force in inertial frame

% Force as a vector wrt inertial frame
F_b = [0;F;0];
F_i = R_F10*F_b;

% Partial derivatives of position where force acts
px = diff(rF_in, x);
py = diff(rF_in, y);
pth = diff(rF_in, th);

% Generalised force components for final model of rocket
Qx = F_i(1)*px(1) + F_i(2)*px(2) + F_i(3)*px(3);
Qy = F_i(1)*py(1) + F_i(2)*py(2) + F_i(3)*py(3);
Qth = F_i(1)*pth(1) + F_i(2)*pth(2) + F_i(3)*pth(3);
Q = simplify([Qx;Qy;Qth])

% Define Mass Matrix used in manipulator equation
M = simplify(hessian(T_R,dqr))

% Define Gravity Matrix used in manipulator equation
G = jacobian(V_R,qr);
G = simplify(transpose(G))

% Define Coriolis Matrix
dM = derivMatrix(M,dqr,qr); % Find derivatuve of mass matrix for coriolis
C = dM*dqr - transpose(jacobian(T_R,qr));
C = simplify(C)

% Manipulator equation used to describe the rocket model
ManipulatorEqn = M*ddqr + C + transpose(G) == Q
EOM = transpose(M)*(Q-C-G)
%% Asteroid

clear c_x; % Clear the variable that will store damping coefficients
iterations = 50; % Define the number of iterations for the simulation
for i = 1:iterations % Loop over 50 iterations
    sim('AsteroidImpact.slx');
    ddx = diff(ast_dx) ./ diff(simulation_time); % Compute the acceleration (ddx) by differentiating velocity (ast_dx) with respect to time (simulation_time)
    m_ast = 10000; % Weight of asteroid
    c = (m_ast * ddx) ./ (-ast_dx(2:end)); % Compute damping coefficients c from the dynamic equation: m·ddx + c·dx = 0
    c_x(i) = mean(c);  % Calculate the average damping coefficient for 50 iterations
end
c_final = mean(c_x) % After all iterations - compute the overall average damping coefficient

%% Functions
% Define Matrix Deriv
function dM = derivMatrix(M,dq,q)
    dM = sym(zeros(length(M),length(M)));
    for i=1:length(M)
        for j=1:length(M)
            dM(i,j) = jacobian(M(i,j),q)*dq;
        end
    end
    dM = simplify(dM);
end
% Rotation matrix to represent object in inertial frame
function A = Rotz(th)
    A = [cos(th)   sin(th) 0;... 
         -sin(th)  cos(th) 0;...
         0        0        1];
end
