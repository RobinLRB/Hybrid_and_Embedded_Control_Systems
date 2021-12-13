%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Hybrid and Embedded control systems
% Homework 1
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initialization
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear
init_tanks;
g = 9.82;
%s = tf('s');

chi = 0.5;
zeta = 0.8;
omega0 = 0.2;


Tau = 1/alpha1*sqrt(2*tank_h10/g);
tau = Tau;
k_tank = 60*beta*Tau;
gamma_tank = alpha1^2/alpha2^2;
uss = alpha2/beta*sqrt(2*g*tank_init_h2)*100/15; % steady state input
yss = 40; % steady state output

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Continuous Control design
s = tf('s');
uppertank = k_tank / (1 + Tau*s); % Transfer function for upper tank
lowertank = gamma_tank / (1 + gamma_tank*Tau*s); % Transfer function for upper tank
G = uppertank*lowertank; % Transfer function from input to lower tank level

% Calculate PID parameters
[K_pid,Ti,Td,N]=polePlacePID(chi, omega0, zeta, Tau, gamma_tank, k_tank);
F = tf([K_pid*(Ti+Ti*Td*N) K_pid*(Ti*N+1) K_pid*N] , [Ti Ti*N 0]);
%bode(F*G);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Digital Control design
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Ts = 4; % Sampling time
%sys = c2d(F, Ts, 'ZOH');

% Discretize the continous controller, save it in state space form
%[A_discretized,B_discretized,C_discretized,D_discretized] = tf2ss(sys.num{1} , sys.den{1});

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Discrete Control design
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Discretize the continous state space system, save it in state space form
A = [-1/Tau 0 ; 1/Tau -1/gamma_tank/Tau];
B = [k_tank/Tau ; 0];
C = [0  1];
D = 0;
[num_sys2 , den_sys2] = ss2tf(A , B , C , D);
sys2_prime = c2d(tf(num_sys2 , den_sys2) , Ts , 'ZOH');
[Phi,Gamma,C_dt,D_dt] = tf2ss(sys2_prime.num{1} , sys2_prime.den{1});


% Observability and reachability
Wc = (rank(ctrb(Phi , Gamma))==2);
Wo = (rank(obsv(Phi , C_dt))==2);

% State feedback controller gain
L = acker(Phi , Gamma , pole(sys2_prime));
% observer gain
K = place(Phi' , C' , 5*pole(sys2_prime));
% reference gain
lr = 1/(C * inv(eye(2) - Phi + Gamma*L) * Gamma);

% augmented system matrices
Aa = 1;
Ba = 1;
