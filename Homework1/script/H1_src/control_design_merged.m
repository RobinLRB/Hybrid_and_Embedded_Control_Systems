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
Tau = 1/alpha1*sqrt(2*tank_h10/g);
k_tank = 60*beta*Tau;
gamma_tank = alpha1^2/alpha2^2;
uss = alpha2/beta*sqrt(2*g*tank_init_h2)*100/15; % steady state input
yss = 40; % steady state output

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Continuous Control design
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
s = tf('s');
uppertank = k_tank / (1+Tau*s); % Transfer function for upper tank
lowertank = gamma_tank / (1 + gamma_tank*Tau*s); % Transfer function for upper tank
G = uppertank*lowertank; % Transfer function from input to lower tank level

% Calculate PID parameters
chi = 0.5;
omega0 = 0.2;
zeta = 0.8;
[K_pid,Ti,Td,N]=polePlacePID(chi,omega0,zeta,Tau,gamma_tank,k_tank);
F = K_pid * (1 + 1/(Ti*s) + Td*N*s/(s+N));

%bode(F*G)

H_cl_continuous = F*G/(1+F*G);
%step(H_cl_continuous);

continuous_poles = pole(minreal(H_cl_continuous));

two_continuous_poles = continuous_poles(abs(real(continuous_poles) - (-chi)) > 0.000001);
two_continuous_poles_2 = [-0.5000 ; -0.5000];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Digital Control design
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Ts = 0.2; % Sampling time
sys = c2d(F, Ts, 'ZOH');

% Discretize the continous controller, save it in state space form
[A_discretized,B_discretized,C_discretized,D_discretized] = tf2ss(sys.num{1} , sys.den{1});

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Discrete Control design
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Discretize the continous state space system, save it in state space form
A = [-1/Tau 0; 1/Tau -1/(Tau*gamma_tank)];
B = [k_tank/Tau; 0];
C = [0 1];
D = 0;

%[num_sys2 , den_sys2] = ss2tf(A , B , C , D);
%sys2_prime = c2d(tf(num_sys2 , den_sys2) , Ts , 'ZOH');
%[Phi,Gamma,C_dt,D_dt] = tf2ss(sys2_prime.num{1} , sys2_prime.den{1});
sys2 = ss(A , B , C , D);
sys2_prime = c2d(sys2 , Ts , 'ZOH');

Phi = sys2_prime.A;
Gamma = sys2_prime.B;
C_dt = sys2_prime.C;
D_dt = sys2_prime.D;

discrete_poles = exp(Ts*two_continuous_poles);
discrete_poles_2 = exp(Ts*two_continuous_poles_2);

% Observability and reachability
Wc = (rank(ctrb(Phi , Gamma))==2);
Wo = (rank(obsv(Phi , C_dt))==2);

% State feedback controller gain
L = acker(Phi , Gamma , discrete_poles);
% observer gain
K = acker(Phi' , C' , discrete_poles_2).';
% reference gain
lr = 1/(C * inv(eye(2) - Phi + Gamma*L) * Gamma);

% augmented system matrices
Aa = [Phi -Gamma*L; K*C (Phi-Gamma*L-K*C)];
Ba = [Gamma*lr;Gamma*lr];
%D = [lr ; 0];

[num_sys3 , den_sys3] = ss2tf(Aa , Ba , [C zeros(size(C))] , D);
sys3 = tf(num_sys3, den_sys3);

%quantization level

bit = 10;
resol = 100/(2^bit);
%QL = 0.05;

sim('tanks');


