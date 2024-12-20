addpath('../car_project/')

Ts = 1/10;

car = Car(Ts);
[xs, us] = car.steady_state(120 / 3.6);
uTs = us(2);
sys = car.linearize(xs, us);
[sys_lon, ~] = car.decompose(sys);

sysl_lond = c2d(sys_lon, Ts);
[A, B, C, D] = ssdata(sysl_lond);

[nx, nu] = size(B);

Q = 1*eye(nx);
R = 10;

[K,Qf,~] = dlqr(A,-B,Q,R);
K = -K;

%% Minimal invariant set
u_tilde = Polyhedron([1; -1], [uTs + 0.5; - uTs + 0.5]);
W = B*u_tilde;

% Compute first step
omega = (A-B*K)^0*W;

% Loop
k = 1;
while 1
    AkW = (A-B*K)^k*W;
    omega_new = plus(omega,AkW);
    if norm((A-B*K)^k) < 1e-2
        F_inf = omega;
        break
    end
    k = k+1;
    omega_new.minHRep;
    omega = omega_new;
end

%% Tightened Constraints
x_safe = 10;

H = [-1  0];
h = [-6+x_safe];

% X
X = Polyhedron(H,h);

X_tilde = minus(X, F_inf);

% U
U = Polyhedron([1; -1], [1; 1]);

U_tilde = minus(U, K*F_inf);

%% Terminal Components
F = X_tilde.A;
f = X_tilde.b;
M = U_tilde.A;
m = U_tilde.b;
Xf = polytope([F;M*K],[f;m]);
Acl = [A-B*K];
while 1
    prevXf = Xf;
    [T,t] = double(Xf);
    preXf = polytope(T*Acl,t);
    Xf = intersect(Xf, preXf);
    if isequal(prevXf, Xf)
        break
    end
end
[Ff, ff] = double(Xf);
Xf_delta = Polyhedron(Ff, ff);

save('tube_mpc_data.mat', 'Xf_delta', 'X_tilde', 'U_tilde', 'x_safe', 'K', 'Q', 'R', 'Qf');
clear