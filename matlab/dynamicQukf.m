function Qk = dynamicQukf(u_cmd)

Ts = 1;
error_cmd = 0.05;
sigma_a = 1e-5;

I3 = eye(3);

% External/unmodelled acceleration process noise
q = sigma_a^2;

Q_external = [ ...
    q*Ts^3/3*I3, q*Ts^2/2*I3;
    q*Ts^2/2*I3, q*Ts*I3 ];

% Actuator execution uncertainty:
% u_real = u_cmd + error_cmd*u_cmd*N(0,1)
Sigma_act = diag((error_cmd .* u_cmd).^2);

G = [ ...
    0.5*Ts^2*I3;
    Ts*I3 ];

Q_act = G * Sigma_act * G';

% Total process noise
Qk = Q_external + Q_act;

end