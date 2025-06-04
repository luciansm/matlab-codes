function [Hbp,b,a] = bandPass_filter_plus_single_integrator(fc,zeta,Ts)
%Filtro paassa-banda com duplo integrador paper 2022
%Implementação Lucian 10/2024
omega_c = fc * 2 * pi;
s = tf('s');
H_bp = (2*zeta*omega_c)^4*s^3/(s^2 + 2*zeta*omega_c*s + omega_c^2)^4;
Hbp = c2d(H_bp,Ts,'tustin');
[b,a] = tfdata(Hbp,'v');