function [sys_disc, b, a] = butterworth_lowpass(n, fc_Hz, Ts)
% BUTTERWORTH_HIGHPASS Projeta um filtro Butterworth passa-alta discreto usando FOH.
% Entradas:
%   - n: Ordem do filtro
%   - fc_Hz: Frequência de corte em Hz
%   - Ts: Período de amostragem (em segundos)
% Saídas:
%   - sys_disc: Função de transferência discreta (objeto tf)
%   - b: Coeficientes do numerador do filtro discreto
%   - a: Coeficientes do denominador do filtro discreto

    % Converter frequência de corte para rad/s
    wc = 2 * pi * fc_Hz; 

    % Projetar o filtro Butterworth analógico passa-alta
    [b_analog, a_analog] = butter(n, wc, 'low', 's');

    % Criar sistema contínuo
    sys_cont = tf(b_analog, a_analog);

    % Discretizar usando First-Order Hold (FOH)
    sys_disc = c2d(sys_cont, Ts, 'foh');

    % Extrair coeficientes discretos
    [b, a] = tfdata(sys_disc, 'v');

end