function [magnitude, frequency] = frequency_derivative_integration(sinal, Fs, operacao)
%FREQUENCY_DERIVATIVE_INTEGRATION Deriva ou integra um sinal no domínio da frequência
%   sinal: Vetor do sinal no tempo (1D)
%   Fs: Frequência de amostragem [Hz]
%   operacao: 'derivar', 'derivar2', 'integrar', 'integrar2'

sinal = sinal(:); % garante vetor coluna
N = numel(sinal);
X = fft(sinal);

% Frequências reais associadas
f = (0:N-1)' * Fs/N;
% Reorganiza para -Fs/2 até Fs/2 se necessário
f = f - Fs * (f >= Fs/2);
omega = 2*pi*f;

% Evita divisão por zero (DC)
omega_safe = omega;
omega_safe(omega == 0) = Inf;  % evitar NaNs na integração

% Aplica a operação no domínio da frequência
switch lower(operacao)
    case 'derivar'
        Y = X .* (1j * omega_safe);
    case 'derivar2'
        Y = X .* (1j * omega_safe).^2;
    case 'integrar'
        Y = X ./ (1j * omega_safe);
    case 'integrar2'
        Y = X ./ ((1j * omega_safe).^2);
    otherwise
        disp('Nenhuma opção escolhida, fazendo o espectro do sinal de entrada....');
        Y = X;
end

% Corrige valor DC
Y(omega == 0) = 0;

% Retorna apenas frequências positivas (até Nyquist)
if mod(N, 2) == 0
    % N par: inclui Nyquist
    idx_pos = 1:N/2+1;
else
    % N ímpar: até antes do Nyquist
    idx_pos = 1:(N+1)/2;
end

magnitude = abs(Y(idx_pos)) / (N/2);
frequency = abs(f(idx_pos));
end