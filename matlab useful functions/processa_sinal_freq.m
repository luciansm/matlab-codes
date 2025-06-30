function sinal_processado = processa_sinal_freq(sinal, Fs, operacao)
%PROCESSA_SINAL Processa um sinal via domínio da frequência
%   sinal: Vetor do sinal de entrada
%   Fs: Frequência de amostragem (Hz)
%   operacao: String com a operação desejada:
%       'derivar' - Primeira derivada
%       'derivar2' - Segunda derivada
%       'integrar' - Integral
%       'integrar2' - Integral dupla

N = numel(sinal);
X = fft(sinal);

% Cálculo das frequências angulares (ω)
k = 0:N-1;
f = (k - (k >= N/2)*N) * Fs/N;
omega = 2*pi*f;

% Tratamento do componente DC (ω=0)
omega(omega == 0) = Inf;

% Aplicação da operação no domínio da frequência
switch operacao
    case 'derivar'
        Y = X .* (1j * omega);
    case 'derivar2'
        Y = X .* (1j * omega).^2;
    case 'integrar'
        Y = X ./ (1j * omega);
    case 'integrar2'
        Y = X ./ ((1j * omega).^2);
    otherwise
        error('Operação inválida. Use: ''derivar'', ''derivar2'', ''integrar'' ou ''integrar2''.');
end

% Remove componentes DC resultantes
Y(omega == Inf) = 0;

% Transformada inversa e parte real
sinal_processado = real(ifft(Y));
end