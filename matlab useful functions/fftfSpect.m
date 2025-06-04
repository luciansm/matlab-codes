function [mag, freq, phase] = fftfSpect(input, Fs, unit)
% unit = 'acel' | 'vel' | 'disp'

N      = numel(input);
X      = fft(input, N);                    % FFT de entrada
freq   = (0:N-1)*(Fs/N);                  % Vetor de frequência
omega  = 2*pi*freq;  
omega(1) = Inf;      % Evita divisão por zero em DC

switch lower(unit)
    case 'acel'                         % Aceleração (sem integração)
        Y = X;
    case 'vel'                          % Integração única (acel -> vel)
        Y = X ./ (1j*omega);
    case 'disp'                         % Dupla integração (acel -> disp)
        Y = -X ./ (omega.^2);           
    otherwise
        error('Unidade deve ser ''acel'', ''vel'', ''disp''');
end

% Processamento single-sided
half = 1:N/2+1;  % Índices para single-sided
mag = abs(Y)/N;   % Magnitude normalizada
phase = angle(Y)*180/pi; % Fase em graus

% Correção de amplitude (exceto DC e Nyquist)
mag(2:N/2) = 2*mag(2:N/2); 

% Extrair apenas a parte single-sided
mag = mag(half);
phase = phase(half);
freq = freq(half);
end