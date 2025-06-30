function [magnitude, frequency] = frequency_derivative_integration3(sinal, Fs, operacao, aplicar_janela)
%FREQUENCY_DERIVATIVE_INTEGRATION
%  Deriva ou integra um sinal no domínio da frequência, com opção de
%  janelamento de Hann.
%
%  sinal   : vetor do sinal no tempo (1 D)
%  Fs      : frequência de amostragem [Hz]
%  operacao: 'derivar', 'derivar2', 'integrar', 'integrar2'
%  aplicar_janela (opcional, default = true) : se verdadeiro, aplica
%               janela de Hann e corrige o ganho coerente
%
%  magnitude : módulo da operação (apenas frequências positivas)
%  frequency : eixo de frequência correspondente [Hz]

    if nargin < 4
        aplicar_janela = true;
    end

    sinal = sinal(:);          % garante coluna
    N     = numel(sinal);

    %% --- Janelamento de Hann ---
    if aplicar_janela
        w   = hann(N,'periodic');     % janela de Hann (≡ Hanning)
        CG  = sum(w)/N;              % coherent gain da janela
        sinal = sinal .* w;          % aplica janela
    else
        CG  = 1;                     % sem correção se não houver janela
    end

    %% --- FFT ---
    X = fft(sinal);

    %% --- Eixo de frequências rad/s e Hz ---
    f     = (0:N-1).' * Fs / N;      % 0 … Fs*(N-1)/N
    f     = f - Fs * (f >= Fs/2);    % reorganiza p/ faixa −Fs/2…Fs/2
    omega = 2*pi*f;
    omega_safe = omega;
    omega_safe(omega == 0) = Inf;    % evita divisão por zero

    %% --- Operação no domínio da frequência ---
    switch lower(operacao)
        case 'derivar'
            Y = X .* (1j*omega_safe);
        case 'derivar2'
            Y = X .* (1j*omega_safe).^2;
        case 'integrar'
            Y = X ./ (1j*omega_safe);
        case 'integrar2'
            Y = X ./ ((1j*omega_safe).^2);
        otherwise
            warning('Operação não reconhecida – retornando espectro original.');
            Y = X;
    end
    Y(omega == 0) = 0;               % força componente DC = 0, se aplicável

    %% --- Seleciona apenas frequências positivas (0 – Nyquist) ---
    if mod(N,2)==0            % N par
        idx_pos = 1:N/2+1;
    else                      % N ímpar
        idx_pos = 1:(N+1)/2;
    end

    %% --- Escalona módulo |Y| ---
    % Dividimos por (N/2) como no original e pela correção de ganho CG
    magnitude = abs(Y(idx_pos)) / ((N/2) * CG);
    frequency = abs(f(idx_pos));
end
