function [magnitude, frequency] = frequency_derivative_integration2(sinal, Fs, operacao, varargin)
%FREQUENCY_DERIVATIVE_INTEGRATION Deriva ou integra um sinal no domínio da frequência
%   sinal: Vetor do sinal no tempo (1D)
%   Fs: Frequência de amostragem [Hz]
%   operacao: 'derivar', 'derivar2', 'integrar', 'integrar2'
%   Parâmetros opcionais (para Welch):
%       segmento: Tamanho do segmento (amostras)
%       overlap: Porcentagem de sobreposição [0-100]

sinal = sinal(:); % Garante vetor coluna
N = numel(sinal);

% Configura parâmetros opcionais (Welch)
usar_welch = false;
segmento = [];
overlap_perc = 0;

if nargin >= 4 && ~isempty(varargin{1})
    usar_welch = true;
    segmento = varargin{1};
    if nargin >= 5 && ~isempty(varargin{2})
        overlap_perc = varargin{2};
    end
end

% Verifica se usa Welch
if usar_welch
    % Ajusta tamanho do segmento se necessário
    if segmento > N
        segmento = N;
        overlap_perc = 0;
    end
    
    % Calcula overlap em amostras
    overlap_amostras = round(overlap_perc / 100 * segmento);
    passo = segmento - overlap_amostras;
    
    % Verifica passo válido
    if passo <= 0
        error('Overlap deve ser menor que 100%');
    end
    
    % Número de segmentos
    num_segmentos = max(1, floor((N - segmento) / passo) + 1);
    
    % Janela (Hamming) e fator de normalização
    janela = hamming(segmento);
    soma_janela = sum(janela);
    
    % Inicializa acumulador de magnitudes
    n_bins = floor(segmento/2) + 1; % Número de bins positivos
    mag_acum = zeros(n_bins, 1);
    
    % Frequências para um segmento (apenas positivas)
    frequency = (0:n_bins-1)' * Fs / segmento;
    
    % Frequências completas (com negativas) para operação
    f_full = (0:segmento-1)' * Fs / segmento;
    f_full(f_full >= Fs/2) = f_full(f_full >= Fs/2) - Fs;
    omega_full = 2*pi * f_full;
    omega_full_safe = omega_full;
    omega_full_safe(omega_full == 0) = Inf; % Evita divisão por zero
    
    % Processa cada segmento
    for i = 1:num_segmentos
        inicio = (i-1)*passo + 1;
        fim = min(inicio + segmento - 1, N);
        
        % Ajusta último segmento se necessário
        seg = sinal(inicio:fim);
        if numel(seg) < segmento
            seg = [seg; zeros(segmento - numel(seg), 1)];
        end
        
        % Aplica janela e FFT
        seg_janelado = seg .* janela;
        X_seg = fft(seg_janelado, segmento);
        
        % Aplica operação
        switch lower(operacao)
            case 'derivar'
                Y_seg = X_seg .* (1j * omega_full_safe);
            case 'derivar2'
                Y_seg = X_seg .* (1j * omega_full_safe).^2;
            case 'integrar'
                Y_seg = X_seg ./ (1j * omega_full_safe);
            case 'integrar2'
                Y_seg = X_seg ./ ((1j * omega_full_safe).^2);
            otherwise
                Y_seg = X_seg;
        end
        
        % Corrige DC e normaliza amplitude
        Y_seg(omega_full == 0) = 0;
        Y_seg_normalizado = Y_seg * 2 / soma_janela;
        
        % Extrai magnitudes positivas
        mag_seg = abs(Y_seg_normalizado(1:n_bins));
        mag_acum = mag_acum + mag_seg;
    end
    
    % Média das magnitudes
    magnitude = mag_acum / num_segmentos;
    
else % Modo sem Welch (FFT direta)
    % FFT do sinal completo
    X = fft(sinal);
    
    % Frequências reais associadas
    f = (0:N-1)' * Fs/N;
    f = f - Fs * (f >= Fs/2);
    omega = 2*pi * f;
    omega_safe = omega;
    omega_safe(omega == 0) = Inf; % Evita NaNs
    
    % Aplica operação
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
            Y = X;
    end
    
    % Corrige DC
    Y(omega == 0) = 0;
    
    % Extrai frequências positivas
    if mod(N, 2) == 0
        idx_pos = 1:N/2+1;
    else
        idx_pos = 1:(N+1)/2;
    end
    
    magnitude = abs(Y(idx_pos)) / (N/2);
    frequency = abs(f(idx_pos));
end
end