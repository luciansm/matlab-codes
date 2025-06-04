function [avg_magnitude, freq_vector, phase_output] = fftf_media(input, Fs, unit, window_size)
    % Função para calcular e plotar a FFT média de um sinal dividido em janelas.
    % input: sinal de entrada
    % Fs: frequência de amostragem
    % unit: unidade ('acel', 'vel', ou outro para deslocamento)
    % window_size: tamanho de cada janela
    
    L = length(input);
    num_windows = floor(L / window_size);
    input_truncated = input(1:num_windows * window_size);
    
    % Redimensiona o sinal truncado em janelas
    windows = reshape(input_truncated, window_size, num_windows);
    
    % Calcula o número de bins de frequência
    num_bins = floor(window_size / 2) + 1;
    magnitude_accumulator = zeros(num_bins, 1);
    
    for i = 1:num_windows
        window_data = windows(:, i);
        fft_window = fft(window_data, window_size);
        
        % Calcula a magnitude e normaliza
        mag = abs(fft_window) / window_size;
        mag = mag(1:num_bins);
        mag(2:end-1) = 2 * mag(2:end-1); % Ajusta exceto DC e Nyquist
        
        magnitude_accumulator = magnitude_accumulator + mag;
    end
    
    avg_magnitude = magnitude_accumulator / num_windows;
    
    % Vetor de frequências
    freq_vector = Fs * (0:(num_bins - 1)) / window_size;
    freq_vector = freq_vector(:); % Converte para vetor coluna
    
    % Aplica conversão de unidade
    switch unit
        case 'acel'
            final_output = avg_magnitude;
            y_label = 'Aceleração (mag)';
            title_str = 'Espectro de Aceleração Média';
        case 'vel'
            final_output = avg_magnitude ./ (2 * pi * freq_vector);
            y_label = 'Velocidade (mag)';
            title_str = 'Espectro de Velocidade Média';
        otherwise
            final_output = avg_magnitude ./ (2 * pi * freq_vector).^2;
            y_label = 'Deslocamento (mag)';
            title_str = 'Espectro de Deslocamento Médio';
    end
    
    % Plotagem
    figure;
    loglog(freq_vector, final_output, 'linewidth', 1.5);
    xlabel('Frequência (Hz)');
    ylabel(y_label);
    title({title_str, sprintf('Fs = %d Hz, Tamanho da Janela = %d', Fs, window_size)});
    grid on;
    hold on;
    
    % Saídas
    magnitude_output = final_output;
    phase_output = []; % Fase não é calculada na média
end