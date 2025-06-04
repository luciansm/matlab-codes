function [magnitude_output, freq_vector, phase_output] = fftf_com_grafico2(input, Fs, unit)
    L = length(input);
    n = L;

    % FFT complexa completa (tamanho n)
    fft_complex = fft(input, n); 

    % Vetor de frequência completo (0 a Fs-Δf)
    freq_full = Fs * (0:n-1) / n; 

    % Vetor omega completo (com tratamento de DC)
    omega_full = 2 * pi * freq_full; 
    omega_full(1) = Inf; % Evita divisão por zero em DC

    % Aplica integrações no DOMÍNIO COMPLEXO COMPLETO
    switch lower(unit)
        case 'acel'
            Y = fft_complex;
        case 'vel'
            Y = fft_complex ./ (1j * omega_full); 
        case 'disp'
            Y = fft_complex ./ ( (1j * omega_full).^2 );
        otherwise
            error('Unidade inválida. Use ''acel'', ''vel'' ou ''disp''.');
    end

    % Extrai parte single-sided (0 a Fs/2)
    Y_single = Y(1:n/2+1);
    freq_vector = freq_full(1:n/2+1); 

    % Magnitude normalizada e correção de amplitude
    magnitude_output = abs(Y_single) / n; 
    if n > 1
        magnitude_output(2:end-1) = 2 * magnitude_output(2:end-1);
    end

    % Fase (graus)
    phase_output = angle(Y_single) * (180/pi); 

    % Plot
    figure;
    loglog(freq_vector, magnitude_output, 'linewidth', 1.5);
    xlabel('Frequência (Hz)');
    grid on;
    title(sprintf('Espectro de %s', unit));
    switch lower(unit)
        case 'acel', ylabel('Aceleração (mag)');
        case 'vel', ylabel('Velocidade (mag)');
        case 'disp', ylabel('Deslocamento (mag)');
    end
end