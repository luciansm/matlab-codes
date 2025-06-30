%Função para plotar FFT do sinal de entrada.
%input = sinal de entrada
%Fs = frequência de amostragem
%Lucian Ribeiro da Silva - 16/10/23
%[output, freq_vector,phase_output] = fftf(input,Fs)
function [magnitude_output, freq_vector, phase_output] = fftf_com_grafico(input, Fs,unit)
L = length(input);
n = L;

% Calcula a FFT completa (complexa)
fft_output = fft(input, n);

% Cria o vetor de frequências até a metade da frequência de amostragem
freq_vector = Fs * (0:(n/2)) / n;

% Calcula a magnitude e normaliza
magnitude_output = abs(fft_output) / n;
magnitude_output = magnitude_output(1:n/2+1);

% Ajusta a amplitude (multiplica por 2, exceto DC e Nyquist)
magnitude_output(2:end-1) = 2 * magnitude_output(2:end-1);

% Calcula a fase (em graus) do resultado complexo da FFT
phase_output = angle(fft_output(1:n/2+1)) * (180 / pi);

%Passando para deslocamento
deslocamento = magnitude_output' ./ (2 * pi * freq_vector).^2;

%Passando para velocidade
velocidade = magnitude_output' ./ (2 * pi * freq_vector);

switch unit
    case 'acel'
    case 'vel'
        magnitude_output = velocidade;
    otherwise
        magnitude_output = deslocamento;
end

end
