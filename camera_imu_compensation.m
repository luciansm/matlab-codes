clc; clear; close all;

%% Ganhos e outras sintonias de testes=====================================
gainIMU = 0.85;  %Ganho utilizado para multiplar o sinal da IMU. Depois é 
                %necessário rever isso.


%Distância ao alvo para calculo da projeção do movimento linear no plano de
%imagem da câmera a partir dos movimentos angulares medidos pela IMU
%localizada acima da câmera
dist_alvo = 6640 * 1; %em mm
Fs = 1000; %Frequência de amostragem da câmera e da ESP32


%Filtro passa-banda para tirar DC e ruido de baixa frequência
Fc1 = 1;     % frequência de corte inferior (Hz)
Fc2 = 250;   % frequência de corte superior (Hz)
[b1, a1] = butter(6, [Fc1 Fc2]/(Fs/2), 'bandpass');

%Filtro passa-alta para tirar DC do sinal no tempo
Fc1 = 1;     % frequência de corte inferior (Hz)
[b2, a2] = butter(6, Fc1/(Fs/2), 'high');



%% LEITURA DOS ARQUIVOS TXT E CSV DOS SINAIS DA IMU DA ESP32 E DOS SINAIS
%DE DESLOCAMENTO DA CÂMERA IRIS MX

%Leitura dos dados da IMU da esp32 (em cima da câmera)=====================
[filename, pathname] = uigetfile({'*.txt', 'Arquivos CSV/TXT (*.csv)'; '*.*', 'Todos os arquivos (*.*)'}, 'Selecione o arquivo');

% Verifica se o usuário cancelou a seleção de arquivo
if isequal(filename, 0)
    disp('Seleção de arquivo cancelada');
    return;
end

% Caminho completo do arquivo
fullpath = fullfile(pathname, filename);

% Lê e processa os dados do arquivo CSV
[data1] = readtable(fullpath);


% %Leitura dos dados da câmera (sujeita a vibração/sem correção)===========
[filename1, pathname] = uigetfile({'*.csv', 'Arquivos CSV (*.csv)'; '*.*', 'Todos os arquivos (*.*)'}, 'Selecione o arquivo CSV');
%
% Verifica se o usuário cancelou a seleção de arquivo
if isequal(filename1, 0)
    disp('Seleção de arquivo cancelada');
    return;
end

% Caminho completo do arquivo
fullpath = fullfile(pathname, filename1);
% Lê e processa os dados do arquivo CSV
[data2] = readtable(fullpath);
Des_Y_cam = (data2.Var3);
Des_Z_cam = (data2.Var5);


% %Leitura dos dados da câmera SEM VIBRAÇÃO NA BASE (MESA PARADA)==========
[filename, pathname] = uigetfile({'*.csv', 'Arquivos CSV (*.csv)'; '*.*', 'Todos os arquivos (*.*)'}, 'Selecione o arquivo CSV');
%
% Verifica se o usuário cancelou a seleção de arquivo
if isequal(filename, 0)
    disp('Seleção de arquivo cancelada');
    return;
end

% Caminho completo do arquivo
fullpath = fullfile(pathname, filename);
% Lê e processa os dados do arquivo CSV
[data3] = readtable(fullpath);
Des_Z_cam_base_parada = (data3.Var5);

if (max(Des_Z_cam_base_parada) > 70)
Des_Z_cam_base_parada = Des_Z_cam_base_parada / 1000; %passa para mm se estiver em microns
end


%Aqui é uma verificação se os dados fornecidos pela câmera estão em microns
%ou mm. Se estiver acima do valor de 70, provavelmente estão em microns,
%portanto divide-se por 1000 para passar para mm.
if (max(Des_Z_cam) > 70)
Des_Z_cam = Des_Z_cam / 1000; %passa para mm se estiver em microns
end

%% OBTENÇÃO DO INSTANTE EM QUE O LASER É ATIVADO E AJUSTE DOS VETORES DOS
%SINAIS A SEREM PROCESSADOS

%Extrai a amostra do momento que a câmera identificou o laser do nome do
%arquivo dos sinais de deslocamento da câmera. O valor da amostra está
%sendo colocado manualmente no fim do nome do arquivo de modo a facilitar a
%identificação pelo programa de processamento
valor_str = regexp(filename1, '(\d{1,4})\.csv$', 'tokens', 'once');

% Converte para número (se encontrado)
if ~isempty(valor_str)
    idx_CAM = str2double(valor_str{1});
else
    idx_CAM = NaN; % Valor padrão se não encontrar
    warning('Nenhum número encontrado antes de .csv');
end

%Sinal do trigger do laser para identificação do instante de ativação NA
%ESP32
Flag_laser_ESP32 = data1.Var1(1:end-1);

% %Encontra a amostra da Flag do laser no ESP32 (trigger)
[~,idx_ESP32] = max(abs(diff(Flag_laser_ESP32)));   % `idx` is the sample **before** the jump

%Corrige a amostra do trigger da ESP32
idx_ESP32 = idx_ESP32 + 1; 

%Corrige a amostra do trigger da câmera
idx_CAM = idx_CAM + 1;

%Define a primeira amostrar a pegar da IMU da ESP32
amostraInicial = idx_ESP32   - idx_CAM;
amostraFinal =   idx_ESP32   - idx_CAM - 1 + length(Des_Z_cam);


%Mapeamento dos dados digitais de vel. angular e aceleração para graus/s e
%mm/s²

% SINAIS DE VELOCIDADE ANGULAR NOS 3 EIXOS (IMU)
%Detalhe, estou multiplicando o map_giro por um ganho para melhorar a
%estimação da correção. Verificar isso depois
map_giro = 1/2097.2 * gainIMU;
map_accel = 1/16384 * 9.81 * 1000 * gainIMU;
Gx  = -(data1.Var5(1:end-1))*map_giro;
Gy  = (data1.Var6(1:end-1))*map_giro;
Gz  = (data1.Var7(1:end-1))*map_giro;


% SINAIS DE ACELERAÇÃO NOS 3 EIXOS (IMU)
Az  =  -(data1.Var4(1:end-1))*map_accel;
Ay  =   (data1.Var3(1:end-1))*map_accel;
Ax  =   (data1.Var2(1:end-1))*map_accel;


%% Filtrando os sinais antes das integrações===============================
Gx_filt = filtfilt(b1,a1,Gx);
Gy_filt = filtfilt(b1,a1,Gy);
Gz_filt = filtfilt(b1,a1,Gz);

Ax_filt = filtfilt(b1,a1,Ax);
Ay_filt = filtfilt(b1,a1,Ay);
Az_filt = filtfilt(b1,a1,Az);



% %Pega o timestamp do sinal do giroscópio com base no trigger usando o laser
% Gx_frame = Gx_filt(amostraInicial:amostraFinal);
% Gy_frame = Gy_filt(amostraInicial:amostraFinal);
% Gz_frame = Gz_filt(amostraInicial:amostraFinal);
% 
% %Pega o timestamp do sinal do acelerometro com base no trigger usando o laser
% Ax_frame = Ax_filt(amostraInicial:amostraFinal);
% Ay_frame = Ay_filt(amostraInicial:amostraFinal);
% Az_frame = Az_filt(amostraInicial:amostraFinal);



%% ABAIXO É REALIZADA A PASSAGEM DE VELOCIDADE ANGULAR PARA DESLOCAMENTO 
%ANGULAR E DE ACELERAÇÃO LINEAR PARA DESLOCAMENTO LINEAR. TUDO É FEITO NO
%DOMINIO DA FREQUÊNCIA

% --- FFT complexa DA VEL. ANGULAR E ACELERAÇÃO- -------------------------
Gx_rad   = Gx_filt * pi/180;            % convert to rad s⁻¹ ; keep sign!
Gx_rad = Gx_rad';
Theta = processa_sinal_freq(Gx_rad, Fs, 'integrar');

%Multiplica o ângulo pela distância que a câmera está do alvo. O certo seria
%utilizar Tg(theta), mas para pequenos ângulos Tg(theta) ~ theta
Des_linear_projetado_Gx = dist_alvo * Theta;

Az_filt = Az_filt';
Des_linear_Az = processa_sinal_freq(Az_filt, Fs, 'integrar2');


% 2. Aplicação do filtro com fase nula nos sinais da IMU e da câmera
Des_Z_cam = Des_Z_cam';
Des_linear_projetado_Gx_filtrado_frame = (Des_linear_projetado_Gx(amostraInicial:amostraFinal));
Des_linear_Az_filtrado_frame = (Des_linear_Az(amostraInicial:amostraFinal));

%Calcula o deslocamento resultante em Z (correção do sinal de vibração da
%câmera)
Des_resultante_Z = filtfilt(b2,a2,Des_Z_cam)' - filtfilt(b2,a2,Des_linear_projetado_Gx_filtrado_frame)' - filtfilt(b2,a2,Des_linear_Az_filtrado_frame') ;

%% Avaliando sina no tempo=================================================

% Aqui é passado um filtro passa-altas para tirar a componente DC dos
% sinais



figure
plot(filtfilt(b2,a2,Des_Z_cam), 'k', 'LineWidth', 2); hold on
plot(filtfilt(b2,a2,Des_linear_projetado_Gx_filtrado_frame), '-.r', 'LineWidth', 2);
plot(filtfilt(b2,a2,Des_resultante_Z), 'b', 'LineWidth', 2);
plot(filtfilt(b2,a2,Des_linear_Az_filtrado_frame'), 'm','LineWidth', 2);


legend({'Câmera (mm)', 'Giro → Proj (mm)', 'Residual (mm)','Deslocamento em z (Accel)'}, ...
    'FontSize', 12, 'Location', 'best');

xlabel('Amostras', 'FontSize', 14);
ylabel('Deslocamento (mm)', 'FontSize', 14);
title('Câmera vs Deslocamento do Giro Projetado', 'FontSize', 16);

grid on
set(gca, 'FontSize', 12); % aumenta tamanho dos ticks dos eixos









%Tamanho do vetor a ser processado para tirar o espectro
vector_size = 2500;

%% Bins candidatos a correção
[mag_giro, freq_giro] = frequency_derivative_integration3(Des_linear_projetado_Gx_filtrado_frame(1:vector_size)', Fs, 'nada','true');
[mag_accel, freq_accel] = frequency_derivative_integration3(Des_linear_Az_filtrado_frame(1:vector_size)', Fs, 'nada','true');

figure
loglog(freq_giro, mag_giro, 'LineWidth', 1.5, 'DisplayName', 'Deslocamento causado pelo GIRO');
hold on
loglog(freq_accel, mag_accel, 'LineWidth', 1.5, 'DisplayName', 'Deslocamento medido pela câmera');
xlabel('Frequência (Hz)', 'FontSize', 12)
ylabel('Deslocamento (mm)', 'FontSize', 12)
title('Bins candidatos a correção', 'FontSize', 12)
grid on;



%% Espectro de deslocamento=================================================
[mag_compensado, freq_compensado] = frequency_derivative_integration3(Des_resultante_Z(1:vector_size)', Fs, 'nada','true');
[mag_camera, freq_camera] = frequency_derivative_integration3(Des_Z_cam(1:vector_size)', Fs, 'nada','true');
[mag_camera_parada, freq_camera_parada] = frequency_derivative_integration3(Des_Z_cam_base_parada(1:vector_size)', Fs, 'nada','true');

figure
loglog(freq_compensado, mag_compensado, 'LineWidth', 1.5, 'DisplayName', 'Deslocamento do alvo (compensado)');
hold on
loglog(freq_camera, mag_camera, 'LineWidth', 1.5, 'DisplayName', 'Deslocamento medido pela câmera');
loglog(freq_camera_parada, mag_camera_parada, 'LineWidth', 1.5, 'DisplayName', 'Deslocamento medido pela câmera (sem vibração)');
grid on

xlabel('Frequência (Hz)', 'FontSize', 12)
ylabel('Deslocamento (mm)', 'FontSize', 12)
title('Espectro de Deslocamento', 'FontSize', 12)

legend('Location', 'best')
legend boxon



% Espectro de velocidade==================================================
[mag_compensado, freq_compensado]       = frequency_derivative_integration3(Des_resultante_Z(1:vector_size)', Fs, 'derivar','true');
[mag_camera, freq_camera]               = frequency_derivative_integration3(Des_Z_cam(1:vector_size)', Fs, 'derivar','true');
[mag_camera_parada, freq_camera_parada] = frequency_derivative_integration3(Des_Z_cam_base_parada(1:vector_size)', Fs, 'derivar','true');

figure
loglog(freq_compensado, mag_compensado, 'LineWidth', 1.5, 'DisplayName', 'Velocidade do alvo (compensado)');
hold on
loglog(freq_camera, mag_camera, 'LineWidth', 1.5, 'DisplayName', 'Velocidade medida pela câmera');
loglog(freq_camera_parada, mag_camera_parada, 'LineWidth', 1.5, 'DisplayName', 'Velocidade medida pela câmera (sem vibração)');
grid on

xlabel('Frequência (Hz)', 'FontSize', 12)
ylabel('Velocidade (mm/s)', 'FontSize', 12)
title('Espectro de Velocidade', 'FontSize', 12)

legend('Location', 'best')
legend boxon

