clc; clear; close all;

%Leitura dos dados do Labview (acelerômetro IEPE na tubulação e flag de ativação do laser)
[filename, pathname] = uigetfile({'*.txt', 'Arquivos CSV/TXT (*.csv)'; '*.*', 'Todos os arquivos (*.*)'}, 'Selecione o arquivo');

% Verifica se o usuário cancelou a seleção de arquivo
if isequal(filename, 0)
    disp('Seleção de arquivo cancelada');
    return;
end

% Caminho completo do arquivo
fullpath = fullfile(pathname, filename);

% Lê e processa os dados do arquivo CSV
[data1] = readAndProcessCSV(fullpath);

%Distância ao alvo
dist_alvo = 6620 * 1; %em mm
Fs = 1000;

%Dados de VELOCIDADE ANGULAR (mgraus/s -> graus/s)
map_giro = 1/2097.2 * 0.8;
map_accel = 1/16384 * 9.81 * 1000;
Gx  = -(data1.Var5(1:end-1))*map_giro;
Gy  = (data1.Var6(1:end-1))*map_giro;
Gz  = (data1.Var7(1:end-1))*map_giro;

Az  =   -(data1.Var4(1:end-1))*map_accel;
% Gy  = detrend(data1.Var2(1:end-1))/1000;
% Gz  = detrend(data1.Var3(1:end-1))/1000;
Flag_laser_arduino = data1.Var1(1:end-1);


%
% %Encontra a amostra da Flag do laser no arduino (trigger)
[~,idx_Arduino] = max(abs(diff(Flag_laser_arduino)));   % `idx` is the sample **before** the jump
% idx_Arduino = idx_Arduino + 1;
idx_Arduino = idx_Arduino + 1;


% %Leitura dos dados da câmera
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
[data2] = readAndProcessCSV(fullpath);
Displacement_Z_cam = (data2.Var5);
% Flag_laser_cam = data2.Var8;

% if (rms(Displacement_Z_cam) > 100)
Displacement_Z_cam = Displacement_Z_cam / 1000; %passa para mm se estiver em microns
% end

%Extrai o valor do trigger do nome do arquivo
% Extrai o número usando expressão regular
valor_str = regexp(filename, '(\d{1,4})\.csv$', 'tokens', 'once');

% Converte para número (se encontrado)
if ~isempty(valor_str)
    idx_CAM = str2double(valor_str{1});
else
    idx_CAM = NaN; % Valor padrão se não encontrar
    warning('Nenhum número encontrado antes de .csv');
end

disp(idx_CAM);



% %Encontra a amostra da Flag do laser na câmera
idx_CAM = idx_CAM + 1;

%Pega o timestamp do sinal de deslocamento da câmera com base no trigger
%usando o laser
Displacement_Z_cam_frame = (Displacement_Z_cam(idx_CAM:end));

%Pega o timestamp do sinal do giroscópio com base no trigger usando o laser
Gx_frame = Gx(idx_Arduino:idx_Arduino + length(Displacement_Z_cam_frame)-1);



%Pega o timestamp do sinal do acelerometro com base no trigger usando o laser
Az_frame = Az(idx_Arduino:idx_Arduino + length(Displacement_Z_cam_frame)-1);


%% --- FFT complexa do Gx e Az -------------------------
Gx_rad   = Gx * pi/180;            % convert to rad s⁻¹ ; keep sign!
Gx_rad = Gx_rad';
Theta = processa_sinal_freq(Gx_rad, 1000, 'integrar');
d_g_t = dist_alvo * Theta;

Az = Az';
Dz_acel = processa_sinal_freq(Az, 1000, 'integrar2');


%Filtro passa-banda entre 4 Hz e 120 Hz
Fc1 = 4;     % frequência de corte inferior (Hz)
Fc2 = 20;   % frequência de corte superior (Hz)
Wn = [Fc1 Fc2]/(Fs/2);  % Normaliza para Nyquist
[b, a] = butter(1, Wn, 'bandpass');  % Filtro de 2ª ordem passa-banda

% 2. Aplicação do filtro com fase nula
d_g_t_filtrado = filtfilt(b, a, d_g_t);
Dz_acel_filtrado = filtfilt(b, a, Dz_acel);


d_g_t_frame = d_g_t(idx_Arduino:idx_Arduino + length(Displacement_Z_cam_frame)-1);
Displacement_Z_cam_filtrado = filtfilt(b, a, Displacement_Z_cam);
Displacement_Z_cam_filtrado_frame = Displacement_Z_cam_filtrado(idx_CAM:end)';
d_g_t_filtrado_frame = (d_g_t_filtrado(idx_Arduino:idx_Arduino + length(Displacement_Z_cam_frame)-1));
Dz_acel_filtrado_frame = (Dz_acel_filtrado(idx_Arduino:idx_Arduino + length(Displacement_Z_cam_frame)-1));


% Carregar ou definir os vetores
A = (Displacement_Z_cam_filtrado_frame);
B = (d_g_t_filtrado_frame);
C = (Dz_acel_filtrado_frame);
A_aligned = A;
B_aligned = B;
C_aligned = C;
% B_aligned = 0


desloc_resultante_aux = A_aligned' - B_aligned' - C_aligned' ;

%% ------------------------- visualisation --------------------------------
figure
plot(A_aligned, 'k', 'LineWidth', 2); hold on
plot(B_aligned, '-.r', 'LineWidth', 2);
plot(desloc_resultante_aux, 'b', 'LineWidth', 2);
plot(C_aligned', 'm','LineWidth', 2);


legend({'Câmera (mm)', 'Giro → Proj (mm)', 'Residual (mm)','Deslocamento em z (Accel)'}, ...
    'FontSize', 12, 'Location', 'best');

xlabel('Amostras', 'FontSize', 14);
ylabel('Deslocamento (mm)', 'FontSize', 14);
title('Câmera vs Deslocamento do Giro Projetado', 'FontSize', 16);

grid on
set(gca, 'FontSize', 12); % aumenta tamanho dos ticks dos eixos








% %Leitura dos dados da câmera SEM VIBRAÇÃO NA BASE
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
[data3] = readAndProcessCSV(fullpath);
Displacement_Z_cam_base_parada = (data3.Var5);
Displacement_Z_cam_base_parada = Displacement_Z_cam_base_parada / 1000; %passa para mm se estiver em microns
Displacement_Z_cam_base_parada_filtrada = filtfilt(b, a, Displacement_Z_cam_base_parada);
Displacement_Z_cam_base_parada_filtrada_frame  = Displacement_Z_cam_base_parada_filtrada(idx_CAM:end);

% Espectro de deslocamento
[mag_compensado, freq_compensado] = fftf_sem_grafico(desloc_resultante_aux(1:3000)', 1000, 'acel');
[mag_camera, freq_camera] = fftf_sem_grafico(Displacement_Z_cam_filtrado_frame(1:3000)', 1000, 'acel');
[mag_camera_parada, freq_camera_parada] = fftf_sem_grafico(Displacement_Z_cam_base_parada_filtrada_frame(1:3000)', 1000, 'acel');

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




% Espectro de velocidade
[mag_compensado, freq_compensado]       = frequency_derivative_integration(desloc_resultante_aux(1:3000), 1000, 'derivar');
[mag_camera, freq_camera]               = frequency_derivative_integration(Displacement_Z_cam_filtrado_frame(1:3000), 1000, 'derivar');
[mag_camera_parada, freq_camera_parada] = frequency_derivative_integration(Displacement_Z_cam_base_parada_filtrada_frame(1:3000), 1000, 'derivar');

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



% fftf_com_grafico(desloc_resultante_aux(1:2000)', 1000, 'acel')
% fftf_media(desloc_resultante_aux, Fs, 'acel', round(length(desloc_resultante_aux)/1));

function [data] = readAndProcessCSV(fullpath)
data = readtable(fullpath);
end
