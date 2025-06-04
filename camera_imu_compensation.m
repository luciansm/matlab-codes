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
dist_alvo = 3990 * 0.8741; %em mm 
Fs = 1000;

%Dados de VELOCIDADE ANGULAR (mgraus/s -> graus/s)
map_giro = 1/16.4;
map_accel = 1/16384 * 9.81 * 1000;
Gx  = -(data1.Var5(1:end-1))*map_giro;
Gy  = (data1.Var6(1:end-1))*map_giro;
Gz  = (data1.Var7(1:end-1))*map_giro;

Az  =   (data1.Var4(1:end-1))*map_accel;
% Gy  = detrend(data1.Var2(1:end-1))/1000;
% Gz  = detrend(data1.Var3(1:end-1))/1000;
Flag_laser_arduino = data1.Var1(1:end-1);


% 
% %Encontra a amostra da Flag do laser no arduino (trigger)
[~,idx_Arduino] = max(abs(diff(Flag_laser_arduino)));   % `idx` is the sample **before** the jump


% 
% %% =====================================================================%%
% 
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
Flag_laser_cam = data2.Var8;

if (rms(Displacement_Z_cam) > 100)
    Displacement_Z_cam = Displacement_Z_cam / 1000; %passa para mm se estiver em microns
end



% %Encontra a amostra da Flag do laser na câmera
[~,idx_CAM] = max(abs(diff(Flag_laser_cam)));   % `idx` is the sample **before** the jump
%mesa_10hz e hexapod parado = 1297 (original 1298)
%mesa_15hz e hexapod parado = 825 (original 826)
%mesa_20hz e hexapod parado = 908 (original 925)
%mesa_10hz e hexapod 10hz = 810 (original 810)
%mesa_15hz e hexapod 15Hz = 803 (original 804)
%mesa_20hz e hexapod 20hz = 814 (original 814)
idx_CAM = 919;



%Pega o timestamp do sinal de deslocamento da câmera com base no trigger
%usando o laser
Displacement_Z_cam_frame = (Displacement_Z_cam(idx_CAM:end));

%Pega o timestamp do sinal do giroscópio com base no trigger usando o laser
Gx_frame = Gx(idx_Arduino:idx_Arduino + length(Displacement_Z_cam_frame)-1);



%Pega o timestamp do sinal do acelerometro com base no trigger usando o laser
Az_frame = Az(idx_Arduino:idx_Arduino + length(Displacement_Z_cam_frame)-1);


%% --- FFT complexa do giro -------------------------
Gx_rad   = Gx * pi/180;            % convert to rad s⁻¹ ; keep sign!
Gx_rad = Gx_rad';

N   = numel(Gx_rad);
X   = fft(Gx_rad);                       % FFT de 2 lados (considera parte complexa)
k   = 0:N-1;
f   = (k - (k>=N/2)*N) * Fs/N;           % frequências -Fs/2 até +Fs/2
omega = 2*pi*f;                          % now ω carries the ± sign
omega(omega==0) = Inf;                   % avoid ÷0 at DC

ThetaSpec =  X ./ (1j*omega);            % Integração na frequência
ThetaSpec(omega==Inf) = 0;               % kill the DC column cleanly
DgSpec   = dist_alvo * ThetaSpec;          % project to linear disp.

d_g_t = real(ifft(DgSpec))';              % perfectly scaled, no extra ×2
% d_g_t = detrend(d_g_t)';                  % optional bias removal

% 1. Coeficientes do filtro Butterworth passa-alta
Fc = 4;
[Wn] = Fc/(Fs/2);   % Normalização da frequência (Nyquist)
[b, a] = butter(1, Wn, 'high');

% 2. Aplicação do filtro com fase nula
d_g_t_filtrado = filtfilt(b, a, d_g_t);
d_g_t_frame = d_g_t(idx_Arduino:idx_Arduino + length(Displacement_Z_cam_frame)-1);
Displacement_Z_cam_filtrado = filtfilt(b, a, Displacement_Z_cam);
Displacement_Z_cam_filtrado_frame = Displacement_Z_cam_filtrado(idx_CAM:end);
d_g_t_filtrado_frame = (d_g_t_filtrado(idx_Arduino:idx_Arduino + length(Displacement_Z_cam_frame)-1));

% Carregar ou definir os vetores
A = (Displacement_Z_cam_filtrado_frame);
B = (d_g_t_filtrado_frame);
% B = (d_g_t_filtrado_frame);

% Calcular correlação cruzada normalizada
[corr_values, lags] = xcorr(A, B, 'coeff');

[~, idx] = max((corr_values)); % Usar valor absoluto para considerar inversões de fase
optimal_lag = lags(idx);

if abs(optimal_lag) > 3

    optimal_lag = 0;
end


if optimal_lag ~= 0
    % B está atrasado: remover as primeiras `optimal_lag` amostras de B
    A_aligned = detrend(Displacement_Z_cam_filtrado(idx_CAM+optimal_lag:end));
    B_aligned = detrend(d_g_t_filtrado(idx_Arduino:idx_Arduino + length(A_aligned)-1));
else
    A_aligned = A;
    B_aligned = B;
end




desloc_resultante_aux = A_aligned - B_aligned;


%% ------------------------- visualisation --------------------------------
figure
plot(A_aligned, 'k', 'LineWidth', 2); hold on
plot(B_aligned, '-.r', 'LineWidth', 2);
plot(desloc_resultante_aux, 'b', 'LineWidth', 2);

legend({'Câmera (mm)', 'Giro → Proj (mm)', 'Residual (mm)'}, ...
       'FontSize', 12, 'Location', 'best');

xlabel('Amostras', 'FontSize', 14);
ylabel('Deslocamento (mm)', 'FontSize', 14);
title('Câmera vs Deslocamento do Giro Projetado', 'FontSize', 16);

grid on
set(gca, 'FontSize', 12); % aumenta tamanho dos ticks dos eixos


fftf_media(desloc_resultante_aux, Fs, 'acel', round(length(desloc_resultante_aux)/1));

function [data] = readAndProcessCSV(fullpath)
    data = readtable(fullpath);
end
