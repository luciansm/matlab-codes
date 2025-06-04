function resultado = angular_para_linear(valor_angular, tipo_entrada, d, f, tipo_saida)
% ANGULAR_PARA_LINEAR - Converte deslocamento ou velocidade angular para linear
%
% Entradas:
%   valor_angular - valor de entrada angular (em graus ou graus/s)
%   tipo_entrada  - 'deslocamento' ou 'velocidade'
%   d             - distância ao alvo (em metros)
%   f             - frequência (Hz) do movimento
%   tipo_saida    - 'deslocamento' ou 'velocidade'
%
% Saída:
%   resultado     - valor linear convertido (em mm ou mm/s)
%
% Autor: Lucian Ribeiro da Silva

    % Converter entrada para radianos ou rad/s
    switch lower(tipo_entrada)
        case 'deslocamento'
            theta = deg2rad(valor_angular); % rad
        case 'velocidade'
            omega = deg2rad(valor_angular); % rad/s
        otherwise
            error('Tipo de entrada inválido: use "deslocamento" ou "velocidade".')
    end

    % Cálculos
    switch lower(tipo_saida)
        case 'deslocamento'
            if ~exist('theta','var')
                % Se entrou com velocidade, converte para deslocamento angular
                theta = omega / (2 * pi * f);
            end
            resultado = d * theta * 1000; % deslocamento linear (mm)
        case 'velocidade'
            if ~exist('omega','var')
                omega = 2 * pi * f * theta;
            end
            resultado = d * omega * 1000; % velocidade linear (mm/s)
        otherwise
            error('Tipo de saída inválido: use "deslocamento" ou "velocidade".')
    end
end
