function resultado = conversor_harmonico(amplitude, tipo_entrada, f, tipo_saida)
% CONVERSOR_HARMONICO - Converte entre deslocamento, velocidade e aceleração
% máximos em sinais harmônicos com base na frequência.
%
% Sintaxe:
%   resultado = conversor_harmonico(amplitude, tipo_entrada, f, tipo_saida)
%
% Entradas:
%   amplitude     - valor da amplitude conhecida (em m, m/s ou m/s²)
%   tipo_entrada  - 'deslocamento', 'velocidade' ou 'aceleracao'
%   f             - frequência (Hz)
%   tipo_saida    - 'deslocamento', 'velocidade' ou 'aceleracao'
%
% Saída:
%   resultado     - valor convertido na unidade correspondente
%
% Autor: Lucian Ribeiro da Silva

    omega = 2 * pi * f;

    switch lower(tipo_entrada)
        case 'deslocamento'
            A = amplitude;
            v = omega * A;
            a = omega^2 * A;
        case 'velocidade'
            v = amplitude;
            A = v / omega;
            a = omega * v;
        case 'aceleracao'
            a = amplitude;
            A = a / (omega^2);
            v = a / omega;
        otherwise
            error('Tipo de entrada inválido. Use: deslocamento, velocidade ou aceleracao.')
    end

    switch lower(tipo_saida)
        case 'deslocamento'
            resultado = A;
        case 'velocidade'
            resultado = v;
        case 'aceleracao'
            resultado = a;
        otherwise
            error('Tipo de saída inválido. Use: deslocamento, velocidade ou aceleracao.')
    end
end
