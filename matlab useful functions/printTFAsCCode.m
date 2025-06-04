function printTFAsCCode(discreteTF, tfName)
    % Função para gerar código C com os coeficientes da TF discreta
    % Entrada:
    %  - discreteTF: função de transferência discreta (objeto tf)
    %  - tfName: nome da função de transferência para usar no código C

    % Obter numerador e denominador
    [num, den] = tfdata(discreteTF, 'v');
    
    % Inverter ordem dos coeficientes
    numFlipped = flip(num);
    denFlipped = flip(den);
    
    % Impressão do numerador no formato C
    fprintf('    // Atribuições para os coeficientes do numerador\n');
    for i = 1:length(numFlipped)
        fprintf('    num_%s[%d] = %.10f;\n', tfName, i-1, numFlipped(i));
    end
    fprintf('\n');
    
    % Impressão do denominador no formato C
    fprintf('    // Atribuições para os coeficientes do denominador\n');
    for i = 1:length(denFlipped)
        fprintf('    den_%s[%d] = %.10f;\n', tfName, i-1, denFlipped(i));
    end
    fprintf('\n');
end
