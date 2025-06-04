function corrected_data = removecomma(data)

corrected_data = str2double(strrep(data, ',', '.'));