function Rmes = func_cau_rmes(a)
biaozhuncha = std(a)
b = length(a)
Rmes = sqrt(sum((a-biaozhuncha).^2)/b)