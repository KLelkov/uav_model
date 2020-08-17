% Параметры подстройки
delta = 0.6;
alfa = 0.002;
% Вектор инаправления поиска коэффициентов
var = [0.7, 0.3, 0]; %Uk
% Случайное распределение начальных коэффициентов
params0 = [rand()*3, rand()*3, rand()*1];
% params0 = [2, 1.4, 0.95];
params = params0;
% Начальное значение ошибки
derivative0 = (simulate(params + delta*var) - simulate(params - delta*var)) / (2*delta);
derivative = derivative0;
% Количество шагов поиска коэффициентов
nSteps = 50;
for i = 1:nSteps
    % подстройка параметров
    params = params - alfa * derivative;
    % расчёт градиента
    derivative = (simulate(params + delta*var) - simulate(params - delta*var)) / (2*delta);
    if isnan(derivative)
        fprintf('Плохая выборка! Завершено всего %G итераций!\n', i)
        break;
    end
end
if i == nSteps
    fprintf('\nУСПЕХ!\n\n')
end
fprintf('Начальные значения коэффициентов:\n Kp = %G, Kd = %G, Ku = %G\n', params0(1), params0(2), params0(3))
fprintf('Коэффициенты после подбора за %d итераций:\n Kp = %G, Kd = %G, Ku = %G\n', i, params(1), params(2), params(3))