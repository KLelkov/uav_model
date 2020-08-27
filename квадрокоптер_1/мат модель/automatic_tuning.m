% Параметры подстройки
delta = 0.2;
alfa = 0.05;
% Вектор инаправления поиска коэффициентов
var = [0.49, 0.49, 0.02]; %Uk
% Случайное распределение начальных коэффициентов
params0 = [rand()*5, rand()*3, rand()*0.2];
% params0 = [4.84, 2.17, 0.03];
params = params0;
% Начальное значение ошибки
derivative0 = (simulate(params + delta*var) - simulate(params - delta*var)) / (2*delta);
derivative = derivative0;
% Количество шагов поиска коэффициентов
nSteps = 350;
graph_log = zeros(nSteps, 1);
for i = 1:nSteps
    % подстройка параметров
    params = params - alfa * derivative*var;
    % расчёт градиента
    derivative = (simulate(params + delta*var) - simulate(params - delta*var)) / (2*delta);
    graph_log(i) = simulate(params);
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

figure
plot(graph_log(1:i), 'b', 'LineWidth', 2);