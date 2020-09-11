clc
%% Параметры подстройки
delta = 0.8;
alfa = 1.2;
% Вектор инаправления поиска коэффициентов
var = [0.7, 0.3, 0]; %Uk
% Случайное распределение начальных коэффициентов
Kp0 = rand()*3;
Kd0 =rand()*3;
Ku0=rand()*2;
% params0 = [2, 1.4, 0.95];
Kp=Kp0; Kd=Kd0;Ku=Ku0;
% Начальное значение ошибки
derivative0 = (Real_cvadro_model1(Kp + delta*var(1),Kd + delta*var(2),Ku + delta*var(3))-Real_cvadro_model1(Kp - delta*var(1),Kd - delta*var(2),Ku - delta*var(3)))/(2*delta);
derivative = derivative0;
% Количество шагов поиска коэффициентов
nSteps = 50;
for i = 2:nSteps
    % подстройка параметров
     Kp = Kp - alfa * derivative;
     Kd= Kd -  alfa * derivative;
     Ku =Ku -  alfa * derivative;
     %расчет основной программы
     cost = Real_cvadro_model1(Kp,Kd,Ku)
    % расчёт градиента
    derivative = (Real_cvadro_model1(Kp + delta*var(1),Kd + delta*var(2),Ku + delta*var(3))-Real_cvadro_model1(Kp - delta*var(1),Kd - delta*var(2),Ku - delta*var(3)))/(2*delta);
end
fprintf('Начальные значения коэффициентов:\n Kp = %G, Kd = %G, Ku = %G\n', Kp0, Kd0, Ku0)
fprintf('Коэффициенты после подбора за %d итераций:\n Kp = %G, Kd = %G, Ku = %G\n', nSteps, Kp, Kd, Ku)
