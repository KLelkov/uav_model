% ��������� ����������
delta = 0.002;
alfa = 0.001;
% ������ ������������ ������ �������������
var = [0.7, 0.3, 0]; %Uk
% ��������� ������������� ��������� �������������
params0 = [rand()*5, rand()*3, rand()*1.5];
% params0 = [2, 1.4, 0.95];
params = params0;
% ��������� �������� ������
derivative0 = (simulate(params + delta*var) - simulate(params - delta*var)) / (2*delta);
derivative = derivative0;
% ���������� ����� ������ �������������
nSteps = 250;
graph_log = zeros(nSteps, 1);
for i = 1:nSteps
    % ���������� ����������
    params = params - alfa * derivative;
    % ������ ���������
    derivative = (simulate(params + delta*var) - simulate(params - delta*var)) / (2*delta);
    graph_log(i) = simulate(params);
    if isnan(derivative)
        fprintf('������ �������! ��������� ����� %G ��������!\n', i)
        break;
    end
end
if i == nSteps
    fprintf('\n�����!\n\n')
end
fprintf('��������� �������� �������������:\n Kp = %G, Kd = %G, Ku = %G\n', params0(1), params0(2), params0(3))
fprintf('������������ ����� ������� �� %d ��������:\n Kp = %G, Kd = %G, Ku = %G\n', i, params(1), params(2), params(3))

figure
plot(graph_log(1:i), 'b', 'LineWidth', 2);