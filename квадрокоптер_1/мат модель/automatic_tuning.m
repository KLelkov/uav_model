% ��������� ����������
delta = 0.6;
alfa = 0.002;
% ������ ������������ ������ �������������
var = [0.7, 0.3, 0]; %Uk
% ��������� ������������� ��������� �������������
params0 = [rand()*3, rand()*3, rand()*1];
% params0 = [2, 1.4, 0.95];
params = params0;
% ��������� �������� ������
derivative0 = (simulate(params + delta*var) - simulate(params - delta*var)) / (2*delta);
derivative = derivative0;
% ���������� ����� ������ �������������
nSteps = 50;
for i = 1:nSteps
    % ���������� ����������
    params = params - alfa * derivative;
    % ������ ���������
    derivative = (simulate(params + delta*var) - simulate(params - delta*var)) / (2*delta);
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