delta = 0.1;
alfa = 0.5;
var = [1, 0, 0]; %Uk
params = [rand()*5, rand()*5, rand()*1];
% params0 = [2, 1.4, 0.95];
params = params0;
derivative0 = (simulate(params + delta*var) - simulate(params - delta*var)) / (2*delta);
derivative = derivative0;
nSteps = 50;
for i = 1:nSteps
    params = params - alfa * derivative;
    derivative = (simulate(params + delta*var) - simulate(params - delta*var)) / (2*delta);
end
params