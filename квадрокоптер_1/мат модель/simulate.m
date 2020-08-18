function cost = simulate(params)
    %% ��������� �������������
    cost(1)=0;
    start_time = 0;
    end_time = 6;
    dt = 0.005; % ��� �������������
    nSim = (end_time - start_time)/ dt; % ���������� ����� ��� �������������
    times = start_time:dt:end_time;
    % ������� ��� ����� ����� � ���������� ��������
    Coords = zeros(nSim, 3);
    Speed = zeros(nSim, 3);
    Acceleration = zeros(nSim, 3);
    Angles = zeros(nSim, 3);
    Rates = zeros(nSim, 3);
    Time = zeros(nSim, 1);
    %% �������������� ���������
    m = 0.3;
    a = 0.245;%����� �������������
    width = 0.290;%������ �������������
    h = 0.055; %������ �������������
    S = zeros(3,3);%������� �������������
    S(1) = a*h;%Sx
    S(2) = width*h;%Sy
    S(3)= width*a;%Sz
    % L - ����� �� ������ ������������� �� ������
    L = sqrt((a/2)^2+(width/2)^2);
    Ix = 1/12*m*(3*L^3+h^2);
    Iy = Ix;
    Iz = m*L^2/2;
    % I - ������� �������, �������� ���
    I =[Ix 0 0;0 Iy 0; 0 0 Iz];
    %% ���������� ��������
    g = 9.81;
    r0 =1.2;%��������� �������
    %�d - ����������� ���������������� ����
    cd =0.4;
    %b - ����������� ��������� ������� �������. k - ��������� �����������
    b =0.7426*10^-6;
    k = 1.4851*10^-5;
    % ����������� �������� �������� �������� ������ ��� ��������� �� �����
    W_hover = 49257.0198; % ������� ������� ��������
    %% ������������� �����
    % Simulate some disturbance in the angular velocity.
    % The magnitude of the deviation is in radians / second.
    % deviation = 100;
    % thetadot = deg2rad(2 * deviation * rand(3, 1) - deviation);

     %% ��������� �������
    x = [0; 0; 10]; % X Y Z
    xdot = [0; 0; 0]; % Vx Vy Vz
    theta = [5; 0; 0]; % ���� ������ ����
    thetadot = [0;0;0];
    theta_need =[0;0;0];% �������� ����, ������,����
    thetadot_need=[0;0;0];%�������� ������� ��������
    Dthetadot =[0;0;0]; 
    dW = 50; % ��������� ������� �������� ��� �������� ��������� ���
     %������������ �� ����������
     Kd =0;
     Kp =1;
    % �������� �������� ������
    %i = [W_hover+dW, W_hover+dW, W_hover+dW, W_hover+dW];
    %% �������� ����
    for j = 1:nSim
        % ������ ������� ��������� � ��������� (body) ��
         omega = thetadot2omega(thetadot, theta);
        i = controller(theta_need,theta,thetadot_need,thetadot,m,g,k,b,I,L,Dthetadot, params);
        % ������ �������� ���������
        a = acceleration(i, theta, xdot, m, g, k, cd,S,r0);
        % ������ ������� ���������
        omegadot = angular_acceleration(i, omega, I, L, b, k);
        Dthetadot = omega2thetadot(omegadot, theta);
        % �������������� ������� ���������
        omega = omega + dt * omegadot;
        % ������ ������� ��������� � ������������ (inertial) ��
        thetadot = omega2thetadot(omega, theta);
        % ������ ����� ����������
        theta = theta + dt * thetadot;
        % ������ ��������� ��������
        xdot = xdot + dt * a;
        % ������ ���������
        x = x + dt * xdot;
        % ���� ����� ��� ��������
        Coords(j,:) = x(:,1);
        Speed(j,:) = xdot(:,1);
        Acceleration(j,:) = a(:,1);
        Angles(j,:) = theta;
        Rates(j,:) = thetadot;
        Time(j) = j*dt; 
    % ������� ��������� (������) ��� ������������� ������� �������������
    % ����������
    cost(j+1) =cost(j)+ 1/(end_time - start_time) * sum((theta_need - theta).^2) * dt;
        if isnan(cost(j+1))
            cost = NaN;
            return
        end
    end
    cost=cost(j+1);
end


