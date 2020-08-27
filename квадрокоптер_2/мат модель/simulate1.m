function [cost,x,xdot,theta,thetadot] = simulate1(params,j,theta,thetadot,Dthetadot,theta_need,thetadot_need,x,xdot,dt,end_time,start_time)
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
        Time(j) = j*dt; % ������ ������� ��������� � ��������� (body) ��
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
    cost = 1/(end_time - start_time) * sum((theta_need - theta).^2) * dt;
    
end