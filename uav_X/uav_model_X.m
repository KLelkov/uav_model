function uav_state = uav_model_X(uav_state, controls, dt)
    %% ������� ���������
    % ���������� � ������������ ��
    x = uav_state.coords(1); y = uav_state.coords(2); z = uav_state.coords(3);
    % ���� ����������
    roll = uav_state.roll; pitch = uav_state.pitch; yaw = uav_state.yaw;
    % �������� �������� � ��������� ��
    Vbx = uav_state.Vb(1); Vby = uav_state.Vb(2); Vbz = uav_state.Vb(3);
    % ������� �������� � ��������� ��
    Wbx = uav_state.Wb(1); Wby = uav_state.Wb(2); Wbz = uav_state.Wb(3);
    W1 = controls(1); W2 = controls(2);
    W3 = controls(3); W4 = controls(4);
    
    %% �������������� ���������
    a = 0.36; % ����� ������������� � ������
    k = 1.4851*10^-4; % k - ��������� ����������
    h = 0.07; % ������ ������������
    L1 = 485/2/1000; % ���������� �� ����� ������������� �� �����
    % ����� ����� ������������� ����� �, ����� ������� �� ����� ����� ����� L
    l = 0.36/2;
    m = 0.7; % ����� �������������
    g = 9.81; % ��������� ���������� �������
    Ix = 1/12*m*(3*L1^3+h^2); % ������ �������
    Iz = Ix; % ������ �������
    Iy = m*L1^2/2; % ������ ������� 
    Irot = 0; % ������ ������� ������
    Sx = a*h; Sz = a*h; Sy = a*a; % ������� ������������ �������������
    cd = 0.1; % ���������������� �����������
    md = 0.00004; % ���������������� �����������
    ro = 1.2; % ��������� �������
    n = 105/2;
    b = 0.0002; % ����������� ����������� �������
    
    %% �������� ��������
    % �������� ���� ������� �� ��� ��������� ��
    Fgb = [-m*g*sind(pitch);
           -m*g*cosd(pitch)*cosd(roll);
           m*g*cosd(pitch)*sind(roll)];
    % �������� ���� ������� �� ��� ��������� ��
    Fpb = [0;
           k*(W1^2+W2^2+W3^2+W4^2);
           0];
    % �������� ��� ����������������� ������������� �� ��� ��������� ��
    Fab = [ -sign(Vbx)*cd*ro*Vbx^2*Sx/2;
            -sign(Vby)*cd*ro*Vby^2*Sy/2;
            -sign(Vbz)*cd*ro*Vbz^2*Sz/2];
    % ��������� ��������� � ��������� ��
    Acor = 2*[Wby*Vbz - Wbz*Vby;
              Wbz*Vbx - Wbx*Vbz;
              Wbx*Vby - Wby*Vbx];
    % ��������� ��������� �������� ���
    dVb = (Fgb + Fpb + Fab)/m - Acor;
    
    %% ������������ ��������
    % ������� �� ���� ������� � ��������� ��
    Mpb = [k*l*(W1^2 + W2^2 - W3^2 - W4^2);
           b*(W1^2 + W3^2 - W2^2 - W4^2);
           k*l*(W1^2 + W4^2 - W2^2 - W3^2)];
    % ������� ���������������� ��� � ��������� ��
    Mab = [ sign(Vbx)*md*ro*Vbx^2*Sx*n/2;
            sign(Vby)*md*ro*Vby^2*Sy*n/2;
            sign(Vbz)*md*ro*Vbz^2*Sz*n/2];
    % ��������������� �������, ��������� ����������� ��������� ���
    Mgyrb = [Wby*Wbz*(Iy - Iz);
             Wbx*Wbz*(Ix - Iz);
             Wbx*Wby*(Iy - Ix);];
    % ��������������� ������� ��������� ��������� ������� ���
    Mgpb = [Irot*Wbz*(W2 + W4 - W1 - W3);
            0;
            Irot*Wbx*(W1 + W3 - W2 - W4);];
    % ��������� �������� �������� ���
    dWb = (Mpb + Mab + Mgyrb + Mgpb) ./ [Ix; Iy; Iz];
    
    %% ������ �������� � ������� ��������� � ��������� �� ���� �������������
    Vbx = Vbx + dVb(1)*dt;
    Vby = Vby + dVb(2)*dt;
    Vbz = Vbz + dVb(3)*dt;
    
    Wbx = Wbx + dWb(1)*dt;
    Wby = Wby + dWb(2)*dt;
    Wbz = Wbz + dWb(3)*dt;
    
    %% ������ ������������� ���������� � ������������ ��
    % ������� �������� �� ��������� �� � ������������ ��
    Rbg = [cosd(yaw)*cosd(pitch), -cosd(yaw)*sind(pitch)*cosd(roll)+sind(yaw)*sind(roll), cosd(yaw)*sind(pitch)*sind(roll)+sind(yaw)*cosd(roll);
        sind(pitch), cosd(pitch)*cosd(roll), -cosd(pitch)*sind(roll);
        -sind(yaw)*cosd(pitch), cosd(yaw)*sind(roll)+sind(yaw)*sind(pitch)*cosd(roll), cosd(yaw)*cosd(roll)-sind(yaw)*sind(pitch)*sind(roll)];
    % �������� �������� � ������������ ��
    V = Rbg * [Vbx; Vby; Vbz];
    % �������� �������� � ������������ ��
    W = Rbg * [Wbx; Wby; Wbz];
    % ������� �������� � ������������ ��
    droll = Wbx - (Wby*cosd(roll) - Wbz*sind(roll)) * tand(pitch);
    dpitch = Wbz*cosd(roll) + Wby*sind(roll);
    dyaw = (Wby*cosd(roll) - Wbz*sind(roll)) / cosd(pitch);
    
    % ������������� ���������
    x = x + V(1)*dt;
    y = y + V(2)*dt;
    z = z + V(3)*dt;
    roll = roll + rad2deg(droll*dt);
    pitch = pitch + rad2deg(dpitch*dt);
    yaw = yaw + rad2deg(dyaw*dt);
    
    %% ������������ �������� ���������
    % ���������� � ������������ ��
    uav_state.coords = [x; y; z];
    % ���� ����������
    uav_state.roll = roll;
    uav_state.pitch = pitch;
    uav_state.yaw = yaw; 
    % �������� �������� � ��������� ��
    uav_state.Vb = [Vbx; Vby; Vbz];
    % ������� �������� � ��������� ��
    uav_state.Wb = [Wbx; Wby; Wbz];
    % �������� �������� � ������������ ��
    uav_state.V = V;
    % ������� �������� � ������������ ��
    uav_state.W = W;
    uav_state.droll = droll;
    uav_state.dpitch = dpitch;
    uav_state.dyaw = dyaw;
    
end

