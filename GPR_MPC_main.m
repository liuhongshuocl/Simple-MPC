%% ����������� ����ѧģ�� ��˹���̻ع� ������·
function [sys,x0,str,ts] = GPR_MPC_main(t,x,u,flag)
switch flag
    case 0
        [sys,x0,str,ts] = mdlInitializeSizes; % Initialization
        
    case 2
        sys = mdlUpdates(t,x,u); % Update discrete states
        
    case 3
        sys = mdlOutputs(t,x,u); % Calculate outputs
        
    case {1,4,9} % Unused flags
        sys = [];
        
    otherwise
        error(['unhandled flag = ',num2str(flag)]); % Error handling
end
%% ��ʼ��
function [sys,x0,str,ts] = mdlInitializeSizes

sizes = simsizes;
sizes.NumContStates  = 0;  %����״̬�����ĸ���
sizes.NumDiscStates  = 3;  %��ɢ״̬�����ĸ���
sizes.NumOutputs     = 10;  %����ĸ���
sizes.NumInputs      = 7;  %����ĸ���
sizes.DirFeedthrough = 1;  % Matrix D is non-empty.
sizes.NumSampleTimes = 1;  %�Ƿ�ֱ�ӹ�ͨ����������ļ���й�
sys = simsizes(sizes);
x0 =zeros(sizes.NumDiscStates,1);
str = [];
ts  = [0.1 0];       % �������ں�mpc����ͳһ

%����ȫ�ֱ���
%��־λ
global gap_flag ;
gap_flag = 0;
global run_flag ;
run_flag = false;
%����״̬���� Carsim E-Class SUV
global VehiclePara;
VehiclePara.m   = 1860;  %������������������ 1590+120+150
VehiclePara.g   = 9.81;  %
VehiclePara.hCG = 0.72;  %���ĸ߶�
VehiclePara.Lf  = 1.18;  %ǰ�����
VehiclePara.Lr  = 1.77;  %�������
VehiclePara.L   = 2.95;  %���
VehiclePara.Tr  = 1.575; %�־�
VehiclePara.mu  = 0.75;  %����Ħ������
VehiclePara.Iz  = 2687.1;  %������Z���ת������
VehiclePara.Ix  = 894.4;  %������x���ת������
VehiclePara.Iy  = 2687.1;  %������y���ת������
VehiclePara.Radius = 0.39;  % ��̥�����뾶
%MPC����
global MPCParameters;
MPCParameters.Np  = 8;% ʱ��
MPCParameters.Ts  = 0.1; %��������
MPCParameters.Nx  = 3; %״̬��
MPCParameters.Ny  = 2; %�����
MPCParameters.Nu  = 1; %������
MPCParameters.lowest_length = 20;  %��СԤ�����
MPCParameters.model = 1;
MPCParameters.useGPonline = false;
MPCParameters.useParallel = true;
%����Ȩ��
global CostWeights;
CostWeights.Wephi   = 1;  %����ƫ��
CostWeights.Wey     = 1;  %����ƫ��
CostWeights.steer = 30;  %������
CostWeights.steer_rate = 0;  %�������仯��
%Լ��
global Constraints;
Constraints.dumax   = 0.1;  %��������Լ�� deg/s
Constraints.umax    = 0.5;  %������Լ�� deg
Constraints.ePhimax = 60*pi/180;  %�����ƫ��ļ���ֵ
Constraints.eymax   = 1.5;  %����ƫ��ļ���ֵ
Constraints.minsteer = -1;
Constraints.maxsteer = 1;
Constraints.amax = 4;

%�ο�·��  Carsim�е�ALT3
global Reftraj
Reftraj = load('shuangyixian.mat');
%��ʼ���������Բ�ƫ�նȺ��� ����ѧģ�Ͳ���
% [y, e] = func_RLSFilter_Calpha_f('initial', 0.95, 10, 10);
% [y, e] = func_RLSFilter_Calpha_r('initial', 0.95, 10, 10);

%% ������ɢ״̬��
function sys = mdlUpdates(t,x,u)
sys = x;

%% ���
function sys = mdlOutputs(t,x,u)

%********************************************һ.������ʼ��********************************************
global Reftraj;
global VehiclePara;
global MPCParameters;
global CostWeights;
global Constraints;
global gap_flag
global run_flag

Ctrl_frontwheelangle = 0;  %��������ǰ��ת��
PosX = 0;  %����ȫ���������µ�X����
PosY = 0;  %����ȫ���������µ�Y����
PosPhi = 0;  %���������
PosVel = 0;
e_phi = 0;  %����ƫ��
e_y = 0;  %����ƫ��
yawrate = 0;  %���ٶ�
Beta = 0;  %���Ĳ�ƫ��
fwa = 0;  %�۲�����ǰ��ƫ��
Next_Steer = 0; %������
spend_Time = 0;  %��ʱ
Predict_Path = [];

%����ǰ����������
if gap_flag < 2
    gap_flag = gap_flag + 1;
else % start control
    gap_flag = gap_flag + 1;
    %********************************************��.����״̬����********************************************
    t_Start = tic; %��ʼ��ʱ
    %���¹۲���
    VehState = func_StateObserver(u);
    PosPhi = VehState.phi;
    PosVel = VehState.Vel;
    PosX = VehState.X;
    PosY = VehState.Y ;
    %���¹����� ����ѧģ��ʹ��
    %ǰ��
    %     Arfa_f = (yawrate*VehiclePara.Lf/Vel - fwa);  %ǰ�ֲ�ƫ��  TODO����������Ӱ��
    %     [yf, Calpha_f1] = func_RLSFilter_Calpha_f(Arfa_f, Fyf);
    %     Caf = sum(Calpha_f1);  %ǰ�ֲ�ƫ�ն�
    %����
    %     Arfa_r = (yawrate*VehiclePara.Lr/Vel);  %���ֲ�ƫ��
    %     [yr, Calpha_r1] = func_RLSFilter_Calpha_r(Arfa_r, Fyr);
    %     CarHat = sum(Calpha_r1);  %���ֲ�ƫ�ն�
    %********************************************��.�ο��켣����********************************************
    [local_path_x,local_path_y] = func_wayPoint2VehicleCoords(VehState,Reftraj.WayPoints_Collect);
    %���ݲ����ж�Ԥ��·������
    total_t = MPCParameters.Np * MPCParameters.Ts * 1.2;
    s_limit = (PosVel + Constraints.amax * total_t / 2) * total_t;
    s_limit = max(MPCParameters.lowest_length,s_limit);
    %����ο��켣��Ϣ
    [total_s,segment_arc_s,predict_length] = func_CurveLength(local_path_x,local_path_y,s_limit);
    local_path_x = local_path_x(1:predict_length);
    local_path_y = local_path_y(1:predict_length);
    heading = func_CurveHeading(local_path_x,local_path_y);  %��������ϵ�²ο�·������
    curvature = func_CurveCurvature(local_path_x,local_path_y);  %�ο�·������
    %������ݳ����Ƿ�ƥ��
    if (length(local_path_x) ~= length(local_path_y)|| length(local_path_x) ~= length(heading)|| ...
            length(local_path_x) ~= length(curvature) || length(local_path_x) ~= length(total_s) || ...
            length(local_path_x) ~= length(segment_arc_s) )
        run_flag = false;
        warning("Data length mismatch");
        warning on ;
    end
    %����������
    [e_phi,e_y] = func_TrackingError(local_path_x,local_path_y);
    %********************************************��.����ģ�������********************************************
    %ѡ����ģ��
    if (run_flag)
        switch MPCParameters.model
            case 1
                [Next_Steer,Predict_Path] = ...
                    func_Simple_Kinematic_model(local_path_x,local_path_y,heading,curvature,total_s,segment_arc_s,VehState,t);  %�������˶�ѧģ��
            case 2
                [Next_Steer,Predict_Path] = ...
                    func_Simple_Dynamics_model(local_path_x,local_path_y,heading,curvature,total_s,segment_arc_s,VehState);  %�����Զ���ѧģ��
            case 3
                [Next_Steer,Predict_Path] = ...
                    func_GPR_Kinematic_model(local_path_x,local_path_y,heading,curvature,total_s,segment_arc_s,VehState);  %�����Ը�˹�����˶�ѧģ��
            case 4
                [Next_Steer,Predict_Path] = ...
                    func_GPR_Dynamics_model(local_path_x,local_path_y,heading,curvature,total_s,segment_arc_s,VehState);  %�����Ը�˹���̶���ѧģ��
        end
    else
        Next_Steer = 0;
    end
%     %�˲�
%     if Next_Steer < 0.001
%         Next_Steer = 0;
%     end 
Ctrl_frontwheelangle = atan(Next_Steer * VehiclePara.L)*180/pi;
spend_Time = toc(t_Start);  %����ÿ�ο�������ʱ��
end
    %********************************************��.���ݴ���********************************************
%ȫ�ֹ켣
figure(1)
plot(Reftraj.WayPoints_Collect(:,1),Reftraj.WayPoints_Collect(:,2),'k-')
hold on
%�����켣
figure(1)
plot(PosX,PosY,'bo')
hold on
pause(0.01);
%������ɫ��Ԥ��·��
global Refer_Path;
if length(Predict_Path) > 1
    figure(1)
    %����תȫ��
    Pre_deltax = (Predict_Path(:,1) - Predict_Path(:,2) * tan(PosPhi)) * cos(PosPhi);
    Pre_deltay = Predict_Path(:,2) / cos(PosPhi) + (Predict_Path(:,1) - Predict_Path(:,2) * tan(PosPhi)) * sin(PosPhi);
    Pre_x = Pre_deltax + PosX;
    Pre_y = Pre_deltay + PosY;
    plot(Pre_x,Pre_y,'-g','linewidth',3);
    hold on
end
% ������ɫ�Ĳο�·��
if length(Refer_Path) > 1
    figure(1)
    %����תȫ��
    Ref_deltax = (Refer_Path(:,1) - Refer_Path(:,2) * tan(PosPhi)) * cos(PosPhi);
    Ref_deltay = Refer_Path(:,2) / cos(PosPhi) + (Refer_Path(:,1) - Refer_Path(:,2) * tan(PosPhi)) * sin(PosPhi);
    Ref_x = Ref_deltax + PosX;
    Ref_y = Ref_deltay + PosY;
    plot(Ref_x,Ref_y,'-m','linewidth',1);
    hold off
end
sys = [Ctrl_frontwheelangle,e_y,e_phi,PosX,PosY,PosPhi,PosVel,Next_Steer,spend_Time,t];



























  
    
    