function VehState = func_StateObserver(CarsimOutput)
% �۲⳵��״̬��������λС��
    VehState.X = round(CarsimOutput(1),2);  %ȫ������Xλ��
    VehState.Y = round(CarsimOutput(2),2);  %ȫ������Yλ��
    VehState.phi = round(CarsimOutput(3),2)*pi/180;  %�����
    VehState.Vel = round(CarsimOutput(4),2)/3.6;  %�ٶ�
    VehState.yawrate = round(CarsimOutput(5),2);  %�������ٶ�
    VehState.fwa = round(0.5*(CarsimOutput(6)+CarsimOutput(7)),2);  %ǰ��ƫת��