%===============================================================
% MPCģ���Ŀ�꺯��
%===============================================================
function cost = MPC_Costfunction(var,NP,dt)

    x_start     = 0;
    y_start     = x_start + NP;
    phi_start   = y_start + NP;
    cte_start   = phi_start + NP;
    ephi_start  = cte_start + NP;
    delta_start = ephi_start + NP;
    
    cte   = 0;
    ephi  = 0;
    delta = 0;
    delta_change = 0;
    for i = 1:1:NP
        cte   = cte + var( cte_start+i )^2;
        ephi  = ephi + var( ephi_start+i)^2;
        delta = delta + var( delta_start+i )^2;
    end
    
    for i = 1:1:NP-1
        delta_change = delta_change + ( var( delta_start+i+1 ) - var( delta_start+i ) )^2;
    end    
    
    % �����ͷ����Ȩ����Ҫ���ĵĵ��Σ��ͷ��������󽵵�ʵʱ��
    cost = 8*cte + 0 * ephi + 0*delta + 60*delta_change;
    
end