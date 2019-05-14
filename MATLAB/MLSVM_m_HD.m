function abcstate = MLSVM_m_HD(svminput)

% input: [vd, vq, v0, wt, voltage level, switching frequency, capacitor voltage, method]
% output: [sa, sb, sc] ,range(0~level-1)

v1 = svminput(1);
v2 = svminput(2);
v3 = svminput(3);
wt = svminput(4);
N = svminput(5) - 1;
fs = svminput(6);           % updata 10 times in a sector, 10*6*50=3e3
vc = svminput(7);
isnarmal = svminput(10);
Ts = 1 / fs;
% if wt / (2*pi*50) == 1e-3
%     a = 1;
% end
time = mod(wt / (2*pi*50), Ts);
if isnarmal == 1
    vc = 2 / N;
end
global pre;
if isempty(pre)
    pre = [0 0 0];
end
global freq;
if isempty(freq)
    freq = 0;
end
global freq_line;
if isempty(freq_line)
    freq_line = 0;
end
global freq_com;
if isempty(freq_com)
    freq_com = 0;
end

switch svminput(8)
    case 1      % H-D Axis
        if svminput(9) == 1        % dq0
            sinwt = sin(wt);
            coswt = cos(wt);
            k1 = 3/2;
            k2 = sqrt(3)/2;
            T = 1 / vc * [k1*sinwt+k2*coswt, k1*coswt-k2*sinwt; k1*sinwt-k2*coswt, k1*sinwt+k2*coswt];
            vhd = T * [v1; v2];
        elseif svminput(9) == 2                    % ab/bc/ca
            T = 1 / vc * [1 0 0; 0 0 -1];
            vhd = T * [v1; v2; v3];
        elseif svminput(9) == 3
            T = 1 / vc * [1 -1 0; 1 0 -1];
            vhd = T * [v1; v2; v3];
        end
        % 过调制
%         if vhd(1) > 0
%             if vhd(2) > N || vhd(2) < N - vhd(1)
            
        v0h = floor(vhd(1));
        v0d = floor(vhd(2));
        hx = vhd(1) - v0h;
        dx = vhd(2) - v0d;
        vdif = hx - dx;

        if (vdif >= 0)    % 倒三角形
            d1 = dx;
            d2 = vdif;
%             statechange = [1 0 0; 0 0 1; 0 1 0];    % ACB up
            change2 = [1 3 2];
        else
            d1 = hx;
            d2 = -vdif;
%             statechange = [1 0 0; 0 1 0; 0 0 1];    % ABC up
            change2 = [1 2 3];
        end
        d0 = 1 - d1 - d2;
        state = [0 -v0h -v0d];  % can only output double
        
    case 2      % g-h Axis
        if svminput(9) == 1
            sinwt = sin(wt);
            coswt = cos(wt);
            k1 = 3/2;
            k2 = sqrt(3)/2;
            T = 1 / vc * [k1*sinwt+k2*coswt, k1*coswt-k2*sinwt; k1*sinwt-k2*coswt, k1*sinwt+k2*coswt];
            vgh = T * [v1; v2];
        else
            T = 1 / vc * [1 0 0; 0 1 0];
            vgh = T * [v1; v2; v3];
        end
        
        vulg = ceil(vgh(1));
        vulh = floor(vgh(2));
        if vgh(1)+vgh(2) - (vulg+vulh) < 0      % 正三角
            d1 = vgh(1) - (vulg-1);     % 相当于向下取整，注意2的ceil和floor都是2
            d2 = vgh(2) - vulh;
            state = [0 -(vulg-1) -(vulg-1)-vulh];
            statechange = [1 0 0; 0 1 0; 0 0 1];    % 从左下角开始按逆时针
            change2 = [1 2 3];
        else              % 倒三角
            d1 = (vulh+1) - vgh(2);     % 向上取整
            d2 = vulg - vgh(1);
            state = [0 -vulg -vulg-(vulh+1)];
            statechange = [0 0 1; 0 1 0; 1 0 0];    % 从右上角开始按顺时针
            change2 = [3 2 1];
        end
        d0 = 1 - d1 - d2;

    case 3      % ab-bc-ca Axis
        if svminput(9) == 1
            sinwt = sin(wt);
            coswt = cos(wt);
            k1 = 3/2;
            k2 = sqrt(3)/2;
            T = 1 / vc * [k1*sinwt+k2*coswt, k1*coswt-k2*sinwt; k1*sinwt-k2*coswt, k1*sinwt+k2*coswt];
            vline = T * [v1; v2];
        else
            T = 1 / vc;
            vline = T * [v1; v2; v3];
        end
        
        fab = floor(vline(1));
        fbc = floor(vline(2));
        fca = floor(vline(3));
        if fab + fbc + fca == -1      % 正三角
%             d0 = vline(3) - fca;
            d1 = vline(1) - fab;
            d2 = vline(2) - fbc;
            statechange = [1 0 0; 0 1 0; 0 0 1];    % 从左下角开始按逆时针
            change2 = [1 2 3];
        else              % 倒三角
%             d0 = fab+1 - vline(1);
            d1 = fca+1 - vline(3);
            d2 = fbc+1 - vline(2);
            statechange = [1 0 0; 0 0 1; 0 1 0];    % 从左上角开始按顺时针
            change2 = [1 3 2];
        end
        d0 = 1 - d1 - d2;
        state = [0 -fab fca+1];
        
    case 4      % H-D*3 Axis
        if svminput(9) == 1
            set_param('mysvm','SimulationCommand','stop');
        end
        endflag = false;
        global mode
        if isempty(mode)
            mode = 3;
        end
        T = 1 / vc * [1 0 0; 0 0 -1; 0 1 0; -1 0 0; 0 0 1; 0 -1 0];
        orders = [1 3 2; 1 2 3; 2 1 3; 2 3 1; 3 2 1; 3 1 2];
        changes = [1 0 0; 0 1 0; 0 0 1];
        while endflag == false 
% {
            vhd = T(mode: mode+1, :) * [v1; v2; v3];    %坐标变换
            
            if vhd(1) < 0 || vhd(2) < 0                 %区域判断，mode=1,3,5
                mode = mode + 2;
                if mode == 7
                    mode = 1;
                end
                continue
            end
            
            if vhd(1) > N                               %过调制判断
                vhd(2) = vhd(2) / vhd(1) * N;
                vhd(1) = N;
            end
            if vhd(2) > N
                vhd(1) = vhd(1) / vhd(2) * N;
                vhd(2) = N;
            end

            v0h = floor(vhd(1)-eps);             %中间变量
            v0d = floor(vhd(2)-eps);      % -3*eps以防止外边界整数floor，但内边界也因此归至-1，
            hx = vhd(1) - v0h;
            dx = vhd(2) - v0d;
            vdif = hx - dx;
            if (vdif > 0)    % 倒三角形
                d1 = dx;
                d2 = vdif;
                order = orders(mode, :);    % ABC相顺序
            else
                d1 = hx;
                d2 = -vdif;
                order = orders(mode+1, :);
            end
            d0 = 1 - d1 - d2;
            states = zeros(4,3);
            states(1, orders(mode, 2)) = -v0d;  
            states(1, orders(mode, 3)) = -v0h;
            states(1,:) = states(1,:) + floor(N/2 + (v0h+v0d)/3);
%             if max(states(1,:)) > N
%                 states(1,:) = states(1,:) - (max(states(1,:))-N) - 1;   %-1留裕量
%             elseif min(states(1,:)) < 0
%                 states(1,:) = states(1,:) - min(states(1,:));
%             end
            states(2,:) = states(1,:) + changes(order(1),:);
            states(3,:) = states(2,:) + changes(order(2),:);
            states(4,:) = states(3,:) + changes(order(3),:);
            endflag = true;
%}
        end
        
    case 5      % D-Y Axis
        sins = [0, sqrt(3)/2, sqrt(3)/2, 0, -sqrt(3)/2, -sqrt(3)/2, 0];    % 7个
        coss = [1, 0.5, -0.5, -1, -0.5, 0.5, 1];
        k = sqrt(3);
        if svminput(9) == 1        % dq0
            sinwt = sin(wt);
            coswt = cos(wt);
            k1 = 3/2;
            k2 = sqrt(3)/2;
            T = 1 / vc * [k1*sinwt+k2*coswt, k1*coswt-k2*sinwt; k1*sinwt-k2*coswt, k1*sinwt+k2*coswt];
            vxyy = T * [v1; v2];
        elseif svminput(9) == 2                   % line
            T = 1 / vc * [0.5 0 -0.5; 0 0.5 0; 0 -0.5, 0]; % x y -y
            vxyy = T * [v1; v2; v3];
        end
        state = fix(vxyy - min(vxyy));
        vx = vxyy(1) - state(1) + state(2)*0.5 + state(3)*0.5;
        vy = vxyy(2) - state(2)*0.5 + state(3)*0.5;
        if (0 <= vy && vy < vx)
            reg = 2;
        elseif (vy >= vx && vy > -vx)
            reg = 3;
        elseif (0 < vy && vy <= -vx)
            reg = 4;
        elseif (vx < vy && vy <= 0)
            reg = 5;
        elseif (vy <= vx && vy < -vx)
            reg = 6;
        else
            reg = 7;
        end
        d1 = 2/sqrt(3) * vx * sins(reg) - 2 * vy * coss(reg);
        d2 = -2/sqrt(3) * vx * sins(reg-1) + 2 * vy * coss(reg-1);
        d0 = 1 - d1 - d2;
        switch reg
            case 2
                change2 = [1 2 3];
            case 3
                change2 = [2 1 3];
            case 4
                change2 = [2 3 1];
            case 5
                change2 = [3 2 1];
            case 6
                change2 = [3 1 2];
            case 7
                change2 = [1 3 2];
        end
       
    case 6      % rotated HD
        if svminput(9) == 3                   % abc
            T = 1 / vc * [0 -1 0; 1 0 0];
            vhd = T * [v1; v2; v3];
        end
        v0h = floor(vhd(1));
        v0d = floor(vhd(2));
        hx = vhd(1) - v0h;
        dx = vhd(2) - v0d;
        vdif = hx - dx;

        if (vdif >= 0)    % 倒三角形
            d1 = dx;
            d2 = vdif;
            statechange = [1 0 0; 0 0 1; 0 1 0];    % AB-CA-BC up
            change2 = [1 3 2];
        else
            d1 = hx;
            d2 = -vdif;
            statechange = [1 0 0; 0 1 0; 0 0 1];    % AB-BC-CA up
            change2 = [1 2 3];          % ab=a++b--, bc=b++c--, ca=c++a--
        end
        d0 = 1 - d1 - d2;
        state = [0 -(v0h+v0d) v0h-2*v0d];  % can only output double
end
   
if svminput(8) ~= 4
    states = zeros(4,3);
    % state1 = state - min(state);    % choose the smallest state
    states(1,:) = state + floor(N/2 - sum(state)/3.0);      % choose the smallest common voltage state
%{
    states(2,:) = states(1,:) + statechange(1,:);
    states(3,:) = states(2,:) + statechange(2,:);
    states(4,:) = states(3,:) + statechange(3,:);
    for i = 1:4
        if max(states(i,:)) > N
            states(i,:) = states(i,:) - (max(states(i,:))-N);
        elseif min(states(i,:)) < 0
            states(i,:) = states(i,:) - min(states(i,:));
        end
    end
    %}
    % 占空比形式输出，与后者波形完全相同
% {
    t = [0 0 0];
    t(3) = d0 * Ts / 4;
    t(2) = t(3) + d2 * Ts / 2;
    t(1) = t(2) + d1 * Ts / 2;
    abcstate = states(1,:);
    if svminput(8) ~= 6
        for i = 1:3
            if (Ts/2 - t(i) <= time && time < Ts/2 + t(i))
                abcstate(change2(i)) = abcstate(change2(i)) + 1;
            end
        end
    else
        for i = 1:3
            if (Ts/2 - t(i) <= time && time < Ts/2 + t(i))
                abcstate(change2(i)) = abcstate(change2(i)) + 1;
                abcstate(mod(change2(i), 3)+1) = abcstate(mod(change2(i), 3)+1) - 1;
            end
        end
    end
    if max(abcstate) > N
            abcstate = abcstate - (max(abcstate)-N);
    elseif min(abcstate) < 0
            abcstate = abcstate - min(abcstate);
    end
    selected = abcstate;
    if (~all(pre == [0 0 0]) && ~all(pre == selected))
        freq = freq + sum(abs(pre - selected));
        freq_line = freq_line + sum(abs([pre(1)-pre(2),pre(2)-pre(3),pre(3)-pre(1)] - [selected(1)-selected(2),selected(2)-selected(3),selected(3)-selected(1)]));
        freq_com = freq_com + abs((sum(pre)-sum(selected)));
    end
    pre = selected;
%}
else
% 时间序列形式输出
% {
    t1 = d1 * Ts;
    t2 = d2 * Ts;
    t0 = d0 * Ts;
    t = zeros(1, 6);
    t(1) = t0/4;      %6个开关时刻
    t(2) = t(1) + t1/2;
    t(3) = t(2) + t2/2;
    t(4) = t(3) + t0/2;
    t(5) = t(4) + t2/2;
    t(6) = t(5) + t1/2;
    seq = 1 + (time >= t(1)) + (time >= t(2)) + (time >= t(3)) + (time > t(4)) + (time > t(5)) + (time > t(6));
    
    switch seq
        case {1,7} 
            selected = states(1,:);
        case {2,6}  %t1
            selected = states(2,:);
        case {3,5}  %t2
            selected = states(3,:);
        otherwise
            selected = states(4,:);
    end
    
    if max(selected) > N
        selected = selected - (max(selected)-N);   %-1留裕量
    elseif min(states(1,:)) < 0
        selected = selected - min(selected);
    end
    if (~all(pre == [0 0 0]) && ~all(pre == selected))
        freq = freq + sum(abs(pre - selected));
        freq_line = freq_line + sum(abs([pre(1)-pre(2),pre(2)-pre(3),pre(3)-pre(1)] - [selected(1)-selected(2),selected(2)-selected(3),selected(3)-selected(1)]));
        freq_com = freq_com + abs((sum(pre)-sum(selected)));
    end
    pre = selected;
    
    abcstate = selected;
    
%}
end
