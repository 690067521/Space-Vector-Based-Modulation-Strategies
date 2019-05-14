function abcstate = MLSVC_m(svcinput)

% input: [va, vb, vc, voltage level, switching frequency, modulation ratio]
% output: [sa, sb, sc] ,range(0~level-1)

vabc = svcinput(1:3);
N = svcinput(4) - 1;        % number of carrier waves
method = svcinput(5);
% t = get_param('mysvm_svc','SimulationTime');
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
switch method
    case 1  % Conventional x-y
        vcc = 2 / N;    % 归一化系数
        vx = (2 * vabc(1) - vabc(2) - vabc(3)) / vcc;   % alpha
        vy = (vabc(2) - vabc(3)) / vcc;                 % belta
        bl = [floor(vx-eps) floor(vy-eps)];             % 在整数线/点上的矢量的bl也保证在左下角
        if rem(sum(bl), 2) == 0 % 偶数
            v1 = bl;
            v2 = v1 + 1;
        else
            v1 = [ceil(vx) floor(vy)];  % 右下
            v2 = v1 + [-1 1];           % 左上
        end
        v0 = (v1 + v2) / 2;     % 中点
        k = (-(v2(1) - v1(1)) / (v2(2) - v1(2))) / 3;   % 斜率
        if vy > k * (vx - v0(1)) + v0(2)        % 分割线
            vo = v2;
        else
            vo = v1;
        end
        k = round(N/2 + vo(1)/3);   % round因为只有一个开关状态，vo2抵消了
        selected = [k, k - (vo(1) - vo(2)) / 2, k - (vo(1) + vo(2)) / 2];
        if max(selected) > N
            selected = selected - (max(selected) - N);   %-1留裕量
        elseif min(selected) < 0
            selected = selected - min(selected);
        end
        
    case 2  % x-y,迭代
        global selected;
        if isempty(selected)
            selected = [0 0 0];
        end
        vcc = 2 / N;    % 归一化系数
        origin = (selected-sum(selected)/3)*vcc;
        T = 1 / vcc * [2 -1 -1; 0 1 -1];
        vXY = T * (vabc - origin'); % = T * vabc - T*origin, 即减去原selected在xy坐标系下的值即可
        vx = vXY(1);        % alpha
        vy = vXY(2);        % belta
        % {
        if (abs(vx) > 1 || abs(vy-vx/3) > 2/3 || abs(vy+vx/3) > 2/3)    % 先判断六边形越界再判断扇区
            change = [1 0 0; 0 0 -1; 0 1 0; -1 0 0; 0 0 1; 0 -1 0];
            if (vy >= -vx/3 && vy < vx/3)   % area1
                selected = selected + change(1, :);
            elseif (vy >= vx/3 && vx >= 0)   % area2
                selected = selected + change(2, :);
            elseif (vx < 0 && vy >= -vx/3)   % area3
                selected = selected + change(3, :);
            elseif (vy < -vx/3 && vy >= vx/3)   % area4
                selected = selected + change(4, :);
            elseif (vy < vx/3 && vx < 0)   % area5
                selected = selected + change(5, :);
            else  % (vx >= 0 && vy < -vx/3)   % area6
                selected = selected + change(6, :);
            end
        %}
        %{
        change = [1 0 0; 0 0 -1; 0 1 0; -1 0 0; 0 0 1; 0 -1 0];
        flag = true;
        if (vx >= 0)    % 使用向下取整的方法同样需要判断在那个扇区，最原始的方法
            if (vx > 1 && -vx/3 < vy && vy <= vx/3)   % area1, 由三条边界划分
                selected = selected + change(1, :);
            elseif (vx/3 < vy && -vx/3+2/3 < vy)   % area2
                selected = selected + change(2, :);
            elseif (vy <= -vx/3 && vy < vx/3-2/3)   % area6
                selected = selected + change(6, :);
            else
                flag = false;
            end
        else
            if (vx < -1 && vx/3 < vy && vy <= -vx/3)   % area4
                selected = selected + change(4, :);
            elseif (-vx/3 < vy && vx/3+2/3 < vy)   % area3
                selected = selected + change(3, :);
            elseif (vy <= vx/3 && vy < -vx/3-2/3)   % area5
                selected = selected + change(5, :);
            else
                flag = false;
            end
        end
        if flag
        %}
            k = round(N/2 - sum(selected)/3);   % round因为只有一个开关状态，vo2抵消了
            selected = selected + k;
            if max(selected) > N
                selected = selected - (max(selected) - N);   %-1留裕量
            elseif min(selected) < 0
                selected = selected - min(selected);
            end
        end
        
    case 3  % HD
        vcc = 2 / N;    % 归一化系数
        T = 1 / vcc * [1 -1 0; 1 0 -1];
        vhd = T * vabc;
        v0h = floor(vhd(1));
        v0d = floor(vhd(2));
        hx = vhd(1) - v0h;
        dx = vhd(2) - v0d;
        vdif = hx - dx;
        t1 = sum(vhd);
        v0sum = v0h + v0d;
        if (vdif >= 0)    % 倒三角形， 确定三角形之后就不管奇偶了
            if (t1 < v0sum + 1 && dx >= hx*2-1)    % left & high
                    vo = [v0h v0d];
            elseif (t1 >= v0sum + 1 && dx >= hx/2)    % right & high
                    vo = [v0h+1 v0d+1];
            else
                    vo = [v0h+1 v0d];
            end
        else        % 正三角
            if (t1 <= v0sum + 1 && dx <= hx/2+0.5)    % left & low
                    vo = [v0h v0d];
            elseif (t1 >= v0sum + 1 && dx <= hx*2)    % right & low
                    vo = [v0h+1 v0d+1];
            else
                    vo = [v0h v0d+1];
            end
        end
        selected = [0 -vo(1) -vo(2)];
        k = round(N/2 - sum(selected)/3);   % round因为只有一个开关状态，vo2抵消了
        selected = selected + k;
        if max(selected) > N
            selected = selected - (max(selected) - N);   %-1留裕量
        elseif min(selected) < 0
            selected = selected - min(selected);
        end
        
    case 4  % HD,迭代
        global selected;
        if isempty(selected)
            selected = [0 0 0];
        end
        vcc = 2 / N;    % 归一化系数
        T = 1 / vcc * [1 -1 0; 1 0 -1];
        origin = (selected-sum(selected)/3)*vcc;
        vhd = T * (vabc - origin');
        if (abs(vhd(1)+vhd(2)) > 1 || abs(vhd(2)-vhd(1)/2) > 0.5 || abs(vhd(2)-vhd(1)*2) > 1)
            change = [1 0 0; 0 0 -1; 0 1 0; -1 0 0; 0 0 1; 0 -1 0];
            if (vhd(2) >= vhd(1)/2 && vhd(2) < vhd(1)*2)    % area1
                selected = selected + change(1, :);
            elseif (vhd(2) >= vhd(1)*2 && vhd(2) >= -vhd(1))   % area2
                selected = selected + change(2, :);
            elseif (vhd(2) < -vhd(1) && vhd(2) >= vhd(1)/2)   % area3
                selected = selected + change(3, :);
            elseif (vhd(2) < vhd(1)/2 && vhd(2) >= vhd(1)*2)   % area4
                selected = selected + change(4, :);
            elseif (vhd(2) < vhd(1)*2 && vhd(2) < -vhd(1))   % area5
                selected = selected + change(5, :);
            else  % (vhd(2) >= -vhd(1) && vhd(2) < vhd(1)/2)   % area6
                selected = selected + change(6, :);
            end
            k = round(N/2 - sum(selected)/3);   % round因为只有一个开关状态，vo2抵消了
            selected = selected + k;
            if max(selected) > N
                selected = selected - (max(selected) - N);   %-1留裕量
            elseif min(selected) < 0
                selected = selected - min(selected);
            end
        end
        
    case 5  % BJ120
        vcc = 2 / N;    % 归一化系数
        T = 1 / vcc * [2 -1 -1; 1 1 -2];
        vBJ = T * vabc;
        v0B = floor(vBJ(1));
        v0J = floor(vBJ(2));
        switch mod(v0B + v0J, 3)
            case 0
                vo = [v0B v0J];
            case 1
                vo = [v0B+1 v0J+1];
            case 2
                if (vBJ(1) + v0J > vBJ(2) + v0B)
                    vo = [v0B+1 v0J];
                else
                    vo = [v0B v0J+1];
                end
        end
        selected = [0 -(2*vo(1)-vo(2))/3 -(vo(1)+vo(2))/3];
        k = round(N/2 + vo(1)/3);   % round因为只有一个开关状态，vo2抵消了
        selected = selected + k;
        if max(selected) > N
            selected = selected - (max(selected) - N);   %-1留裕量
        elseif min(selected) < 0
            selected = selected - min(selected);
        end

    case 6  % BJ120°坐标系，迭代法
        global selected;
        if isempty(selected)
            selected = [0 0 0];
        end
        vcc = 2 / N;    % 归一化系数
        origin = (selected-sum(selected)/3)*vcc;
        %{
        % 多坐标系坐标变换
        global area;
        if isempty(area)
            area = 0;
        end
        T = 1 / vcc * [2 -1 -1;1 1 -2; -1 2 -1;-2 1 1; -1 -1 2;1 -2 1];
        vBJ = T(area*2+1:area*2+2, :) * (vabc - origin');
        while (vBJ(1) < 0 || vBJ(2) < 0)
            area = mod(area + 1, 3);
            vBJ = T(area*2+1:area*2+2, :) * (vabc - origin');
        end
        if (vBJ(1) > 1 || vBJ(2) > 1)   % change along the direction, else still selected
            change = [1 0 0; 0 0 -1; 0 1 0; -1 0 0; 0 0 1; 0 -1 0];
            if (vBJ(1) > vBJ(2))        % two ways
                selected = selected + change(2*area+1, :);
            else
                selected = selected + change(2*area+2, :);
            end
        %}
        % {
        % 坐标区域判断
        T = 1 / vcc * [2 -1 -1;1 1 -2];
        vBJ = T * (vabc - origin');
        if (abs(vBJ(1)) > 1 || abs(vBJ(2)) > 1 || abs(vBJ(2)-vBJ(1)) > 1)   % edges
            change = [1 0 0; 0 0 -1; 0 1 0; -1 0 0; 0 0 1; 0 -1 0]; % [vBJ(1)>=0, vBJ(2)>=0, vBJ(2)>=vBJ(1)]
            if (vBJ(2) >= 0 && vBJ(2) < vBJ(1))    % area1, [* 1 0] [1 * 1] [0 1 *] [* 0 1] [0 * 0] [1 0 *], 无[0 1 0][1 0 1]
                selected = selected + change(1, :);
            elseif (vBJ(2) >= vBJ(1) && vBJ(1) >= 0) % area2, use array can speed up searching time
                selected = selected + change(2, :);
            elseif (vBJ(1) < 0 && vBJ(2) >= 0)      % area3
                selected = selected + change(3, :);
            elseif (vBJ(2) < 0 && vBJ(2) >= vBJ(1))  % area4
                selected = selected + change(4, :);
            elseif (vBJ(2) < vBJ(1) && vBJ(1) < 0) % area5
                selected = selected + change(5, :);
            elseif (vBJ(1) >= 0 && vBJ(2) < 0)      % area6
                selected = selected + change(6, :);
            end
        %}
            k = round(N/2 - sum(selected)/3);   % round因为只有一个开关状态，vo2抵消了
            selected = selected + k;
            if max(selected) > N
                selected = selected - (max(selected) - N);   %-1留裕量
            elseif min(selected) < 0
                selected = selected - min(selected);
            end
        end
        
    case 7  % BJ60°坐标系
        vcc = 2 / N;    % 归一化系数
        T = 1 / vcc * [1 -2 1; 1 1 -2];
        vBJ = T * vabc;
        v0B = floor(vBJ(1));
        v0J = floor(vBJ(2));
        switch mod(v0J - v0B, 3)
            case 2
                vo = [v0B v0J+1];
            case 1
                vo = [v0B+1 v0J];
            case 0
                if (vBJ(1) + vBJ(2) < v0B + v0J + 1)
                    vo = [v0B v0J];
                else
                    vo = [v0B+1 v0J+1];
                end
        end
        selected = [0 -(2*vo(1)+vo(2))/3 -(vo(1)+2*vo(2))/3];
        k = round(N/2 + (vo(1)+vo(2))/3);   % round因为只有一个开关状态
        selected = selected + k;
        if max(selected) > N
            selected = selected - (max(selected) - N);   %-1留裕量
        elseif min(selected) < 0
            selected = selected - min(selected);
        end   
        
    case 8  % BJ60°坐标系，迭代法
        global selected;
        if isempty(selected)
            selected = [0 0 0];
        end
        vcc = 2 / N;    % 归一化系数
        origin = (selected-sum(selected)/3)*vcc;    % state to three-phase
        % {
        % 坐标系不变判断扇区的方法可能更为简单
        global area;
        if isempty(area)
            area = 0;
        end
        T = 1 / vcc * [1 -2 1;1 1 -2; 2 -1 -1;-1 2 -1; 1 1 -2;-2 1 1; -1 2 -1;-1 -1 2; -2 1 1;1 -2 1; -1 -1 2;2 -1 -1];
        vBJ = T(area*2+1:area*2+2, :) * (vabc - origin');
        while (vBJ(1) < 0 || vBJ(2) < 0)    % find the direction
            area = mod(area + 1, 6);
            vBJ = T(area*2+1:area*2+2, :) * (vabc - origin');
        end
        if (sum(vBJ) > 1)   % change along the direction, else still selected
            change = [1 0 0; 0 0 -1; 0 1 0; -1 0 0; 0 0 1; 0 -1 0];
            selected = selected + change(area+1, :);
        %}
        %{
        % 通过坐标判断区域而非坐标变换
        T = 1 / vcc * [1 -2 1; 1 1 -2];     % need only one transformation
        vBJ = T * (vabc - origin');
        if (abs(sum(vBJ)) > 1 || abs(vBJ(1)) > 1 || abs(vBJ(2)) > 1) % edges
            change = [1 0 0; 0 0 -1; 0 1 0; -1 0 0; 0 0 1; 0 -1 0];
            if (vBJ(2) >= 0 && vBJ(1) >= 0)         % area1
                selected = selected + change(1, :);
            elseif (vBJ(1) < 0 && vBJ(2) >= -vBJ(1)) % area2
                selected = selected + change(2, :);
            elseif (vBJ(2) < -vBJ(1) && vBJ(2) >= 0) % area3
                selected = selected + change(3, :);
            elseif (vBJ(2) < 0 && vBJ(1) < 0)       % area4
                selected = selected + change(4, :);
            elseif (vBJ(1) >= 0 && vBJ(2) < -vBJ(1)) % area5
                selected = selected + change(5, :);
            else    % (vBJ(2) >= -vBJ(1) && vBJ(2) < 0) % area6
                selected = selected + change(6, :);
            end
        %}
            k = round(N/2 - sum(selected)/3);   % round因为只有一个开关状态，vo2抵消了
            selected = selected + k;
            if max(selected) > N
                selected = selected - (max(selected) - N);   %-1留裕量
            elseif min(selected) < 0
                selected = selected - min(selected);
            end
        end
        
    case 9  % ABC
        vcc = 2 / N;    % 归一化系数
        vABC = 3 / vcc * vabc;
        fABC = floor(vABC);
        cABC = ceil(vABC);
        if (mod(cABC(3) + 2*fABC(1), 3) == 0)       % cc+2fa=3k，右上角菱形
            vo = [fABC(1) -fABC(1)-cABC(3) cABC(3)];
        elseif (mod(cABC(1) + 2*fABC(2), 3) == 0)   % ca+2fb=3k，左上角菱形
            vo = [cABC(1) fABC(2) -cABC(1)-fABC(2)];
        else
%             (mod(cABC(2) - fABC(3), 3) == 0)     % cb-fc=3k,
%             只有三种情况，故必然成立，下方菱形
            vo = [-cABC(2)-fABC(3) cABC(2) fABC(3)];
        end
        selected = [0 (vo(2)-vo(1))/3 -(vo(2)+2*vo(1))/3];
        k = round(N/2 + vo(1)/3);   % round因为只有一个开关状态，坐标之和为0
        selected = selected + k;
        if max(selected) > N
            selected = selected - (max(selected) - N);   %-1留裕量
        elseif min(selected) < 0
            selected = selected - min(selected);
        end
        
    case 10 % ABC坐标系，迭代法
        global selected;
        if isempty(selected)
            selected = [0 0 0];
        end
        vcc = 2 / N;    % 归一化系数
        origin = (selected-sum(selected)/3)*vcc;
        vABC = 3 / vcc * (vabc - origin');
        % {  
        % 划分区域
        if (abs(vABC(1)) > 1 || abs(vABC(2)) > 1 || abs(vABC(3)) > 1)
            change = [1 0 0; 0 0 -1; 0 1 0; -1 0 0; 0 0 1; 0 -1 0];
            if (vABC(3) < 0 && vABC(2) < 0)         % area 1，每个区域的坐标有明显特征
                selected = selected + change(1, :);
            elseif (vABC(2) >= 0 && vABC(1) >= 0) 	% area 2
                selected = selected + change(2, :);
            elseif (vABC(1) < 0 && vABC(3) < 0)     % area 3
                selected = selected + change(3, :);
            elseif (vABC(3) >= 0 && vABC(2) >= 0) 	% area 4
                selected = selected + change(4, :);
            elseif (vABC(2) < 0 && vABC(1) < 0)     % area 5
                selected = selected + change(5, :);
            else                                    % area 6 (vABC(1) >= 0 && vABC(3) >= 0)
                selected = selected + change(6, :);
            end
        %}
        %{
        % along with largest direction
        [maxv, maxi] = max(vABC);   % 同理可用六轴，另外三轴为ABC的负矢量，则取最大值即可
        [minv, mini] = min(vABC);
        if (maxv > 1 || minv < -1)
            change = [1 0 0; 0 0 -1; 0 1 0; -1 0 0; 0 0 1; 0 -1 0];
            if (maxv > -minv)           % max value direction
                selected = selected + change(maxi*2-1, :);  % 1 3 5
            else                        % min value negtive direction
                selected = selected + change(mod(mini*2, 6)+2, :);  % 4 6 2
            end
        %}
            k = round(N/2 - sum(selected)/3);   % round因为只有一个开关状态，vo2抵消了
            selected = selected + k;
            if max(selected) > N
                selected = selected - (max(selected) - N);   %-1留裕量
            elseif min(selected) < 0
                selected = selected - min(selected);
            end
        end
        
    case 11 % A-CB-AC-B六轴坐标系，可能继续改进
        vcc = 2 / N;    % 归一化系数
        T = 3 / vcc * [1 0 0; 0 0 -1; 0 1 0; -1 0 0; 0 0 1; 0 -1 0];    % [A -C B -A C -B]
        vABC = T * vabc;
        fABC = floor(vABC);
        if (mod(2*fABC(1) - fABC(2), 3) == 0)       % 2fa-f(-c)=3k，右上角菱形
            vo = [fABC(1) fABC(2) fABC(2)-fABC(1) -fABC(1) -fABC(2) fABC(1)-fABC(2)];
        elseif (mod(2*fABC(3)- fABC(4), 3) == 0)    % 2fb-f(-a)=3k，左上角菱形
            vo = [-fABC(4) fABC(3)-fABC(4) fABC(3) fABC(4) fABC(4)-fABC(3) -fABC(3)];
        else   % (mod(-fABC(6) - fABC(5), 3) == 0)   % -f(-b)-fc=3k,只有三种情况，故必然成立，下方菱形
            vo = [fABC(6)-fABC(5) -fABC(5) -fABC(6) fABC(5)-fABC(6) fABC(5) fABC(6)];
        end
        selected = [0 (vo(3)+vo(4))/3 (-vo(3)+2*vo(4))/3];
        k = round(N/2 + vo(1)/3);   % round因为只有一个开关状态，vo2抵消了
        selected = selected + k;
        if max(selected) > N
            selected = selected - (max(selected) - N);   %-1留裕量
        elseif min(selected) < 0
            selected = selected - min(selected);
        end
        
    case 12 % A-CB-AC-B六轴坐标系，迭代法
        global selected;
        if isempty(selected)
            selected = [0 0 0];
        end
        vcc = 2 / N;    % 归一化系数
        origin = (selected-sum(selected)/3)*vcc;
        T = 3 / vcc * [1 0 0; 0 0 -1; 0 1 0; -1 0 0; 0 0 1; 0 -1 0];
        vABC = T * (vabc - origin');
        % {
        % 划分区域
        if (any(vABC > 1))
            change = [1 0 0; 0 0 -1; 0 1 0; -1 0 0; 0 0 1; 0 -1 0];
            if (vABC(2) >= 0 && vABC(3) < 0)         % area 1，每个区域的坐标有明显特征
                selected = selected + change(1, :);
            elseif (vABC(3) >= 0 && vABC(4) < 0) 	% area 2
                selected = selected + change(2, :);
            elseif (vABC(4) >= 0 && vABC(5) < 0)     % area 3
                selected = selected + change(3, :);
            elseif (vABC(5) >= 0 && vABC(6) < 0) 	% area 4
                selected = selected + change(4, :);
            elseif (vABC(6) >= 0 && vABC(1) < 0)     % area 5
                selected = selected + change(5, :);
            else                                    % area 6 (vABC(1) >= 0 && vABC(2) < 0)
                selected = selected + change(6, :);
            end
        %}
        %{
        [maxv, maxi] = max(vABC);   % 六轴，另外三轴为ABC的负矢量，则取最大值即可
        if (maxv > 1)
            change = [1 0 0; 0 0 -1; 0 1 0; -1 0 0; 0 0 1; 0 -1 0];
            selected = selected + change(maxi, :);
            %}
            k = round(N/2 - sum(selected)/3);   % round因为只有一个开关状态，vo2抵消了
            selected = selected + k;
            if max(selected) > N
                selected = selected - (max(selected) - N);   %-1留裕量
            elseif min(selected) < 0
                selected = selected - min(selected);
            end
        end
        
    case 13 % 移位HD，但移位后难以应用于迭代方法
        vcc = 2 / N;    % 归一化系数
        T = 1 / vcc * [1 -1 0; 1 0 -1];
        vhd = T * vabc - [-1/3 -2/3]';
        v0h = floor(vhd(1));
        v0d = floor(vhd(2));
        hx = vhd(1) - v0h;
        dx = vhd(2) - v0d;
        selected = [0 -v0h -v0d];
%         vo = [v0h v0d];
        if (hx >= dx)   % 倒三角
            if (hx+dx < 1 && dx < hx/2)  % left & low
                selected(3) = selected(3) + 1;
%                 vo = [v0h v0d-1];
            elseif (hx+dx >= 1 && dx+1 < hx*2) % right & low
                selected(2) = selected(2) - 1;
%                 vo = [v0h+1 v0d];
            end     % else remain selected
        end
        k = round(N/2 - sum(selected)/3);   % round因为只有一个开关状态，vo2抵消了
        selected = selected + k;
        if max(selected) > N
            selected = selected - (max(selected) - N);   %-1留裕量
        elseif min(selected) < 0
            selected = selected - min(selected);
        end
        
    case 14 % 移位BJ60
        vcc = 2 / N;    % 归一化系数
        T = 0.5 / vcc * [1 -2 1; 1 1 -2];
        vBJ = T * vabc - [-0.5 -0.5]';
        v0B = floor(vBJ(1));
        v0J = floor(vBJ(2));
        Bx = vBJ(1) - v0B;
        Jx = vBJ(2) - v0J;
        switch (mod(v0B-v0J, 3))
            case 0
                vo = [v0B+0.5 v0J+0.5];   % 不 直接取shift之前坐标，节省运算
                if (Bx+Jx < 0.5)
                    vo = vo - [0.5 0.5];
                elseif (Bx+Jx > 1.5)
                    vo = vo + [0.5 0.5];
                end
            case 1
                vo = [v0B v0J+1];
                if (Bx+Jx < 1)    % left
                    if (Jx < 0.5)
                        vo = vo + [0.5 -1];
                    end
                else
                    if (Bx >= 0.5)
                        vo = vo + [1 -0.5];
                    end
                end
            case 2
                vo = [v0B+1 v0J];
                if (Bx+Jx < 1)    % left
                    if (Bx < 0.5)
                        vo = vo + [-1 0.5];
                    end
                else
                    if (Jx >= 0.5)
                        vo = vo + [-0.5 1];
                    end
                end
        end
        vo = (vo + [-0.5 -0.5]) * 2;
        selected = [0 -(2*vo(1)+vo(2))/3 -(vo(1)+2*vo(2))/3];
        k = round(N/2 + (vo(1)+vo(2))/3);   % round因为只有一个开关状态，vo2抵消了
        selected = selected + k;
        if max(selected) > N
            selected = selected - (max(selected) - N);   %-1留裕量
        elseif min(selected) < 0
            selected = selected - min(selected);
        end
        
    case 15 % 零共模电压
        vcc = 2 / N;    % 归一化系数
%         vline = [vabc(1) - vabc(2) vabc(2) - vabc(3) vabc(3) - vabc(1)];
%         vx = (2 * vline(1) - vline(2) - vline(3)) / vcc;   % alpha
%         vy = (vline(2) - vline(3)) / vcc;                 % belta
        vx = (vabc(1) - vabc(2)) / vcc;   % alpha
        vy = -vabc(3) / vcc;              % belta
        bl = [floor(vx-eps) floor(vy-eps)];             % 在整数线/点上的矢量的bl也保证在左下角
        if rem(sum(bl), 2) == 0 % 偶数
            v1 = bl;
            v2 = v1 + 1;
        else
            v1 = [ceil(vx) floor(vy)];  % 右下
            v2 = v1 + [-1 1];           % 左上
        end
        v0 = (v1 + v2) / 2;     % 中点
        k = (-(v2(1) - v1(1)) / (v2(2) - v1(2))) / 3;   % 斜率
        if vy > k * (vx - v0(1)) + v0(2)        % 分割线
            vo = v2;
        else
            vo = v1;
        end
        k = round(N/2 + (vo(1)+vo(2))/2);   % always int
        selected = [k, k - vo(1), k-(vo(1)+3*vo(2))/2];
        if max(selected) > N
            selected = selected - (max(selected) - N);   %-1留裕量
        elseif min(selected) < 0
            selected = selected - min(selected);
        end
end
if (~all(pre == [0 0 0]) && ~all(pre == selected))
    freq = freq + sum(abs(pre - selected));
    freq_line = freq_line + sum(abs([pre(1)-pre(2),pre(2)-pre(3),pre(3)-pre(1)] - [selected(1)-selected(2),selected(2)-selected(3),selected(3)-selected(1)]));
    freq_com = freq_com + abs((sum(pre)-sum(selected)));
end
pre = selected;
abcstate = selected;
% figure(1);plot(SVCoutput.time, [SVCoutput.signals(1).values(:,1) SVCoutput.signals(2).values(:,1) SVCoutput.signals(3).values(:,1)]);ylim([-5,5])