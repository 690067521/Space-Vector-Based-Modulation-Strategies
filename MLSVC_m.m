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
        vcc = 2 / N;    % ��һ��ϵ��
        vx = (2 * vabc(1) - vabc(2) - vabc(3)) / vcc;   % alpha
        vy = (vabc(2) - vabc(3)) / vcc;                 % belta
        bl = [floor(vx-eps) floor(vy-eps)];             % ��������/���ϵ�ʸ����blҲ��֤�����½�
        if rem(sum(bl), 2) == 0 % ż��
            v1 = bl;
            v2 = v1 + 1;
        else
            v1 = [ceil(vx) floor(vy)];  % ����
            v2 = v1 + [-1 1];           % ����
        end
        v0 = (v1 + v2) / 2;     % �е�
        k = (-(v2(1) - v1(1)) / (v2(2) - v1(2))) / 3;   % б��
        if vy > k * (vx - v0(1)) + v0(2)        % �ָ���
            vo = v2;
        else
            vo = v1;
        end
        k = round(N/2 + vo(1)/3);   % round��Ϊֻ��һ������״̬��vo2������
        selected = [k, k - (vo(1) - vo(2)) / 2, k - (vo(1) + vo(2)) / 2];
        if max(selected) > N
            selected = selected - (max(selected) - N);   %-1��ԣ��
        elseif min(selected) < 0
            selected = selected - min(selected);
        end
        
    case 2  % x-y,����
        global selected;
        if isempty(selected)
            selected = [0 0 0];
        end
        vcc = 2 / N;    % ��һ��ϵ��
        origin = (selected-sum(selected)/3)*vcc;
        T = 1 / vcc * [2 -1 -1; 0 1 -1];
        vXY = T * (vabc - origin'); % = T * vabc - T*origin, ����ȥԭselected��xy����ϵ�µ�ֵ����
        vx = vXY(1);        % alpha
        vy = vXY(2);        % belta
        % {
        if (abs(vx) > 1 || abs(vy-vx/3) > 2/3 || abs(vy+vx/3) > 2/3)    % ���ж�������Խ�����ж�����
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
        if (vx >= 0)    % ʹ������ȡ���ķ���ͬ����Ҫ�ж����Ǹ���������ԭʼ�ķ���
            if (vx > 1 && -vx/3 < vy && vy <= vx/3)   % area1, �������߽绮��
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
            k = round(N/2 - sum(selected)/3);   % round��Ϊֻ��һ������״̬��vo2������
            selected = selected + k;
            if max(selected) > N
                selected = selected - (max(selected) - N);   %-1��ԣ��
            elseif min(selected) < 0
                selected = selected - min(selected);
            end
        end
        
    case 3  % HD
        vcc = 2 / N;    % ��һ��ϵ��
        T = 1 / vcc * [1 -1 0; 1 0 -1];
        vhd = T * vabc;
        v0h = floor(vhd(1));
        v0d = floor(vhd(2));
        hx = vhd(1) - v0h;
        dx = vhd(2) - v0d;
        vdif = hx - dx;
        t1 = sum(vhd);
        v0sum = v0h + v0d;
        if (vdif >= 0)    % �������Σ� ȷ��������֮��Ͳ�����ż��
            if (t1 < v0sum + 1 && dx >= hx*2-1)    % left & high
                    vo = [v0h v0d];
            elseif (t1 >= v0sum + 1 && dx >= hx/2)    % right & high
                    vo = [v0h+1 v0d+1];
            else
                    vo = [v0h+1 v0d];
            end
        else        % ������
            if (t1 <= v0sum + 1 && dx <= hx/2+0.5)    % left & low
                    vo = [v0h v0d];
            elseif (t1 >= v0sum + 1 && dx <= hx*2)    % right & low
                    vo = [v0h+1 v0d+1];
            else
                    vo = [v0h v0d+1];
            end
        end
        selected = [0 -vo(1) -vo(2)];
        k = round(N/2 - sum(selected)/3);   % round��Ϊֻ��һ������״̬��vo2������
        selected = selected + k;
        if max(selected) > N
            selected = selected - (max(selected) - N);   %-1��ԣ��
        elseif min(selected) < 0
            selected = selected - min(selected);
        end
        
    case 4  % HD,����
        global selected;
        if isempty(selected)
            selected = [0 0 0];
        end
        vcc = 2 / N;    % ��һ��ϵ��
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
            k = round(N/2 - sum(selected)/3);   % round��Ϊֻ��һ������״̬��vo2������
            selected = selected + k;
            if max(selected) > N
                selected = selected - (max(selected) - N);   %-1��ԣ��
            elseif min(selected) < 0
                selected = selected - min(selected);
            end
        end
        
    case 5  % BJ120
        vcc = 2 / N;    % ��һ��ϵ��
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
        k = round(N/2 + vo(1)/3);   % round��Ϊֻ��һ������״̬��vo2������
        selected = selected + k;
        if max(selected) > N
            selected = selected - (max(selected) - N);   %-1��ԣ��
        elseif min(selected) < 0
            selected = selected - min(selected);
        end

    case 6  % BJ120������ϵ��������
        global selected;
        if isempty(selected)
            selected = [0 0 0];
        end
        vcc = 2 / N;    % ��һ��ϵ��
        origin = (selected-sum(selected)/3)*vcc;
        %{
        % ������ϵ����任
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
        % ���������ж�
        T = 1 / vcc * [2 -1 -1;1 1 -2];
        vBJ = T * (vabc - origin');
        if (abs(vBJ(1)) > 1 || abs(vBJ(2)) > 1 || abs(vBJ(2)-vBJ(1)) > 1)   % edges
            change = [1 0 0; 0 0 -1; 0 1 0; -1 0 0; 0 0 1; 0 -1 0]; % [vBJ(1)>=0, vBJ(2)>=0, vBJ(2)>=vBJ(1)]
            if (vBJ(2) >= 0 && vBJ(2) < vBJ(1))    % area1, [* 1 0] [1 * 1] [0 1 *] [* 0 1] [0 * 0] [1 0 *], ��[0 1 0][1 0 1]
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
            k = round(N/2 - sum(selected)/3);   % round��Ϊֻ��һ������״̬��vo2������
            selected = selected + k;
            if max(selected) > N
                selected = selected - (max(selected) - N);   %-1��ԣ��
            elseif min(selected) < 0
                selected = selected - min(selected);
            end
        end
        
    case 7  % BJ60������ϵ
        vcc = 2 / N;    % ��һ��ϵ��
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
        k = round(N/2 + (vo(1)+vo(2))/3);   % round��Ϊֻ��һ������״̬
        selected = selected + k;
        if max(selected) > N
            selected = selected - (max(selected) - N);   %-1��ԣ��
        elseif min(selected) < 0
            selected = selected - min(selected);
        end   
        
    case 8  % BJ60������ϵ��������
        global selected;
        if isempty(selected)
            selected = [0 0 0];
        end
        vcc = 2 / N;    % ��һ��ϵ��
        origin = (selected-sum(selected)/3)*vcc;    % state to three-phase
        % {
        % ����ϵ�����ж������ķ������ܸ�Ϊ��
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
        % ͨ�������ж������������任
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
            k = round(N/2 - sum(selected)/3);   % round��Ϊֻ��һ������״̬��vo2������
            selected = selected + k;
            if max(selected) > N
                selected = selected - (max(selected) - N);   %-1��ԣ��
            elseif min(selected) < 0
                selected = selected - min(selected);
            end
        end
        
    case 9  % ABC
        vcc = 2 / N;    % ��һ��ϵ��
        vABC = 3 / vcc * vabc;
        fABC = floor(vABC);
        cABC = ceil(vABC);
        if (mod(cABC(3) + 2*fABC(1), 3) == 0)       % cc+2fa=3k�����Ͻ�����
            vo = [fABC(1) -fABC(1)-cABC(3) cABC(3)];
        elseif (mod(cABC(1) + 2*fABC(2), 3) == 0)   % ca+2fb=3k�����Ͻ�����
            vo = [cABC(1) fABC(2) -cABC(1)-fABC(2)];
        else
%             (mod(cABC(2) - fABC(3), 3) == 0)     % cb-fc=3k,
%             ֻ������������ʱ�Ȼ�������·�����
            vo = [-cABC(2)-fABC(3) cABC(2) fABC(3)];
        end
        selected = [0 (vo(2)-vo(1))/3 -(vo(2)+2*vo(1))/3];
        k = round(N/2 + vo(1)/3);   % round��Ϊֻ��һ������״̬������֮��Ϊ0
        selected = selected + k;
        if max(selected) > N
            selected = selected - (max(selected) - N);   %-1��ԣ��
        elseif min(selected) < 0
            selected = selected - min(selected);
        end
        
    case 10 % ABC����ϵ��������
        global selected;
        if isempty(selected)
            selected = [0 0 0];
        end
        vcc = 2 / N;    % ��һ��ϵ��
        origin = (selected-sum(selected)/3)*vcc;
        vABC = 3 / vcc * (vabc - origin');
        % {  
        % ��������
        if (abs(vABC(1)) > 1 || abs(vABC(2)) > 1 || abs(vABC(3)) > 1)
            change = [1 0 0; 0 0 -1; 0 1 0; -1 0 0; 0 0 1; 0 -1 0];
            if (vABC(3) < 0 && vABC(2) < 0)         % area 1��ÿ���������������������
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
        [maxv, maxi] = max(vABC);   % ͬ��������ᣬ��������ΪABC�ĸ�ʸ������ȡ���ֵ����
        [minv, mini] = min(vABC);
        if (maxv > 1 || minv < -1)
            change = [1 0 0; 0 0 -1; 0 1 0; -1 0 0; 0 0 1; 0 -1 0];
            if (maxv > -minv)           % max value direction
                selected = selected + change(maxi*2-1, :);  % 1 3 5
            else                        % min value negtive direction
                selected = selected + change(mod(mini*2, 6)+2, :);  % 4 6 2
            end
        %}
            k = round(N/2 - sum(selected)/3);   % round��Ϊֻ��һ������״̬��vo2������
            selected = selected + k;
            if max(selected) > N
                selected = selected - (max(selected) - N);   %-1��ԣ��
            elseif min(selected) < 0
                selected = selected - min(selected);
            end
        end
        
    case 11 % A-CB-AC-B��������ϵ�����ܼ����Ľ�
        vcc = 2 / N;    % ��һ��ϵ��
        T = 3 / vcc * [1 0 0; 0 0 -1; 0 1 0; -1 0 0; 0 0 1; 0 -1 0];    % [A -C B -A C -B]
        vABC = T * vabc;
        fABC = floor(vABC);
        if (mod(2*fABC(1) - fABC(2), 3) == 0)       % 2fa-f(-c)=3k�����Ͻ�����
            vo = [fABC(1) fABC(2) fABC(2)-fABC(1) -fABC(1) -fABC(2) fABC(1)-fABC(2)];
        elseif (mod(2*fABC(3)- fABC(4), 3) == 0)    % 2fb-f(-a)=3k�����Ͻ�����
            vo = [-fABC(4) fABC(3)-fABC(4) fABC(3) fABC(4) fABC(4)-fABC(3) -fABC(3)];
        else   % (mod(-fABC(6) - fABC(5), 3) == 0)   % -f(-b)-fc=3k,ֻ������������ʱ�Ȼ�������·�����
            vo = [fABC(6)-fABC(5) -fABC(5) -fABC(6) fABC(5)-fABC(6) fABC(5) fABC(6)];
        end
        selected = [0 (vo(3)+vo(4))/3 (-vo(3)+2*vo(4))/3];
        k = round(N/2 + vo(1)/3);   % round��Ϊֻ��һ������״̬��vo2������
        selected = selected + k;
        if max(selected) > N
            selected = selected - (max(selected) - N);   %-1��ԣ��
        elseif min(selected) < 0
            selected = selected - min(selected);
        end
        
    case 12 % A-CB-AC-B��������ϵ��������
        global selected;
        if isempty(selected)
            selected = [0 0 0];
        end
        vcc = 2 / N;    % ��һ��ϵ��
        origin = (selected-sum(selected)/3)*vcc;
        T = 3 / vcc * [1 0 0; 0 0 -1; 0 1 0; -1 0 0; 0 0 1; 0 -1 0];
        vABC = T * (vabc - origin');
        % {
        % ��������
        if (any(vABC > 1))
            change = [1 0 0; 0 0 -1; 0 1 0; -1 0 0; 0 0 1; 0 -1 0];
            if (vABC(2) >= 0 && vABC(3) < 0)         % area 1��ÿ���������������������
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
        [maxv, maxi] = max(vABC);   % ���ᣬ��������ΪABC�ĸ�ʸ������ȡ���ֵ����
        if (maxv > 1)
            change = [1 0 0; 0 0 -1; 0 1 0; -1 0 0; 0 0 1; 0 -1 0];
            selected = selected + change(maxi, :);
            %}
            k = round(N/2 - sum(selected)/3);   % round��Ϊֻ��һ������״̬��vo2������
            selected = selected + k;
            if max(selected) > N
                selected = selected - (max(selected) - N);   %-1��ԣ��
            elseif min(selected) < 0
                selected = selected - min(selected);
            end
        end
        
    case 13 % ��λHD������λ������Ӧ���ڵ�������
        vcc = 2 / N;    % ��һ��ϵ��
        T = 1 / vcc * [1 -1 0; 1 0 -1];
        vhd = T * vabc - [-1/3 -2/3]';
        v0h = floor(vhd(1));
        v0d = floor(vhd(2));
        hx = vhd(1) - v0h;
        dx = vhd(2) - v0d;
        selected = [0 -v0h -v0d];
%         vo = [v0h v0d];
        if (hx >= dx)   % ������
            if (hx+dx < 1 && dx < hx/2)  % left & low
                selected(3) = selected(3) + 1;
%                 vo = [v0h v0d-1];
            elseif (hx+dx >= 1 && dx+1 < hx*2) % right & low
                selected(2) = selected(2) - 1;
%                 vo = [v0h+1 v0d];
            end     % else remain selected
        end
        k = round(N/2 - sum(selected)/3);   % round��Ϊֻ��һ������״̬��vo2������
        selected = selected + k;
        if max(selected) > N
            selected = selected - (max(selected) - N);   %-1��ԣ��
        elseif min(selected) < 0
            selected = selected - min(selected);
        end
        
    case 14 % ��λBJ60
        vcc = 2 / N;    % ��һ��ϵ��
        T = 0.5 / vcc * [1 -2 1; 1 1 -2];
        vBJ = T * vabc - [-0.5 -0.5]';
        v0B = floor(vBJ(1));
        v0J = floor(vBJ(2));
        Bx = vBJ(1) - v0B;
        Jx = vBJ(2) - v0J;
        switch (mod(v0B-v0J, 3))
            case 0
                vo = [v0B+0.5 v0J+0.5];   % �� ֱ��ȡshift֮ǰ���꣬��ʡ����
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
        k = round(N/2 + (vo(1)+vo(2))/3);   % round��Ϊֻ��һ������״̬��vo2������
        selected = selected + k;
        if max(selected) > N
            selected = selected - (max(selected) - N);   %-1��ԣ��
        elseif min(selected) < 0
            selected = selected - min(selected);
        end
        
    case 15 % �㹲ģ��ѹ
        vcc = 2 / N;    % ��һ��ϵ��
%         vline = [vabc(1) - vabc(2) vabc(2) - vabc(3) vabc(3) - vabc(1)];
%         vx = (2 * vline(1) - vline(2) - vline(3)) / vcc;   % alpha
%         vy = (vline(2) - vline(3)) / vcc;                 % belta
        vx = (vabc(1) - vabc(2)) / vcc;   % alpha
        vy = -vabc(3) / vcc;              % belta
        bl = [floor(vx-eps) floor(vy-eps)];             % ��������/���ϵ�ʸ����blҲ��֤�����½�
        if rem(sum(bl), 2) == 0 % ż��
            v1 = bl;
            v2 = v1 + 1;
        else
            v1 = [ceil(vx) floor(vy)];  % ����
            v2 = v1 + [-1 1];           % ����
        end
        v0 = (v1 + v2) / 2;     % �е�
        k = (-(v2(1) - v1(1)) / (v2(2) - v1(2))) / 3;   % б��
        if vy > k * (vx - v0(1)) + v0(2)        % �ָ���
            vo = v2;
        else
            vo = v1;
        end
        k = round(N/2 + (vo(1)+vo(2))/2);   % always int
        selected = [k, k - vo(1), k-(vo(1)+3*vo(2))/2];
        if max(selected) > N
            selected = selected - (max(selected) - N);   %-1��ԣ��
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