function abcstate = MLNLC_m(nlcinput)

% input: [va, vb, vc, voltage level, sampling frequency]
% output: [sa, sb, sc] ,range(0~level-1)

vabc = nlcinput(1:3);
N = nlcinput(4) - 1;        % number of carrier waves
fs = nlcinput(5);
mode_nlc = nlcinput(6);      % PWM Mode
line_nlc = nlcinput(7);     % line mode
t = get_param('mysvm_NLC','SimulationTime');
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

if N > 0
    if line_nlc == 1
        vline = [vabc(1)-vabc(2) vabc(2)-vabc(3) vabc(3)-vabc(1)] * N/2;  % [vab vbc vca]
    %     temp = [vline(1) -vline(3) vline(2) -vline(1) vline(3) -vline(2)];
    %     [maxv, maxi] = max(temp);
        line = round(vline);    % round对-0.5也取为-1，因此正负不同时电平变化时刻不同，应保证0.5就跳变，可改变相位来实现
        selected = round([(line(1)-line(3)) (line(2)-line(1)) (line(3)-line(2))] / 3.0) + N/2;
    %     selected = [0 -line(1) line(3)];
    %     selected = [line(1) 0 -line(2)];
    %     selected = [-line(3) line(2) 0];
    %     switch maxi
    %         case {1,4}
    %             selected = [-line(3) line(2) 0];
    %         case {2,5}
    %             selected = [line(1) 0 -line(2)];
    %         case {3,6}
    %             selected = [0 -line(1) line(3)];
    %     end
    %     selected = selected + round(N/2 - sum(selected)/3);

        if max(selected) > N
            selected = selected - (max(selected) - N);   %-1留裕量
        elseif min(selected) < 0
            selected = selected - min(selected);
        end
        if (~all(pre == [0 0 0]) && ~all(pre == selected))
            freq = freq + sum(abs(pre - selected));
            freq_line = freq_line + sum(abs([pre(1)-pre(2),pre(2)-pre(3),pre(3)-pre(1)] - [selected(1)-selected(2),selected(2)-selected(3),selected(3)-selected(1)]));
            freq_com = freq_com + abs((sum(pre)-sum(selected)));
        end
        pre = selected;
        abcstate = selected;
    else
        if mode_nlc == 1    % PWM
            time = rem(t*fs, 1);
            v_nor = (vabc+1) * N / 2;
            vl = floor(v_nor);
            d = v_nor - vl;
            abcstate = vl + (time < d) .* [1; 1; 1];
        else
    %         global prestate_nlc
    %         if rem(t, 1/fs) < 1e-6
    %             selected = round((vabc)*N/2);    % round对-0.5也取为-1，因此正负不同时电平变化时刻不同，应保证0.5就跳变，可改变相位来实现
    %         else
    %             selected = prestate_nlc;
    %         end
    %         prestate_nlc = selected;
    %         abcstate = selected + N/2;
            selected = round(vabc'*N/2) + fix(N/2); % 初始化导致错误
            if max(selected) > N
                selected = selected - (max(selected) - N);   %-1留裕量
            elseif min(selected) < 0
                selected = selected - min(selected);
            end
            if (~all(pre == [0 0 0]) && ~all(pre == selected))
                freq = freq + sum(abs(pre - selected));
                freq_line = freq_line + sum(abs([pre(1)-pre(2),pre(2)-pre(3),pre(3)-pre(1)] - [selected(1)-selected(2),selected(2)-selected(3),selected(3)-selected(1)]));
                freq_com = freq_com + abs((sum(pre)-sum(selected)));
            end
            pre = selected;
            abcstate = selected;
        end
    end
end