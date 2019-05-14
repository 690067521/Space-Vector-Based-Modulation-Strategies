function abcstate = MLSPWM_m(spwminput)

% input: [va, vb, vc, voltage level, switching frequency, modulation ratio]
% output: [sa, sb, sc] ,range(0~level-1)

vabc = spwminput(1:3);
% N_ps = spwminput(4) - 1;        % number of carrier waves
global N    % 初始化时才能使用N达到输出变长的效果
N_ps = N - 1;
fs = spwminput(5);           % updata 10 times in a sector, 10*6*50=3e3
global mode_ps
Ts = 1 / fs;
t = get_param('mysvm_spwm','SimulationTime');
abcstate = zeros(1, 13);
if mode_ps == 1     % 仅当mode_ps赋值为1时，仅展示A相
    intervals = 0: Ts/N_ps :Ts-eps;  % N个间隔
    compares = zeros(1, N_ps);
    for n = 1:N_ps
        tt = mod(t + intervals(n), Ts);
        if tt < Ts / 2
            abcstate(n) = (tt / Ts * 4 - 1);
            compares(n) = abcstate(n) < vabc(1);
        else
            abcstate(n) = (- tt / Ts * 4 + 3);
            compares(n) = abcstate(n) < vabc(1);
        end
    end
    abcstate(N_ps+1) = vabc(1);
    abcstate(N_ps+2) = sum(compares(:))/N_ps;
else
    abcstate = zeros(1, 3);
    intervals = 0: Ts/N_ps :Ts-eps;  % N_ps个间隔
    compares = zeros(3, N_ps);
    for phase = 1:3
        for n = 1:N_ps
            tt = mod(t + intervals(n), Ts);
            if tt < Ts / 2
                compares(phase, n) = (tt / Ts * 4 - 1) < vabc(phase);
            else
                compares(phase, n) = (- tt / Ts * 4 + 3) < vabc(phase);
            end
        end
        abcstate(phase) = sum(compares(phase, :));
    end
end