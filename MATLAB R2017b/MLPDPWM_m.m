function abcstate = MLPDPWM_m(pdpwminput)

% input: [va, vb, vc, voltage level, switching frequency, modulation ratio]
% output: [sa, sb, sc] ,range(0~level-1)

vabc = pdpwminput(1:3);
N = pdpwminput(4) - 1;        % number of carrier waves
fs = pdpwminput(5);           % updata 10 times in a sector, 10*6*50=3e3
method = pdpwminput(6); 
Ts = 1 / fs;
t = get_param('mysvm_pdpwm','SimulationTime');

% abcstate = zeros(1, 10);  % 不能设为N，不知为何
abcstate = zeros(1, 3);
compares = zeros(3, N);
switch method
    case 1  % PD
        bias = -1 : 2/N : 1-1e-6;  % N个间隔
        for phase = 1:3
            for n = 1:N
                tt = mod(t, Ts);
                if tt < Ts / 2
%                     abcstate(n) = (tt / Ts * 4 / N + bias(n));
                    compares(phase, n) = (tt / Ts * 4 / N + bias(n)) < vabc(phase);
                else
%                     abcstate(n) = (- tt / Ts * 4 / N + 4 / N + bias(n));
                    compares(phase, n) = (- tt / Ts * 4 / N + 4 / N + bias(n)) < vabc(phase);
                end        
            end
            abcstate(phase) = sum(compares(phase, :));
        end
        
    case 2  % POD
        bias = [-1 : 2/N : -1e-6, -2/N : 2/N : 1-1e-6];  % N个间隔
        for phase = 1:3
            for n = 1:N
                tt = mod(t, Ts);
                if xor(tt < Ts / 2, n > N / 2)
%                     abcstate(n) = (tt / Ts * 4 / N + bias(n));
                    compares(phase, n) = (tt / Ts * 4 / N + bias(n)) < vabc(phase);
                else
%                     abcstate(n) = (- tt / Ts * 4 / N + 4 / N + bias(n));
                    compares(phase, n) = (- tt / Ts * 4 / N + 4 / N + bias(n)) < vabc(phase);
                end        
            end
            abcstate(phase) = sum(compares(phase, :));
        end
        
    case 3  % APOD
        bias = sort([-1 : 4/N : 1-1e-6, -1+1e-6 : 4/N : 1]);  % N个间隔
        for phase = 1:3
            for n = 1:N
                tt = mod(t, Ts);
                if xor(tt < Ts / 2, mod(n, 2) == 0)
%                     abcstate(n) = (tt / Ts * 4 / N + bias(n));
                    compares(phase, n) = (tt / Ts * 4 / N + bias(n)) < vabc(phase);
                else
%                     abcstate(n) = (- tt / Ts * 4 / N + 4 / N + bias(n));
                    compares(phase, n) = (- tt / Ts * 4 / N + 4 / N + bias(n)) < vabc(phase);
                end        
            end
            abcstate(phase) = sum(compares(phase, :));
        end
end