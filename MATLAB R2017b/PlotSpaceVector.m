function PlotSpaceVector(block)
% Level-2 MATLAB file S-Function for limited integrator demo.
%   Copyright 1990-2009 The MathWorks, Inc.

  setup(block);
  
%endfunction

function setup(block)
  
  %% Register number of dialog parameters   
  block.NumDialogPrms = 8;      % [level vdc rate normal plot hexagon sw disabled]
%   block.DialogPrmsTunable = {'Tunable', 'Tunable', 'Tunable'};
  
  %% Register number of input and output ports
  block.NumInputPorts  = 2;     % [sa, sb, sc; vd, vq]
  block.NumOutputPorts = 0;

  %% Setup functional port properties to dynamically
  %% inherited.
  block.SetPreCompInpPortInfoToDynamic;
%   block.SetPreCompOutPortInfoToDynamic;
  
  block.InputPort(1).DatatypeID   = 0;      % double
  block.InputPort(1).Complexity   = 'Real';
  block.InputPort(1).Dimensions   = 3;      % sa sb sc
  %block.InputPort(1).SamplingMode = 'Sample';
  
  block.InputPort(2).DatatypeID   = 0;      % double
  block.InputPort(2).Complexity   = 'Real';
  block.InputPort(2).Dimensions   = 2;      % vd vq
  
%   block.OutputPort(1).DatatypeID       = 8; % boolean
%   block.OutputPort(1).Complexity       = 'Real';
%   block.OutputPort(1).Dimensions       = 6;
%   block.OutputPort(1).SamplingMode     = 'Sample';  %�ԷǼ̳е�����˿������������ģʽ
  
  %% Set block sample time to continuous
  block.SampleTimes = [0.5e-6 0];   %һ��������Ϊ2e-4��ִ��50��output������Խ��PWM�ź�Խ׼ȷ��1e-6��0.5e-6��
                                    %THD�ܴ�0.10%��Ϊ0.04%���Դ�ģ�鱣��0.02%
  
  %% Setup Dwork


  %% Set the block simStateCompliance to default (i.e., same as a built-in block)
  block.SimStateCompliance = 'DefaultSimState';

  %% Register methods
  block.RegBlockMethod('PostPropagationSetup',    @DoPostPropSetup);  
  block.RegBlockMethod('InitializeConditions',    @InitConditions);  
  block.RegBlockMethod('Outputs',                 @Output);  
  block.RegBlockMethod('SetInputPortSamplingMode',@SetInputPortSamplingMode);
  block.RegBlockMethod('Terminate',                 @Terminate);  
%   block.RegBlockMethod('Terminate',                 @hex);  
%   block.RegBlockMethod('Updata',                  @Update);  
  
%endfunction

function SetInputPortSamplingMode(block, idx, fd)
    block.InputPort(idx).SamplingMode = fd;
    block.InputPort(idx).SamplingMode = fd;
    
%     block.OutputPort(1).SamplingMode = fd;
%     block.OutputPort(2).SamplingMode = fd;
%endfunction
function DoPostPropSetup(block)

  %% Setup Dwork  data work vector �����������ڴ洢block������������볣��

  
  block.NumDworks = 7;
  block.Dwork(1).Name = 'handles'; %��ͷ����������output�и��£�Ϊ�ο���ѹʸ����ͷ�����������������ʾtext���
  block.Dwork(1).Dimensions      = 4;
  block.Dwork(1).DatatypeID      = 0;
  block.Dwork(1).Complexity      = 'Real';
  block.Dwork(1).UsedAsDiscState = false;
  
  block.Dwork(2).Name = 'plotinterval';     % interval
  block.Dwork(2).Dimensions      = 1;
  block.Dwork(2).DatatypeID      = 4;       % int16
  block.Dwork(2).Complexity      = 'Real';
  block.Dwork(2).UsedAsDiscState = false;
  
  block.Dwork(3).Name = 'numstr';     %position text handle
  block.Dwork(3).Dimensions      = 6;
  block.Dwork(3).DatatypeID      = 5;   %uint16�洢char
  block.Dwork(3).Complexity      = 'Real';
  block.Dwork(3).UsedAsDiscState = false;  
  
  block.Dwork(4).Name = 'V1_6';     %position text handle
  block.Dwork(4).Dimensions      = 7;
  block.Dwork(4).DatatypeID      = 0;
  block.Dwork(4).Complexity      = 'complex';
  block.Dwork(4).UsedAsDiscState = false;  
  
  block.Dwork(5).Name = 'switch_state';     % ��һʱ��abc���ƽ�Լ���ƽ�仯
  block.Dwork(5).Dimensions      = 3;   % ����
  block.Dwork(5).DatatypeID      = 0;   
  block.Dwork(5).Complexity      = 'real';
  block.Dwork(5).UsedAsDiscState = false;    
  
  block.Dwork(6).Name = 'periodswitching';     %ÿ���ڿ����л�ʱ��
  block.Dwork(6).Dimensions      = 6;  %�����л�����״̬��ʱ��
  block.Dwork(6).DatatypeID      = 0;   %
  block.Dwork(6).Complexity      = 'real';
  block.Dwork(6).UsedAsDiscState = false; 
  
  block.Dwork(7).Name = 'Flags';     % flag
  block.Dwork(7).Dimensions      = 1;      % ֻ�ܴ洢�������ܾ���
  block.Dwork(7).DatatypeID      = 8;       % bool�洢����״̬
  block.Dwork(7).Complexity      = 'real';
  block.Dwork(7).UsedAsDiscState = false;   
  

  %% Register all tunable parameters as runtime parameters.
%   block.AutoRegRuntimePrms;

%endfunction

function InitConditions(block)
if block.DialogPrm(8).Data == 0     % disable
    global points
    if isempty(points)
        points = containers.Map;
    end
    figure(block.DialogPrm(5).Data)       % set(gcf, 'position', [0 0 100 100]);
    clf
    N = block.DialogPrm(1).Data - 1;  
    k = sqrt(3) / 2;
    
    % plot outline
    outline = N * [0.5-k*1i, 1, 0.5+k*1i, -0.5+k*1i, -1, -0.5-k*1i, 0.5-k*1i, 1, 0.5+k*1i];   % 6 1~6 1 2
    plot(outline(2:8), 'linewidth', 2) %plot the outline
    hold on
    
    % plot dashed line
    linecolor = [1 1 1] * 0.2;
    for i = 2:7     % 1~6
        for j = 1:N-1
            v1 = (j * outline(i-1) + (N-j) * outline(i)) / N;
            v2 = (j * outline(i+2) + (N-j) * outline(i+1)) / N;
            plot([v1, v2], '--', 'Color', linecolor)
        end
    end
    for i = 3:4
        plot([outline(i), outline(i+3)], '--', 'Color', linecolor)
    end
    plot([-N, N], [0, 0], '--', 'Color', linecolor)
    axis equal
    
%     theta = linspace(pi/6,13*pi/6,7);   % ȫ��������
%     hexx = cos(theta)/sqrt(3);
%     hexy = sin(theta)/sqrt(3);
%     if block.DialogPrm(6).Data
%         for yy = 0 : N
%             for xx = yy/2 - N : N - yy/2
%                 plot(hexx+xx,hexy+yy*sqrt(3)/2,'r-','LineWidth',1);
%                 plot(hexx+xx,hexy-yy*sqrt(3)/2,'r-','LineWidth',1);
%             end
%         end
%     end

    % save handles of vector: [Vref, Vuse]
    block.Dwork(1).Data = [double(quiver(0, 0, 0, 0, 1, 'Color', 'r', 'LineWidth', 1.5, 'MaxHeadSize', 0.5)), ...
        double(quiver(0, 0, 0, 0, 1, 'Color', 'k', 'LineWidth', 1, 'MaxHeadSize', 0.25)), double(gca), 0];
    block.Dwork(2).Data = int16(0);
    block.Dwork(5).Data = [0 0 0];
    block.Dwork(7).Data(1) = false;
end
     
function Output(block)
if block.DialogPrm(8).Data == 0     % disable
    if block.DialogPrm(4).Data == 1 % �ѹ�һ��
        N = block.DialogPrm(1).Data - 1;
        Valpha = block.InputPort(2).Data(1) / (2/3*2/N);
        Vbeta = block.InputPort(2).Data(2) / (2/3*2/N); 
    else
        vc = block.DialogPrm(2).Data;
        Valpha = block.InputPort(2).Data(1) / (2/3*vc);
        Vbeta = block.InputPort(2).Data(2) / (2/3*vc); 
    end
    sa = block.InputPort(1).Data(1);
    sb = block.InputPort(1).Data(2);
    sc = block.InputPort(1).Data(3);
    x = sa - (sb + sc) / 2;
    y = (sb - sc) * sqrt(3) / 2;
    xy = strcat(num2str(x), ',', num2str(y), ',', num2str(block.DialogPrm(5).Data));
    
    if(block.Dwork(2).Data == block.DialogPrm(3).Data)  % ˢ����
        set(block.Dwork(1).Data(1), 'UData', Valpha, 'VData', Vbeta)
        set(block.Dwork(1).Data(2), 'UData', x, 'VData', y)
        if block.DialogPrm(7).Data  % ��ʾ����״̬ʱ��ʾǳ�켣
%             plot(block.Dwork(1).Data(3), Valpha, Vbeta, 'Marker', '.', 'MarkeredgeColor', [1 1 1]*0.2, 'markersize', 6)
            % �����л�ʵʱ��ʾ
%             plot(block.Dwork(1).Data(3), [Valpha x], [Vbeta y], 'k-', 'LineWidth', 0.6)
            plot(block.Dwork(1).Data(3), Valpha, Vbeta, 'k.', 'markersize', 5)
        else
%             plot(block.Dwork(1).Data(3), [Valpha x], [Vbeta y], 'k-', 'LineWidth', 1)
%             plot(block.Dwork(1).Data(3), Valpha, Vbeta, 'Marker', '.', 'MarkeredgeColor', [1 1 1]*0.2, 'markersize', 6)
            plot(block.Dwork(1).Data(3), Valpha, Vbeta, 'k.', 'markersize', 10)
        end
        block.Dwork(2).Data = int16(0);
    end
    
    
    if block.Dwork(7).Data(1) == false  % ��һ����δ��ʱ
        % show switching
        if block.DialogPrm(7).Data
            if any(block.InputPort(1).Data ~= block.Dwork(5).Data)   % ����ʱ�̿���״̬�仯
%                 quiver(Valpha, Vbeta, x-Valpha, y-Vbeta, 1, 'Color', 'g', 'LineWidth', 2, 'MaxHeadSize', 10, 'marker', '*', 'markerfacecolor', 'g', 'markersize', 5, 'markeredgecolor', 'k')
                % �����л����ӻ�
                if block.CurrentTime ~= 0
                    plot(block.Dwork(1).Data(3), [Valpha x], [Vbeta y], 'k-', 'LineWidth', 2)
                    plot(block.Dwork(1).Data(3), Valpha, Vbeta, 'ko', 'LineWidth', 2, 'markerfacecolor', 'g', 'markersize', 8, 'markeredgecolor', 'k')
                    plot(block.Dwork(1).Data(3), x, y, 'k.', 'markersize', 10)
                end
%                 plot(block.Dwork(1).Data(3), Valpha, Vbeta, 'o', 'markerfacecolor', 'g', 'markersize', 5, 'markeredgecolor', 'k')
                block.Dwork(5).Data = block.InputPort(1).Data;
            end
        end
        global points
        if points.isKey(xy)
            points(xy) = points(xy) + 1;
        else
            points(xy) = 1;
        end
        if abs(block.CurrentTime - 0.02) <= 2e-7    % 0.02sʱ��ɢ��ͼ��������
            keys = points.keys;
            xys = zeros(points.Count, 3);
            for i = 1:points.Count
                xyfig = str2num(cell2mat(keys(i)));
                if xyfig(3) == block.DialogPrm(5).Data  % ֻѡȡ��ǰplot�ĵ�
                    xys(i,:) = [xyfig(1:2) points(cell2mat(keys(i)))];
                    points.remove(cell2mat(keys(i)));
                end
            end
            xys(all(xys==0, 2), :) = [];
            theta = linspace(pi/6,13*pi/6,7);
            hexx = cos(theta)/sqrt(3);
            hexy = sin(theta)/sqrt(3);
            if block.DialogPrm(6).Data
                for i = 1 : length(xys)
                        plot(block.Dwork(1).Data(3), hexx+xys(i,1), hexy+xys(i,2), 'r-', 'LineWidth', 1);   % ������
                end
            end
            if ~block.DialogPrm(7).Data
            % ɢ��ͼ
                block.Dwork(1).Data(4) = scatter(block.Dwork(1).Data(3), xys(:,1),xys(:,2),xys(:,3)/2, 'r', 'filled', 'MarkerFaceAlpha', 0.5);
            end
                % size can be changed: set(block.Dwork(1).Data(4), 'SizeData', get(block.Dwork(1).Data(4),'SizeData')/2)
            % or gca; ch = ans.Children; a=ch(1); a.SizeData = a.SizeData/3
            set(block.Dwork(1).Data(1), 'Visible','off')
            set(block.Dwork(1).Data(2), 'Visible','off')
            set(block.Dwork(1).Data(3), 'Visible','off')
            block.Dwork(7).Data(1) = true;  % һ���ڻ�ͼ����
        end
    %     drawnow limitrate
        block.Dwork(2).Data = block.Dwork(2).Data + 1;  % counter
    end
end
% endfunction
    
function Terminate(block)
if block.DialogPrm(8).Data == 0     % disable
    if block.Dwork(7).Data(1) == false  % ��һ����δ��ʱֹͣ����ɾ����ǰplot����ɢ��
        global points
        keys = points.keys;
        for i = 1:points.Count
            xyfig = str2num(cell2mat(keys(i)));
            if xyfig(3) == block.DialogPrm(5).Data
                points.remove(cell2mat(keys(i)));
            end
        end
    end
end