%% 吉布斯效应可视化 (Gibbs Phenomenon Visualization)
clear; clc; close all;

%% 图1：时域表现 - 为什么方波会有“尖角”？
% 原理：用有限项傅里叶级数合成方波
t = linspace(-1.5, 1.5, 10000);
y_approx = zeros(size(t));

% 尝试合成的谐波数量 (你可以修改这个数，比如 5, 20, 100)
% 你会发现，即使 N 很大，尖角依然存在
N_harmonics = 50; 

for k = 1:2:(2*N_harmonics-1)
    y_approx = y_approx + (4/pi) * (1/k) * sin(2*pi*k*t);
end

figure('Color', 'w', 'Position', [100, 100, 800, 400]);
plot(t, y_approx, 'b', 'LineWidth', 1.5); hold on;
yline(1, 'r--', '理想方波顶', 'LineWidth', 1);
yline(-1, 'r--', '理想方波底', 'LineWidth', 1);
title(['时域吉布斯效应 (合成谐波数量 N=', num2str(N_harmonics), ')']);
xlabel('时间 t'); ylabel('幅值');
grid on;
% 标注那个著名的“尖角”
text(0.25, 1.1, '\leftarrow 永远存在的 9% 过冲', 'FontSize', 12, 'Color', 'r');

%% 图2：频域表现 - 滤波器设计中的纹波
% 比较：直接截断(矩形窗) vs 平滑截断(汉明窗)
N_tap = 64;   % 滤波器阶数
Wn = 0.4;     % 截止频率

% 1. 使用矩形窗 (Rectangular) -> 吉布斯效应明显
b_rect = fir1(N_tap, Wn, rectwin(N_tap+1));
[h_rect, w] = freqz(b_rect, 1, 512);

% 2. 使用汉明窗 (Hamming) -> 抑制吉布斯效应
b_hamm = fir1(N_tap, Wn, hamming(N_tap+1));
[h_hamm, ~] = freqz(b_hamm, 1, 512);

figure('Color', 'w', 'Position', [100, 550, 800, 400]);
plot(w/pi, 20*log10(abs(h_rect)), 'r', 'LineWidth', 1.5); hold on;
plot(w/pi, 20*log10(abs(h_hamm)), 'b', 'LineWidth', 1.5);
title('频域吉布斯效应对比：矩形窗 vs 汉明窗');
legend('矩形窗 (直接截断) - 纹波大', '汉明窗 (平滑处理) - 纹波小');
xlabel('归一化频率 (\times \pi rad/sample)');
ylabel('幅度 (dB)');
grid on; ylim([-100, 10]);

% 标注关键点
text(0.1, 2, '\leftarrow 通带纹波 (Gibbs)', 'Color', 'r');
text(0.5, -40, '\leftarrow 旁瓣很高 (阻带切不干净)', 'Color', 'r');
%% 滤波器大对决：矩形窗(吉布斯效应) vs 汉明窗(平滑处理)

% --- 1. 滤波器设计参数 ---
Fs = 1000;          % 采样率 1000 Hz
Fn = Fs / 2;        % 奈奎斯特频率
Fc = 100;           % 截止频率 100 Hz
N  = 100;           % 滤波器阶数 (阶数越高，延迟越大)

% 归一化截止频率
Wn = Fc / Fn;

% --- 2. 设计两个滤波器 ---

% A. "糟糕"的滤波器：矩形窗 (Rectangular Window)
% 这种设计方法直接截断了 Sinc 函数，完全不考虑吉布斯效应
% 虽然它在频域下降得最快，但在时域震荡最严重
b_rect = fir1(N, Wn, rectwin(N+1));

% B. "理想/实用"的滤波器：汉明窗 (Hamming Window)
% 这种设计使用了平滑的窗函数，牺牲了一点陡峭度，但消除了震荡
b_hamm = fir1(N, Wn, hamming(N+1));

% --- 3. 生成测试信号 (阶跃信号) ---
t = -0.05 : 1/Fs : 0.15; % 时间轴
u = zeros(size(t));
u(t >= 0) = 1;           % 0时刻发生阶跃

% --- 4. 滤波处理 ---
% 使用 filter 函数进行滤波
y_rect = filter(b_rect, 1, u);
y_hamm = filter(b_hamm, 1, u);

% --- 关键步骤：修正相位延迟 ---
% FIR 滤波器的群延迟通常为 N/2 个采样点
% 为了让波形对齐在 t=0 时刻，我们需要把波形往回平移
delay = N / 2;
y_rect_shifted = y_rect(delay+1:end); 
y_hamm_shifted = y_hamm(delay+1:end);
% 补齐长度以便画图
t_plot = t(1:length(y_rect_shifted));

% --- 5. 画图对比 ---
figure('Color', 'w', 'Position', [100, 100, 1000, 700]);

% === 子图 1: 时域阶跃响应 (看震荡) ===
subplot(2,1,1);
plot(t, u, 'k--', 'LineWidth', 1); hold on;
plot(t_plot, y_rect_shifted, 'r', 'LineWidth', 1.5);
plot(t_plot, y_hamm_shifted, 'b', 'LineWidth', 2);

title('时域对比：阶跃响应 (Step Response)');
xlabel('时间 (s)'); ylabel('幅度');
legend('理想输入', '矩形窗 (吉布斯效应严重)', '汉明窗 (平滑无震荡)', 'Location', 'SouthEast');
grid on; axis([-0.02 0.1 -0.2 1.2]);

% 标注震荡
text(0.005, 1.12, '\leftarrow 9% 剧烈过冲', 'Color', 'r', 'FontSize', 10);
text(0.015, 0.95, '\leftarrow 平滑稳定', 'Color', 'b', 'FontSize', 10);

% === 子图 2: 频域幅频响应 (看陡峭度与纹波) ===
subplot(2,1,2);
[h_rect, w] = freqz(b_rect, 1, 1024, Fs);
[h_hamm, ~] = freqz(b_hamm, 1, 1024, Fs);

plot(w, 20*log10(abs(h_rect)), 'r', 'LineWidth', 1); hold on;
plot(w, 20*log10(abs(h_hamm)), 'b', 'LineWidth', 2);

title('频域对比：幅频响应 (Frequency Response)');
xlabel('频率 (Hz)'); ylabel('幅度 (dB)');
legend('矩形窗 (下降快，但有纹波)', '汉明窗 (下降稍慢，但在阻带更干净)');
grid on; ylim([-80 10]); xlim([0 300]);

% 标注频域特征
text(80, 2, '通带纹波 \downarrow', 'Color', 'r', 'HorizontalAlignment', 'center');
text(150, -30, '\leftarrow 矩形窗旁瓣高 (切不干净)', 'Color', 'r');
text(150, -70, '\leftarrow 汉明窗衰减深 (切得干净)', 'Color', 'b');