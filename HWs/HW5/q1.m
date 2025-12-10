%% Nyquist plot (Guaranteed working version)
clear; clc; close all;

K = 1;   % Change gain here

% Transfer function: L(s) = 100K / (s^2 (1 + 0.1s))
num = 100*K;
den = [0.1 1 0 0];   % 0.1*s^3 + 1*s^2 + 0*s + 0

L = tf(num, den);

% Frequency range (very important!)
w = logspace(-3, 4, 5000);

% Plot
figure('Position',[300 150 900 650],'Color','w');
nyquist(L, w);
grid on; hold on;

% Mark -1 point
plot(-1,0,'ro','MarkerSize',10,'LineWidth',2);

title('Nyquist Plot of L(s) = 100K / (s^2(1+0.1s))','FontSize',14,'FontWeight','bold');
xlabel('Real Axis'); ylabel('Imag Axis');

axis equal;
