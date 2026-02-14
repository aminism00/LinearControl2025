clear all;
clc;

%get data
load('mydata.mat');


y = tankheight;             
t = tout;     
area = 100;

if isscalar(inflow)
    u = inflow * ones(size(tankheight));  
else
    u = inflow; 
end

Ts = t(2) - t(1);   
lolim = 0;

data = iddata(y, u, Ts);

order = 1;
sys_ss = ssest(data, order);
A = sys_ss.A
B = sys_ss.B
C = sys_ss.C
D = sys_ss.D


y_sim = lsim(sys_ss, u, t);
figure;
plot(t, y, 'b', 'LineWidth', 2); hold on;
plot(t, y_sim, 'r--', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Tank Height (m)');
legend('Measured', 'State-Space Model');
title('State-Space Model Identification');
grid on;
