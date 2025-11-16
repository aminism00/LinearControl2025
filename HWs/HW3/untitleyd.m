Kt = 0;    
Kb = 0.5;
f  = 0.2;
J  = 1;
Ra = 2;
Km = 0.8;
Ka = 0.05;
aalpha = Km / Ra;


modelname = 'untitled';


open_system(modelname);


[A,B,C,D] = linmod(modelname);   
sys_lin = ss(A,B,C,D);           
G_tf_all = tf(sys_lin);         

G11 = G_tf_all(1,1);             

[num, den] = tfdata(G11, 'v'); 


s = sym('s');                    
G_sym = poly2sym(num, s) / poly2sym(den, s);
G_sym = simplify(G_sym);         


disp('تابع تبدیل (نمادین):');
pretty(G_sym)    

step(G11);
grid on;
title('Step Response for Ka = 0.05 and Kt = 0');
xlabel('Time (s)');
ylabel('\Theta(t)');