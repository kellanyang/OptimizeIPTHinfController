%--------------------- Weighting Functions Selection ---------------------%
% Objective model transfer function
T = 0.001;
ksi = 0.8;
nuWm = 1;
dnWm = [T^2  2*ksi*T  1];
gainWm = 1.0;
Wm = gainWm*tf(nuWm,dnWm);

% Performance weighting function
nuWp = [1];
dnWp = [10 1];
gainWp = 480; 
Wp = gainWp*tf(nuWp,dnWp);

% Control weighting function           
nuWu = 1;
dnWu = 1;
gainWu = 0.01;
Wu = gainWu*tf(nuWu,dnWu);

%--------------------- Open-loop Weighting Connection --------------------%
systemnames = ' Gss Wm Wp Wu ';
inputvar = '[ ref; dist; control ]';
outputvar = '[ Wp; Wu; ref-Gss-dist ]';
input_to_Gss = '[ control ]';
input_to_Wm = '[ ref ]';
input_to_Wp = '[ Gss+dist-Wm ]';
input_to_Wu = '[ control ]';
sys_ic = sysic;
%--------------------------- H_infinity Design ---------------------------%
nmeas = 1;
ncon = 1;
gmin = 0;
gmax = 10;

tol = 0.001;
hin_ic = sys_ic.Nominal;
[~,~,gopt] = hinfsyn(hin_ic,nmeas,ncon,'GMIN',gmin,'GMAX',gmax,'TOLGAM',tol,'DISPLAY','off');
gmin = 1.1*gopt;
[K_hin,clp_hin,gfin] = hinfsyn(hin_ic,nmeas,ncon,'GMIN',gmin,'GMAX',gmin,'TOLGAM',tol,'DISPLAY','off');

% H-inf norm of the imaginary closed-loop system for controller design
fakeHinf = hinfnorm(clp_hin)

% Reduce the Order of H-inf Controller
order_ob = 5;
[K_hin_red,~] = reduce(K_hin,order_ob);

% Continuous State-space Description
ssK = K_hin_red;
[aK1,bK1,cK1,dK1] = ssdata(ssK);
tfK = tf(ssK);

% Discrete State-space Description
Ts = 1/fs0;
ssKD = c2d(ssK,Ts);
[akd,bkd,ckd,dkd] = ssdata(ssKD);