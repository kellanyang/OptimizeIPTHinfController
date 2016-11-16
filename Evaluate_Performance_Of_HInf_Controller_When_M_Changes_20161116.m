Configure_Parameters_Of_SS_IPT_System

%%% H Infinity Controller Design
%--------------------- Weighting Functions Selection ---------------------%
% Objective model transfer function
T = 0.001;
ksi = 0.8;
nuWm = 1;
dnWm = [T^2  2*ksi*T  1];
gainWm = 1.0;
Wm = gainWm*tf(nuWm,dnWm);

%--------------------------
% Performance weighting function
nuWp = [1];
dnWp = [10 1];
gainWp = 1000; 
Wp = gainWp*tf(nuWp,dnWp);

%--------------------------
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
% opt = robopt('DISPLAY','off');

tol = 0.001;
hin_ic = sys_ic.Nominal;
[~,~,gopt] = hinfsyn(hin_ic,nmeas,ncon,'GMIN',gmin,'GMAX',gmax,'TOLGAM',tol,'DISPLAY','off');
gmin = 1.1*gopt;
[K_hin,clp_hin,gfin] = hinfsyn(hin_ic,nmeas,ncon,'GMIN',gmin,'GMAX',gmin,'TOLGAM',tol,'DISPLAY','off');

order_ob = 5;
[K_hin_red,~] = reduce(K_hin,order_ob);


% Reduced-Order Continues h-inf controller
ssK = K_hin_red;
[aK1,bK1,cK1,dK1] = ssdata(ssK);
tfK = tf(ssK);

% Discretize State-Space
Ts = 1/fs0;
ssKD = c2d(ssK,Ts);
[akd,bkd,ckd,dkd] = ssdata(ssKD);

hinfout = hinfnorm(clp_hin)

%% --------------connection for performance analysis------------- %%
systemnames = 'Gss K_hin';
inputvar = '[ ref; dist ]';
outputvar = '[ Gss+dist; K_hin ]';
input_to_Gss = '[ K_hin ]';
input_to_K_hin = '[ ref-Gss-dist ]';
clp_per = sysic;

hinfout22 = hinfnorm(clp_per)

opt = robuststabOptions('Sensitivity','off','Display','on');
% [stabmarg,destabunc,report,info]=robuststab(clp_per,opt)

nsample = 5;
cls_per_uniform = gridureal(clp_per,nsample); %等间距平均采样20次


%% -------------- Reference Tracking Performance -------------- %%
% Reference definition
r1 = 50;
r2 = 40;
r3 = 30;
ti = 1e-6;
tfin = 0.16;
time_r = 0:ti:tfin;
nstep = size(time_r,2);
ref(1:floor(nstep/4)) = r1;
ref(floor(nstep/4):floor(nstep/2)) = r1;%r2
ref(floor(nstep/2):3*floor(nstep/4)) = r1;%r3
ref(3*floor(nstep/4):nstep) = r1;%r1
dist(1:nstep) = 0.0;

figure(1)
for i = 1:nsample
    [y_hinf_r,~,sysStates] = lsim(cls_per_uniform(1:2,1:2,i),[ref',dist'],time_r);
    plot(time_r,ref,'r--',time_r,y_hinf_r(:,1),'b-')
    hold on
end

%% ------------------ Disturbance Resistance ------------------ %%

% Disturbance definition
% % tfin = 0.16;        % final time
% % ti = 1e-6;          % sampling time
% % [dist1,time_d] = gensig('square',tfin/2,tfin,ti);
% % dist = 1 - 2*dist1;
% % nstep = size(time_d,1);
% % ref(1:nstep) = 0.0;
% % 
% % figure(2)
% % for i = 1:nsample
% %     [y_hinf_d,~] = lsim(cls_per_uniform(1:2,1:2,i),[ref',dist],time_d);
% %     plot(time_d,dist,'r--',time_d,y_hinf_d(:,1),'b-')
% %     hold on
% % end

%% ------------- Transient Response when M changes ------------- %%
clear ref; clear dist 
% Reference definition
r1 = 50;
ti = 1e-6;
tfin = 0.16;
time_r = 0:ti:tfin;
nstep = size(time_r,2);
ref(1:nstep) = r1;
dist(1:nstep) = 0.0;

% [y,t,x] = lsim( cls_per.Nominal,[ref(1:3000)',dist(1:3000)],time(1:3000));
% figure(8)
% plot(t,y,'b-')
% hold on



Import_Table_Of_Mutual_Inductances %import 30 mutual inductances.
numberOfM = size(thirtyValuesOfM,1);
arrayOfThirtyClosedLoopSystems = usubs(clp_per,'M',thirtyValuesOfM);

initialStates = sysStates(nstep,:);
startingTime = 1;
SimulationPeriod = floor(nstep/numberOfM);
endingTime = startingTime + SimulationPeriod;

for discreteTimeInAPeriod = 1:numberOfM
    [yy,tt,xx] = lsim(arrayOfThirtyClosedLoopSystems(:,:,discreteTimeInAPeriod,1),[ref(startingTime:endingTime)',dist(startingTime:endingTime)'],time_r(startingTime:endingTime),initialStates);
   
    figure(8)
    plot(time_r(startingTime:endingTime),yy(:,1),'b-')
    hold on
    [r n] =size(xx);
    initialStates=xx(r,:);
    startingTime = startingTime + SimulationPeriod;
    endingTime = endingTime + SimulationPeriod;
end 

figure(8)
grid
title('Closed-loop transient response of dist')
xlabel('Time (secs)')
ylabel('y(t)')
hold on