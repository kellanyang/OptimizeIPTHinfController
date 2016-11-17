%{
main.m
This file contains the main function that plots the transient performance 
of a closed-loop IPT system. The system output voltage is controlled by an 
H-infinity controller designed with the help of performance weighting 
function Wp and control weighting function Wu. This file calls three 
external m files, setParametes.m, importM.m and designController.m, to help 
evaluate system performance while the mutual inductance, M, is changing.
%}
% clear all;clf;clc;
%% External Process
% WpGain = 480;
setParameters
designController

%% Closed-loop System Connection
systemnames = 'Gss K_hin';
inputvar = '[ ref; dist ]';
outputvar = '[ Gss+dist; K_hin ]';
input_to_Gss = '[ K_hin ]';
input_to_K_hin = '[ ref-Gss-dist ]';
clp_perf = sysic;

%% Evaluate CL System Stability
BIBOStability = hinfnorm(clp_perf);   %supposed to be less than 1.

%% Grid uncertain parameters uniformly
nsample = 1;
cls_perf_uniform = gridureal(clp_perf, nsample);

%% -- Plot Reference Tracking Performance --
r1 = 50;
r2 = 40;
r3 = 30;

ti = 1e-6;
tfin = 0.5;
time_r = 0:ti:tfin;
nstep = size(time_r, 2);

ref(1:floor(nstep/4)) = r1;
ref(floor(nstep/4):floor(nstep/2)) = r2;%r2
ref(floor(nstep/2):3*floor(nstep/4)) = r3;%r3
ref(3*floor(nstep/4):nstep) = r1;%r1

dist(1:nstep) = 0.0;

% figure(1)
for i = 1:nsample
    [y_hinf_r, ~, sysStates] = lsim(cls_perf_uniform(1:2, 1:2, i), [ref', dist'], time_r);
%     plot(time_r, ref, 'r--', time_r, y_hinf_r(:,1), 'b-');
%     hold on
end

%% -- Plot Transient Response when M changes --
initStates = sysStates(nstep, :);
clear ref dist r1 r2 r3 tfin time_r nstep
r1 = 50;
ti = 1e-6;
tfin = 0.2;
time_r = 0:ti:tfin;
nstep = size(time_r, 2);
ref(1:nstep) = r1;
dist(1:nstep) = 0.0;

importM                                       %Import 30 mutual inductances

numM = size(thirtyM, 1);
thirtyCls = usubs(clp_perf, 'M', thirtyM);

startT = 1;
simTime = floor(nstep/numM);
endT = startT + simTime;

for indexM = 1:numM
    [yTemp,tTemp,xTemp] = lsim(thirtyCls(:, :, indexM, 1), [ref(startT:endT)', dist(startT:endT)'], time_r(startT:endT), initStates);
   
    figure(2)
    plot(time_r(startT:endT), yTemp(:,1), 'b-')
    hold on
    
    % Refreshing
    [row, ~] =size(xTemp);
    initStates=xTemp(row, :);
    startT = startT + simTime;
    endT = endT + simTime;
end 

figure(2)
grid
title('Closed-loop transient response of dist')
xlabel('Time (secs)')
ylabel('y(t)')
hold on