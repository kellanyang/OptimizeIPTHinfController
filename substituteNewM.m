%%% substituteNewM.m >>> Redefine nominal parameters. This m file functions
%%% as a part of file FiveOrderLIMIN_changingM_v1.m to help change nominal
%%% parameters in realtime.
%% Uncertain Parameters
M=ureal('M',M0,'Percentage',20);                 %H   Mutual Inductance
Rdc=ureal('Rdc',Rdc0,'Percentage',20);           %Om  DC Load Resistance
w=ureal('w',w0,'Percentage',0.0000001);                %rad Operating Angular Frequency

Cp = 1/w0^2/Lp;
Cs = 1/w0^2/Ls;

det=M^2-Lp*Ls;                                  %    Determinant
%% System
A = [   Rp*Ls/det w Ls/det 0 M*Rs/det 0 M/det 0 2*M/pi/det ;
        -w Rp*Ls/det 0 Ls/det 0 M*Rs/det 0 M/det 0 ;
        1/Cp 0 0 w 0 0 0 0 0 ;
        0 1/Cp -w 0 0 0 0 0 0 ; 
        M*Rp/det 0 M/det 0 Rs*Lp/det w Lp/det 0 2*Lp/pi/det ;
        0 M*Rp/det 0 M/det -w Rs*Lp/det 0 Lp/det 0 ; 
        0 0 0 0 1/Cs 0 0 w 0 ;
        0 0 0 0 0 1/Cs -w 0 0 ;
        0 0 0 0 4/pi/Cf 0 0 0 -1/Cf/Rdc ];
B = [0 2*Ls/pi/det 0 0 0 2*M/pi/det 0 0 0]';
C = [0 0 0 0 0 0 0 0 1];
D = 0;
Gss = ss(A, B, C, D);

systemnames = 'Gss K_hin';
inputvar = '[ ref; dist ]';
outputvar = '[ Gss+dist; K_hin ]';
input_to_Gss = '[ K_hin ]';
input_to_K_hin = '[ ref-Gss-dist ]';
clp_per = sysic;

nsample = 5;
cls_per_uniform = gridureal(clp_per,nsample); %等间距平均采样5次