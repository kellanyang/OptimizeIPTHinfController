
%% Uncertain Parameters
M0 = 19.03e-6;
Rdc0 = 2.5;
w0 = 20000*2*pi;

M = ureal('M',M0,'Percentage',0.0000001);          %H   Mutual Inductance
Rdc = ureal('Rdc',Rdc0,'Percentage',0.0000001);    %Om  DC Load Resistance
w = ureal('w',w0,'Percentage',0.0000001);          %rad Operating Frequency

%% Certain Parameters
Rp = 0.04293;                                %Om  Primary Coil Resistance
Lp = 83.52e-06;                              %H   Primary Coil Inductance
Cp = 1/w0^2/Lp;                              %C   Primary Capacitance

Rs = 0.02513;                                %Om  Secondary Coil Resistance
Ls = 43.53e-06;                              %H   Secondary Coil Inductance
Cs = 1/w0^2/Ls;                              %C   Secondary Capacitance

Cf = 940e-6;                                 %C   Filter Capacitance
fs0 = w0/2/pi;                               %Hz  Operating Frequency

%% GSSA Model of Series-series Compensated IPT system
det=M^2-Lp*Ls;                  %Determinant for the simplicity of matrix A
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

Gnom = Gss.NominalValue;