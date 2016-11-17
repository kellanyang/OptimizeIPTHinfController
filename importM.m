% thirtyM = -1e-6*26 + 1e-6*[...
% 45.4825
% 48.075
% 48.1725
% 47.7675
% 47.775
% 47.4575
% 47.0475
% 46.32
% 45.9525
% 45.0325
% 43.485
% 41.3925
% 40.625
% 40.685
% 40.555
% 41.2325
% 42.9425
% 44.945
% 46.1525
% 46.6225
% 46.8025
% 47.0925
% 46.7275
% 46.89
% 46.7825
% 46.7125
% 46.9075
% 47.2275
% 47.42
% 45.985
% ];

% thirtyM = ones(30,1)*19.03e-6;

%%% Produce a changing M0 sequence
perturbRange = (19.03*0.20)*1e-6;
tfin_perturb = 0.02;        % perturbation duration
period_perturb = 0.01;      % perturbation period/perturbation frequency
ti_perturb = 0.0001;         % sampling time
[M_perturb,time_perturb] = gensig('sine',period_perturb,tfin_perturb,ti_perturb);

thirtyM = 19.03*1e-6 + perturbRange*M_perturb;