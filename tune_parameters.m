clear;clc;
%% path
addpath('new\','models\','try\','work\');

%% model
rot = rotfe('tutorial');

%% define parameters
logic = 0;% 是否施加控制力

freqs = 49;
Ncase = 10:0.5:28;

theta_res = zeros(size(Ncase));
phaic0_res = zeros(size(Ncase));
lamda_res = zeros(size(Ncase));

%% Run Simulation
for i = 1:numel(Ncase)
    set_param('run_sim/w_set','H',strcat(num2str(freqs),'*60'));
    set_param('run_sim/N','Value',num2str(Ncase(i)));
    set_param('run_sim/if_apply','Value','1');
%     set_param('run_sim/Subsystem2/Step','After',num2str(Ncase(i)));

    disp(strcat('case: ',num2str(i),'/',num2str(numel(Ncase))));
    tic;
    Simout = sim('run_sim',20,[]);
    toc;

    A = sqrt(Simout.A.signals.values(Simout.A.time>10,1).^2+...
        Simout.A.signals.values(Simout.A.time>10,2).^2)*1000;
    N = Simout.N.signals.values(Simout.N.time>10);

    r = mean(sqrt(Simout.A.signals.values(Simout.A.time>15,1).^2+...
        Simout.A.signals.values(Simout.A.time>15,2).^2))*1000;
%     save(strcat('trainingdata_freq',num2str(freqs),'_N',num2str(Ncase(i)),'.mat'),'A','freqs','N','r');
    save(strcat('trainingdata_freq',num2str(freqs),'_N',num2str(Ncase(i)),'.mat'),'A','freqs','N');
    
    % tune parameters
    Nnum = 120000;
    ts = 1000;
    gap = 8; % 减小训练集大小

    % reference input
    e = r - A(1:gap:Nnum);
    N2 = N(1:gap:Nnum);

    beta1 = tf([1 0],[1 -1],ts);
    fai1_l = lsim(beta1,e);
    
    a1 = 0;
    b1 = 0;
    
    for k=1:numel(N2)
        a1 = a1 + fai1_l(k)*fai1_l(k);
        b1 = b1 + fai1_l(k)*N2(k);
    end
    theta1 = abs(b1/a1);
    
    phaic0 = 0.2/((10+1)*theta1);
    lamda = 10*phaic0^2;

    theta_res(i) = theta1;
    phaic0_res(i) = phaic0;
    lamda_res(i) = lamda;
end
save(strcat('tunepara_freq',num2str(freqs),'.mat'),'theta_res','phaic0_res','lamda_res');

% %% tuning
% load trainingdata_freq48_N
% 
% A = A*1000;
% 
% Nnum = 15000;
% 
% % Mz = alpha/(z - (1 - alpha))
% ts=0.001;
% alpha = 0.9;
% numm = [alpha];%[1.3 -0.689];
% denm = [1 -alpha];
% sysm = tf(numm,denm,ts);
% 
% % 参考输入
% % r_x = (A(2:Nnum+1) - (1 - alpha)*A(1:Nnum))/alpha;
% r_x = r*ones(size(A(1:Nnum)));
% e = r_x - A(1:Nnum);
% 
% beta1=tf([1 0],[1 -1],ts);
% fai1_l=lsim(beta1,e);
% 
% a1=0;
% b1=0;
% 
% for k=1:Nnum
%     a1 = a1+fai1_l(k)*fai1_l(k);
%     b1 = b1+fai1_l(k)*N(k);
% end
% theta1 = b1/a1
% 
% phaic0 = 0.2/((10+1)*theta1)
% lamda = 10*phaic0^2