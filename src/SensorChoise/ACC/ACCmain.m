%% zentei
c_pole =[ -18.8496 + 0.0000i  -9.4248 -10.8828i  -9.4248 +10.8828i]; % butterworth pf 6 pi
ST = 33*e-3; % 33ms
%%
syms TAU
Asym = [0 1 0;0 0 1;0 0 -1/TAU]
Bsym = [0;0;1/TAU]
syms Ts
Adsym=expm([Asym Bsym;0 0 0 0])

%%
% C
F1 = 0.075; F2 = 0.25;
% B
 F1 = 0.030; F2 = 0.30;
% A
F1 = 0.025; F2 = 0.41;

%%
datanum = 4;

sigmaset = [2.2634, 1.9050,3.6461,3.0518];
STset = [351.9 160.9 70 108]/1000;
tau_ = 0.2;

Ppred = zeros(6,datanum);
for i = 1:datanum
    sigma_ = sigmaset(i);
    ST_ = STset(i);
    calcLyapunov;
    Ppred(:,i) = diag(Pend);
end
%%

figure(1)
plot(1:6,Ppred(:,1),'o',1:6,Ppred(:,2),'x',1:6,Ppred(:,3),'*',1:6,Ppred(:,4),'s')

%%
figure(2)
Ppredl = log(Ppred);
plot(1:6,Ppredl(:,1),'o',1:6,Ppredl(:,2),'x',1:6,Ppredl(:,3),'*',1:6,Ppredl(:,4),'s')
xticks([1:6])
xticklabels({'p_i','v_i','a_i','p_i est','v_i est','a_i est'})
grid on
legend('SGBM','ELAS','Line-Sweep','Pillai2016','Location','Best')
xlabel('Steady-state state variable')
ylabel('logarithm of the error variance')
%%
title('Controller group C based matching method evaluation')
SaveFigPDF(2,'StereoComparison_contC')
%%
title('Controller group B based matching method evaluation')
SaveFigPDF(2,'StereoComparison_contB')
%%
title('Controller group A based matching method evaluation')
SaveFigPDF(2,'StereoComparison_contA')
