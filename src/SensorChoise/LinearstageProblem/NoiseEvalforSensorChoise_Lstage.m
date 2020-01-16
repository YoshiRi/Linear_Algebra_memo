%%
clear all

%%
M =  0.0936; D= 1.1226; K= 1.2783;

%%sys = ss(tf([1],[M,D,K]));

%%
A = [ 0, 1 ;-K/M, -D/M];
B = [0;1];
C = [1  0];
ST = 0.030;
sys = ss(A,B,C,[]);
sysd = c2d(sys,ST)

% sim setting
len = 200;
time = 0:ST:ST*(len-1);
x_gt = zeros(2,len);
x_obs = zeros(2,len); 
x_obs2 = zeros(2,len);

% noise variable 
sigma1 = 0.01;
Noise = [sigma1] .* randn(1,len);

% init
x_gt(:,1) = [1;-2];
x_obs(:,1) = [-1;2]; % converted 
x_obs2(:,1) = [-1;2]; % Non-converted
U = sin(1*pi/3*time);

bwbase = 1/sqrt(2)*[1+i,1-i];


final_err = zeros(20,2);

W = sigma1^2;
for lpole = 1:20
    Ks = place(sysd.A',sysd.A'*sysd.C',exp(ST*bwbase*-1*lpole))';
    Cnom=Ks*W*Ks';
    Pends = dlyap(sysd.A-Ks*sysd.C*sysd.A,Cnom);
    final_err(lpole,:)=diag(Pends)';
end

lpoles = 1:20;
figure(1)
plot(lpoles,log(final_err(:,1)),'o-',lpoles,log(final_err(:,2)),'x-')
grid on
xlabel('pole frequency [rad]')
ylabel('log of error covariance ')
legend('pos','vel','Best')


%%
P_obs_ = zeros(2,2,len);
Fnum = 5;
F = -place(sysd.A,sysd.B,exp(ST*-Fnum*bwbase));
 

sigma1 = 0.1
W = sigma1^2;

lpoles = 1:20;

final_errex = zeros(20,4);
for lpole = 1:20
    Ks = place(sysd.A',sysd.A'*sysd.C',exp(ST*bwbase*-1*lpole))';
    Aex=[sysd.A+sysd.B*F -sysd.B*F;zeros(2) sysd.A-Ks*sysd.C*sysd.A];
    Kex = [0;0;-Ks];
    Cnomex=Kex*W*Kex';
    Pexends = dlyap(Aex,Cnomex);
    final_errex(lpole,:)=diag(Pexends)';
end

figure(2)
% plot(final_errex(:,1:3))
grid on
plot(lpoles,log(final_errex(:,1)),'o-',lpoles,log(final_errex(:,2)),'x-')
grid on
xlabel('pole frequency [rad]')
ylabel('log of error covariance ')
legend('pos','vel','acc','best')
title('Terminal state variance')
SaveFigPDF(2,strcat('statenoise_pred_F',num2str(Fnum)))

figure(3)
% plot(final_errex(:,4:6))
grid on
plot(lpoles,log(final_errex(:,3)),'o-',lpoles,log(final_errex(:,4)),'x-')
grid on
xlabel('pole frequency [rad]')
ylabel('log of error covariance ')
legend('pos','vel','acc','best')
title('Observation noise convariance')
SaveFigPDF(3,strcat('obsnoise_pred_F',num2str(Fnum)))

[Min,stateindx]=min(final_errex(:,1)+final_errex(:,2))
[Min,obsindx]=min(final_errex(:,3)+final_errex(:,4))

figure(10)
grid on
plot(lpoles,log(final_errex(:,1)+final_errex(:,2)),'s-',lpoles,log(final_errex(:,3)+final_errex(:,4)),'*-')
grid on
xlabel('pole frequency [rad]')
ylabel('log of trace of error covariance ')
legend('terminal state','observation err','Location','best')
title('Trace of control and observation noise convariance')
SaveFigPDF(10,strcat('trace_pred_F',num2str(Fnum)))

%%
figure(100)
clf
plot(bwbase*-Fnum,'ro')
grid on
hold on

cmap = colormap;
for lpole=1:20
plot(bwbase*-1*lpole,'*','color',cmap(lpole*2,:))
end
legend('Controller poles','Observer poles')
xlabel('Real')
ylabel('Imaginary')
SaveFigPDF(100,'poleposition_F',num2str(Fnum)))

%% KF vs Poles
KFPs = []
KFpoles = []
[P,L,G] = dare(sysd.A',sysd.C', sysd.B*sysd.B'*1e-4 ,sigma1^2) %Pend, Eig, KFgain 
KFPs=[KFPs, diag(P)];KFpoles=[KFpoles, log(L)/ST];
% KFL=P'*sysd.C'/(sysd.C*P'*sysd.C'+sigma1^2)
[P,L,G] = dare(sysd.A',sysd.C', sysd.B*sysd.B'*1e-5 ,sigma1^2) %Pend, Eig, KFgain 
KFPs=[KFPs, diag(P)];KFpoles=[KFpoles, log(L)/ST];
[P,L,G] = dare(sysd.A',sysd.C', sysd.B*sysd.B'*1e-6 ,sigma1^2) %Pend, Eig, KFgain 
KFPs=[KFPs, diag(P)];KFpoles=[KFpoles, log(L)/ST];
[P,L,G] = dare(sysd.A',sysd.C', sysd.B*sysd.B'*1e-7 ,sigma1^2) %Pend, Eig, KFgain 
KFPs=[KFPs, diag(P)];KFpoles=[KFpoles, log(L)/ST];

KFpoles
%save('sigma1e-1.mat','Final_err_real')

figure(101)
clf
plot(bwbase*-Fnum,'ro')
grid on
hold on
plot(KFpoles(:,1),[0,0],'mx')
cmap = colormap;
for lpole=1:20
plot(bwbase*-1*lpole,'*','color',cmap(lpole*2,:))
end

legend('Controller poles','Steationary Kalman filter poles','Observer poles','Location','best')
xlabel('Real')
ylabel('Imaginary')
SaveFigPDF(101,strcat('polepositionwithKF_F',num2str(Fnum)))




%% simulation loop


Final_err_real=zeros(20,4);


for lpole = 1:20
simloop = 400;
Ks = place(sysd.A',sysd.A'*sysd.C',exp(ST*bwbase*-1*lpole))';

x_obs_loop = zeros(2,simloop);
e_obs_loop = zeros(2,simloop);

for j = 1:simloop
Noise = [sigma1] .* randn(1,len);

for i = 2:len
    % update real value
    x_gt(:,i) = sysd.A * x_gt(:,i-1) + sysd.B*F*x_obs(:,i-1) + sysd.B*U(:,i);
    y = sysd.C * x_gt(:,i)+Noise(:,i);

    x_obs_hat = sysd.A * x_obs(:,i-1) + sysd.B*F*x_obs(:,i-1) + sysd.B*U(:,i);
    yobs_hat = sysd.C * x_obs_hat;
    x_obs(:,i) = x_obs_hat + Ks*(y-yobs_hat);
end
x_obs_loop(:,j) = x_gt(:,end); 
e_obs_loop(:,j) = x_gt(:,end)-x_obs(:,end); 
end

% Pinf
Pinf = e_obs_loop*e_obs_loop'/(simloop-1)

% Xe noise
xe =x_obs_loop-mean(x_obs_loop,2);
Pxinf = xe*xe'/(simloop-1)

Final_err_real(lpole,:) = [diag(Pxinf);diag(Pinf)];
end

Final_err_real

% kalmanfilter
simloop = 400;
Ks = place(sysd.A',sysd.A'*sysd.C',L)';

x_obs_loop = zeros(2,simloop);
e_obs_loop = zeros(2,simloop);

for j = 1:simloop
Noise = [sigma1] .* randn(1,len);

for i = 2:len
    % update real value
    x_gt(:,i) = sysd.A * x_gt(:,i-1) + sysd.B*F*x_obs(:,i-1) + sysd.B*U(:,i);
    y = sysd.C * x_gt(:,i)+Noise(:,i);

    x_obs_hat = sysd.A * x_obs(:,i-1) + sysd.B*F*x_obs(:,i-1) + sysd.B*U(:,i);
    yobs_hat = sysd.C * x_obs_hat;
    x_obs(:,i) = x_obs_hat + Ks*(y-yobs_hat);
end
x_obs_loop(:,j) = x_gt(:,end); 
e_obs_loop(:,j) = x_gt(:,end)-x_obs(:,end); 
end

% Pinf
Pinf = e_obs_loop*e_obs_loop'/(simloop-1)

% Xe noise
xe =x_obs_loop-mean(x_obs_loop,2);
Pxinf = xe*xe'/(simloop-1)

KF_err_real = [diag(Pxinf);diag(Pinf)];


%% Showing

onespole = ones(size(lpoles));
figure(4)
% plot(final_errex(:,1:3))
clf
grid on
plot(lpoles,log(Final_err_real(:,1)),'o-',lpoles,log(Final_err_real(:,2)),'x-',lpoles,onespole*log(KF_err_real(1)),'--',lpoles,onespole*log(KF_err_real(2)),'-.')
grid on
xlabel('pole frequency [rad]')
ylabel('log of error covariance ')
legend('pos','vel','KF pos','KF vel')
title('Remaining noise after control')

figure(5)
clf
% plot(final_errex(:,4:6))
grid on
plot(lpoles,log(Final_err_real(:,3)),'o-',lpoles,log(Final_err_real(:,4)),'x-',KF_err_real(3),KF_err_real(4),'s',lpoles,onespole*log(KF_err_real(3)),'--',lpoles,onespole*log(KF_err_real(4)),'-.')
grid on
xlabel('pole frequency [rad]')
ylabel('log of error covariance ')
legend('pos','vel','KF pos','KF vel')
title('Observation noise')



%% Compared with KF?

[P,L,G]=dare(sysd.A',sysd.C',diag([1e-4,1e-4]),sigma1^2)

%P =
%    1.0e-03 *
% 
%     0.1404   -0.1349
%    -0.1349    0.1708
% 
% L =
% 
%     0.9487
%     0.7254
% 
% G =
% 
%     0.0134   -0.0140
KFL=P'*sysd.C'/(sysd.C*P'*sysd.C'+sigma1^2)
log(eig(sysd.A-KFL*sysd.C*sysd.A))/ST

%    -4.0829
%   -10.5329


%%
figure(6)
clf
% plot(final_errex(:,4:6))
plot(lpoles,log(final_errex(:,1)),'-',lpoles,log(final_errex(:,2)),'--')
hold on
plot(lpoles,log(Final_err_real(:,1)),'o',lpoles,log(Final_err_real(:,2)),'x')
grid on
xlabel('pole frequency [rad]')
ylabel('log of error covariance ')
legend('pos_{pred}','vel_{pred}','pos_{sim}','vel_{sim}','Location','eastoutside')
title('Terminal state covariance with different poles')
SaveFigPDF(6,strcat('statenoise_sim',num2str(Fnum)))

figure(7)
clf
% plot(final_errex(:,4:6))
plot(lpoles,log(final_errex(:,3)),'-',lpoles,log(final_errex(:,4)),'--')
hold on
plot(lpoles,log(Final_err_real(:,3)),'o',lpoles,log(Final_err_real(:,4)),'x')
grid on
xlabel('pole frequency [rad]')
ylabel('log of error covariance ')
legend('pos_{pred}','vel_{pred}','pos_{sim}','vel_{sim}','Location','eastoutside')
title('Observation noise with different poles')
SaveFigPDF(7,strcat('obsnoise_sim',num2str(Fnum)))


figure(11)
clf
hold on
% plot(lpoles,log(final_errex(:,1)+final_errex(:,2)),'s-',lpoles,log(final_errex(:,3)+final_errex(:,4)),'*-')
plot(lpoles,log(Final_err_real(:,1)+Final_err_real(:,2)),'s-',lpoles,log(Final_err_real(:,3)+Final_err_real(:,4)),'*-')
grid on
xlabel('pole frequency [rad]')
ylabel('log of trace of error covariance ')
legend('terminal state','observation err','Location','best')
title('Trace of control and observation noise convariance')
SaveFigPDF(11,strcat('trace_sim_F',num2str(Fnum)))

figure(12)
clf
hold on
plot(lpoles,log(final_errex(:,1)+final_errex(:,2)),'-',lpoles,log(final_errex(:,3)+final_errex(:,4)),'-.')
plot(lpoles,log(Final_err_real(:,1)+Final_err_real(:,2)),'s',lpoles,log(Final_err_real(:,3)+Final_err_real(:,4)),'*')
grid on
xlabel('pole frequency [rad]')
ylabel('log of trace of error covariance ')
legend('fb_{pred}','obs_{pred}','fb_{sim}','obs_{sim}','Location','best')
title('Trace of control and observation noise convariance')
SaveFigPDF(12,strcat('trace_simcomp_F',num2str(Fnum)))

%%
figure(10)
plot(time,x_gt)
grid on
xlabel('time [s]')
ylabel('state value')
legend('pos','vel','acc')

figure(10)
plot(time,x_gt)
grid on
xlabel('time [s]')
ylabel('state value')
legend('pos','vel','acc')

figure(100)
clf
plot(bwbase*-0.5,'rx')
grid on
hold on

cmap = colormap;
for lpole=1:20
plot(bwbase*-1*lpoles,'*','color',cmap(lpoles*2,:))
end
legend('Controller poles','Observer poles')
xlabel('Real')
ylabel('Imaginary')
SaveFigPDF(100,'polepositions')

[minval,minindex]=min(Final_err_real(:,1))



