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
x_kf = zeros(2,len);
x_obs(:,1) = [-1;2]; % Non-converted
x_kf(:,1) = [-1;2]; % Non-converted


%%
P_obs_ = zeros(2,2,len);
Fnum = 5;
F = -place(sysd.A,sysd.B,exp(ST*-Fnum*bwbase));
 
KFpoles=log(L)/ST;

sigma1 = 0.1
W = sigma1^2;
iteration = 50;
offset = 0;
lpoles = 1:iteration;

final_errex = zeros(iteration,4);
for lpole = 1:iteration
    Ks = place(sysd.A',sysd.A'*sysd.C',exp(ST*[KFpoles(1),-offset-1*lpole/20]))';
    Aex=[sysd.A+sysd.B*F -sysd.B*F;zeros(2) sysd.A-Ks*sysd.C*sysd.A];
    Kex = [0;0;-Ks];
    Cnomex=Kex*W*Kex';
    Pexends = dlyap(Aex,Cnomex);
    final_errex(lpole,:)=diag(Pexends)';
end

figure(2)
% plot(final_errex(:,1:3))
grid on
plot(offset+lpoles/20,log(final_errex(:,1)),'o-',offset+lpoles/20,log(final_errex(:,2)),'x-')
grid on
xlabel('pole frequency [rad]')
ylabel('log of error covariance ')
legend('pos','vel','Location','best')
title('Terminal state variance')
% SaveFigPDF(2,['statenoise_pred_F'+string(Fnum)])

figure(3)
% plot(final_errex(:,4:6))
grid on
plot(offset+lpoles/20,log(final_errex(:,3)),'o-',offset+lpoles/20,log(final_errex(:,4)),'x-')
grid on
xlabel('pole frequency [rad]')
ylabel('log of error covariance ')
legend('pos','vel','Location','best')
title('Observation noise convariance')
% SaveFigPDF(3,'obsnoise_pred_F'+string(Fnum))

%%
figure(100)
clf
plot(bwbase*-Fnum,'ro')
grid on
hold on
plot(KFpoles(:,1),[0,0],'mx')
cmap = colormap;
for lpole=1:iteration
plot([KFpoles(1),-1*lpole/20],[0,0],'*','color',cmap(lpole*2,:))
end
legend('Controller poles','Steationary Kalman filter poles','Observer poles','Location','best')
xlabel('Real')
ylabel('Imaginary')
% SaveFigPDF(100,'poleposition_F'+string(Fnum))


%% show control results
Ks = place(sysd.A',sysd.A'*sysd.C',L)';

Fnum = 5;
F = -place(sysd.A,sysd.B,exp(ST*-Fnum*bwbase));


Noise = [sigma1] .* randn(1,len);
steps = ST*(1:len);

for i = 2:len
    % update real value
    x_gt(:,i) = sysd.A * x_gt(:,i-1) + sysd.B*F*x_kf(:,i-1) + sysd.B*U(:,i);
    y = sysd.C * x_gt(:,i)+Noise(:,i);

    x_obs_hat = sysd.A * x_kf(:,i-1) + sysd.B*F*x_kf(:,i-1) + sysd.B*U(:,i);
    yobs_hat = sysd.C * x_obs_hat;
    x_kf(:,i) = x_obs_hat + Ks*(y-yobs_hat);
end


x_gt_kf = x_gt;
figure(20)
plot(steps,x_gt,steps,x_kf,'--')
xlabel('time [s]')
grid on
legend('x1_{GT}','x2_{GT}','x1_{OBS}','x2_{OBS}')
title('Steady-state KF based control')




%% OBS based control
Ks = place(sysd.A',sysd.A'*sysd.C',exp([-5*0.86+2.5i,-5*0.86-2.5i]))';
for i = 2:len
    % update real value
    x_gt(:,i) = sysd.A * x_gt(:,i-1) + sysd.B*F*x_obs(:,i-1) + sysd.B*U(:,i);
    y = sysd.C * x_gt(:,i)+Noise(:,i);

    x_obs_hat = sysd.A * x_obs(:,i-1) + sysd.B*F*x_obs(:,i-1) + sysd.B*U(:,i);
    yobs_hat = sysd.C * x_obs_hat;
    x_obs(:,i) = x_obs_hat + Ks*(y-yobs_hat);
end
figure(21)
plot(steps,x_gt,steps,x_obs,'--')
xlabel('time [s]')
grid on
legend('x1_{GT}','x2_{GT}','x1_{OBS}','x2_{OBS}')
title('OBS based control')

%% KF vs OBS

figure(22)
plot(steps,x_gt_kf,steps,x_gt,'--')
xlabel('time [s]')
grid on
legend('x1_{KF}','x2_{KF}','x1_{OBS}','x2_{OBS}')

Aex=[sysd.A+sysd.B*F -sysd.B*F;zeros(2) sysd.A-Ks*sysd.C*sysd.A];
Kex = [0;0;-Ks];
Cnomex=Kex*W*Kex';
Pexends = dlyap(Aex,Cnomex);
diag(Pexends)' % 0.0002    0.0414    0.0100   16.6391