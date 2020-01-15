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