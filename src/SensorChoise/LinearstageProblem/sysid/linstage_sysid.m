clear

%% load chirp2
time = chirp2.X.Data;
ST = time(2)-time(1)

%actout = input
in = chirp2.Y(2).Data;
out = chirp2.Y(3).Data;

%%
winsize = []
freq = 1/ST
[PXX,FREQ] = tfestimate(in,out,winsize,[],[],freq);
Txx = frd(PXX,FREQ*2*pi,1/freq);

%%
figure(2)
bode(Txx)
figure(3)
[COHER,cFREQ] = mscohere(in,out,winsize,[],[],freq);
semilogx(cFREQ*2*pi,COHER);

%%
weight = zeros(size(COHER));
weight(COHER>0.95) = 1;
fmax = 100;
fmin = 1;
weight(cFREQ >= fmax)= 0;
weight(cFREQ <= fmin)= 0;

[num den] = invfreqs(PXX,FREQ*2*pi,0,2,weight,10);

%%

% bode関数のFigureプロパティを取得 
p = bodeoptions;   
% 位相オフセットの調整をOnに設定し、保持する値を0°に設定
p.PhaseMatching = 'on'; 
p.PhaseMatchingValue = 0;   

fitsys = tf(num,den);

figure(4);
bb=bodeplot(Txx,p,fitsys,p);
legend('ground truth','Estimated')
xlim([2 80])
xlabel('Frequency')
ylabel('Gain')
title('Bode plot')

%% rough answer
