%% Make system
Ad_calc = double(subs(Adsym,{TAU,Ts},{tau_,ST_}));
Adp = Ad_calc(1:3,1:3);
Bdp = Ad_calc(1:3,4);
Cdp = [1 0 0];
Fdp = [-F1,-F2,0];

%% calc K from KF of riccati
[P_kf,Pole_kf,Kdpkf] = dare(Adp',Cdp',diag([1,1,1])*1.5e-1,sigma_);
Pole_kf

est = diag(dlyap(Adp-Kdpkf'*Cdp*Adp,Kdpkf'*sigma_*Kdpkf))
pplot(log(Pole_kf)/ST_,'o')
%% get extended system and calc end

Aex = [Adp+Bdp*Fdp Bdp*Fdp;
       zeros(size(Bdp*Fdp)) Adp - Kdp*Cdp*Adp];
Kex = [0;0;0;-Kdp];
Pend = dlyap(Aex,Kex*sigma_*Kex');