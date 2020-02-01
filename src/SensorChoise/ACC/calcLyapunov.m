%input: tau_ ST_ sigma_

%%
Ad_calc = double(subs(Adsym,{TAU,Ts},{tau_,ST_}));
Adp = Ad_calc(1:3,1:3);
Bdp = Ad_calc(1:3,4);
Cdp = [1 0 0];
Fdp = [-F1,-F2,0];
Kdp = place(Adp',Adp'*Cdp',exp(c_pole*ST_))';

%%
Aex = [Adp+Bdp*Fdp Bdp*Fdp;
       zeros(size(Bdp*Fdp)) Adp - Kdp*Cdp*Adp];
Kex = [0;0;0;-Kdp];
Pend = dlyap(Aex,Kex*sigma_*Kex');
