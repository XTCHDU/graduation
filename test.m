j=sqrt(-1);
%radar_transmitter_signal=A*exp(j*(2*pi*(fc*t+1/2*k*t^2)+theta0));
t=0:0.0001:1;
Ptr=1;
Lt=1;
Ls=1;
krcs=1;
r=1000;
theta=1;
Gvt(1)=1;
Gvr(1)=1;
c=3e8;
Wc=5000;
lamda=c/Wc/2/pi;
Wd=0;
radar_tr_sig=sqrt(Ptr*Lt/4/pi)*Gvt(theta)*exp(j*Wc*t);%խ���״�
radar_re_sig=radar_tr_sig*sqrt(Ls/Lt/(4*pi)^2)*Gvr(theta)/r^2*lamda*sqrt(krcs);
t2=0:0.0001:0.7;
jam_tr_sig=sqrt(Ptr*Lt/4/pi)*Gvt(theta)*exp(j.*t2.*Wc);
r2=1000;
jam_re_sig=jam_tr_sig*sqrt(Ls/Lt/(4*pi)^2)*Gvr(theta)/r2^2*lamda*sqrt(krcs);

