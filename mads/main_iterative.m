function [anss]=main_iterative
warning off;
Loop = 10000;%最大迭代次数
global h;
global b;
global c;
global d;
 nh = length(h);  %信道响应阶数

 nb = length(b); %非线性多项式阶数=2*nb+1,不考虑偶次项

thresh = 1e-20;    %迭代门限 
nd=length(d); 
 global N;
N=nd;
y=fun(b,h,c,d); 
global SNR;
h_est_0=[1 -0.1 -0.1];
h_est = h_est_0;%初始估计值
temp_b = [1 -0.1  -0.1];
temp_h = h_est;
h1 = [0.996 0.0628 0.079];
b1 = [ 1 -0.735 -0.0986 ];
c1 = [1 -0.125 -0.77 ];
%temp_b=b1;
%temp_h=h1;
%y = awgn(y,SNR);%加噪
temp_c=[1 -0.1 -0.1];
global fdelta;
fdelta=0.1;
global ress;
for kk = 1:Loop
     
     sum=0;
     utemp=zeros(nd,1);
     
for n=nh:nd
    for k=0:nh-1
      for j=1:nb
          sum=sum+temp_b(j)*d(n-k)*abs(d(n-k))^(2*j-2);
      end
      sum=sum*temp_h(k+1);
      utemp(n)=utemp(n)+sum;
      sum=0; 
    end
end
utemp=utemp(nh:N);
utemp=flipud(utemp);

f=@(a,x) x+a(1)*x.*abs(x).^2+a(2)*x.*abs(x).^4;
[g_est, res1]=lsqcurvefit(f,temp_c(2:3),utemp',y');

 c_est=[1 g_est];
%c_est=c;

[P,res2]=mads(temp_b,temp_h,c_est,d,y,res1);

b_est=P(1:3);
h_est=P(4:6);
ress=res1-res2
res1;
res2;
if  ress<thresh
    break;
end
  
       temp_b = b_est;
       temp_h = h_est;
       temp_c=c_est;
      
end
disp(b_est)
disp(h_est)
disp(c_est)
anss=[b_est h_est c_est];

