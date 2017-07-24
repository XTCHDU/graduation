function f=fun(b,h,g,x)

global N;
nh=3;
nb=3;
nc=3;
U1=zeros(N,nb);
for j = 1:N
    U1(j,:) = [x(N-j+1) x(N-j+1)*abs(x(N-j+1))^2 x(N-j+1)*abs(x(N-j+1))^4];
end
H = zeros(N-nh+1,N);% H矩阵 （信号长度-信道系数长度+1，信号长度）
       for j = 1: N-nh+1
           for jj = j: nh-1+j
               H(j,jj) = h(jj-j+1);
           end
       end

y=H*U1*b';

V=zeros(N-nh+1,nc);
for j=1:N-nh+1
    V(j,:)=[y(j) y(j)*abs(y(j))^2 y(j)*abs(y(j))^4];
end
f=V*g';

