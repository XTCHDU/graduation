function f=calcres(x,y)
f=0;
global N;
for i=1:N-2;
   f=f+(x(i)-y(i))^2;
end
