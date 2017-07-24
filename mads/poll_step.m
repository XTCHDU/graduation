function poll_step(x,delta,d,g,y)
global newstep;
global res_best;
global a;
global flag;
newx=zeros(1,6);
newx(1)=1;
for i=1:243
   
    for j=2:6
        newx(j)=x(j)+newstep(i,j-1)*delta;
       
    end
   
    newy=fun(newx(1:3),newx(4:6),g,d);
    res=calcres(newy,y);
   k1= [res newx];
   k2= [res_best x];
     if res<res_best 
        res_best=res;
        a=newx;
        flag=true;
        
     end
end
    
        