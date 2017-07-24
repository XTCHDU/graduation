function search_step(x,delta,k,d,g,y,last)
global newstep;
global res_best;
global a;
global flag;
if k>500 || flag
    return;
end

    newx=zeros(1,6);
    newx(1)=1;
    
for i=1:243
    
    for j=2:6
        newx(j)=x(j)+newstep(i,j-1)*delta;
        
    end
   
    newy=fun(newx(1:3),newx(4:6),g,d);
    res=calcres(newy,y);
    %[res res_best newx]
    if res<res_best
        res_best=res;
        a=newx;
        flag=true;
        
        return;
    end
   if res<last 
       search_step(newx,delta,k+1,d,g,y,res);
   end
   
    
       %  [last res]
        
    
end

    