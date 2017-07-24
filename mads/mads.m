function [f,ress]=mads(nowb,nowh,g,x,y,ress)

nowy=fun(nowb,nowh,g,x);
global res_best;
%res_best=calcres(nowy,y);
res_best=ress;
delta_k=0.1;
delta_p=sqrt(delta_k)*7;
global a;
a=[nowb nowh];
global flag;
time=0;
while(delta_k>=0.00001)
    res_best
   % delta_k
    a
    g
    %time=time+1
   % if (time>100) break;
    %end
flag=false;
search_step(a,delta_k,1,x,g,y,res_best);
if flag==false
    delta_p=sqrt(delta_k)*6;
    poll_step(a,delta_p,x,g,y);
end
if flag==true
   

    delta_k=4*delta_k;
else
       if delta_k<=0.0001 delta_k=delta_k/10;
       else
        delta_k=0.0001*delta_k;
       end
end
end
f=a;
ress=res_best;

        

