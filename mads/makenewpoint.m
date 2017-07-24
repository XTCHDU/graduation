function makenewpoint(k)
data=[-1,0,1];
global now;
global newstep;

if k<5
    for i=1:3
        for j=now:243
        newstep(j,k+1)=data(i);
        end
        makenewpoint(k+1);
    end
else
    now=now+1;    
end