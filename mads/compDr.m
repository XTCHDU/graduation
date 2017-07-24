function [myDr,myPe,err1_index,err2_index]=compDr(X1table,X2table,theta1,theta2,Nofun)
%Nofun 1 计算出错位置      2 不计算


Xlength=size(X1table,1);% 每个SNR次数

err1_index=[];
err2_index=[];
yerr_num=0;
if Nofun==1
    for kkk=1:Xlength
        theta3=X1table(kkk,:);
        if norm(theta3-theta1,2)>norm(theta3-theta2,2)
            yerr_num=yerr_num+1;
            err1_index=[err1_index,kkk];
        end
        
        theta4=X2table(kkk,:);
        if norm(theta4-theta2,2)>norm(theta4-theta1,2)
            yerr_num=yerr_num+1;
            err2_index=[err2_index,kkk];
        end
    end
elseif Nofun==2
    for kkk=1:Xlength
        theta3=X1table(kkk,:);
        if norm(theta3-theta1,2)>norm(theta3-theta2,2)
            yerr_num=yerr_num+1;
        end
        
        theta4=X2table(kkk,:);
        if norm(theta4-theta2,2)>norm(theta4-theta1,2)
            yerr_num=yerr_num+1;
        end
    end
else
    
end
myPe=yerr_num/(2*Xlength);%错误率
myDr=1-myPe;
end