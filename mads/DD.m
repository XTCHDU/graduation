clc;
clear all;
close all;
global now;
now=1;
makenewpoint(0);
global h;
global b;
global c;
global d;
global SNR;
global ress

h1 = [0.996 0.0628 0.079];
b1 = [ 1 -0.735 -0.0986 ];
c1 = [1 -0.125 -0.753 ];
b2=[1 -0.613 -0.088];
h2=[0.9 0.053 0.07]; 
c2=[1 -0.1 -0.6];
h=h1;
b=b1;
c=c1;
Xtable1=zeros(10,9);
Xtable2=zeros(10,9);
time=1;
anss=zeros(1,100);
pe=zeros(1,100);
err1=zeros(1,100);
err2=zeros(1,100);
ftable1=zeros(30,9,50);
ftable2=zeros(30,9,50);
SNR=1000;
    Xtable1=zeros(30,9);
Xtable2=zeros(30,9);
for i=1:1

d=rand(1,20);
h=h1;
b=b1;
c=c1;

Xtable1(i,:)=main_iterative;

h=h2;
b=b2;
c=c2;
%Xtable2(i,:)=main_iterative;
end
[anss(time),pe(time)]=compDr(Xtable1,Xtable2,[b1 h1 c1],[b2 h2 c2],2);
ftable1(:,:,time)=Xtable1;
ftable2(:,:,time)=Xtable2;
time=time+1;


