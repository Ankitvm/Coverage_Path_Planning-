clc
close all
clear all
w = 0.5;                    
radr = 0.125;                  
rad = 0.04*rand(10,1);
xrnd = 8*rand(10,1)+1;
yrnd = 8*rand(10,1)+1;
figure(1)
for i = 1:10
rectangle('Position',[xrnd(i)-sqrt(rad(i)),yrnd(i)-sqrt(rad(i)),2*sqrt(rad(i)),2*sqrt(rad(i))],'Curvature', [1 1]);
end
hold on
x = 0:0.5:10;
for i = 1:10
    if x(i) ~= (xrnd(i)+2.5*radr)||(xrnd(i)-2.5*radr)
     %ylim([0 10]);
     y = 0:0.5:10;
     plot([x; x], [zeros(1, length(x))*min(y); ones(1, length(x))*max(y)], 'Color', 'k')
     %plot(x,y);
    end
end
title('Robot path');
hold on;




