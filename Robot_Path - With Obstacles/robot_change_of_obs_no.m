%------------ Optimal Robot Path Planning ---------------------------------------
% Code:     Coverage Path Planning Algorithm - No Obstacles
% Authors:  Ankit Manerikar, Debasmit Das, Pranay Banerjee
% Date:     04/11/2016
% Course:   AAE568
%--------------------------------------------------------------------------------
clc
close all
clear all

% Generate Independent Variables ------------------------------------------
toms t
toms tf
p = tomPhase('p', t, 0, tf, 10, [], 'fem1s'); % Use splines with FEM constraints
%   p = tomPhase('p', t, 0, tf, 100);           % Use linear finite elements
%  p = tomPhase('p', t, 0, 2, 100);            % Use Gauss point collocation
setPhase(p);

% Generate States and Control Variables -----------------------------------
tomStates   x1 x2 x3 x4
tomControls u1 u2

%Robot/Area Dimensions-----------------------------------------------------------
w =0.5;                       % weighing constant
radr = 0.25;                  % radius of coverage

Dx = 10;
Dy = 10;

x1_var = [];
x2_var = [];
x3_var = [];
x4_var = [];
u1_var = [];
u2_var = [];
pass_time = [];
energy = [];

for obsno = 2:2:20
%Obstacle Parameters ------------------------------------------------------ 
rad = 0.22*rand(obsno,1);
xrnd = 8*rand(obsno,1)+1;
yrnd = 8*rand(obsno,1)+1;

% Generate Obstacles ------------------------------------------------------
figure(1)

rectangle('Position',[0,0,Dx,Dy],'Curvature', [0 0],'LineWidth',1.5,'Linestyle',':');
for i = 1:obsno
rectangle('Position',[xrnd(i)-sqrt(rad(i)),yrnd(i)-sqrt(rad(i)),2*sqrt(rad(i)),2*sqrt(rad(i))],'Curvature', [1 1]);
end
box on;
xlim([-1 11]);
ylim([-1 11]);
title('Robot Coverage Path - Number of Obstacles = 10');
xlabel('X co-ordinate: State Variable, x_1');
ylabel('Y co-ordinate: State Variable, x_2');
hold on;
% pause(0.001);

for i = radr:2*radr:Dx-radr

iter = round((i-radr)/(2*radr));

x0 = { tf == 10 
     icollocate({x1 == t; x2 == i; x3 == 1; x4 == 1})
     collocate({u1 == 1; u2==1})};

cbox = { 0  <= collocate(x1) <= 10
         i-2*radr+0.01 <= collocate(x2) <= i+2*radr};

cbnd = {initial({x1 == 0; x2 == i
                 x3 == 1; x4 == 0})
        final({  x1 == 10; x2 == i
                 x3 == 1; x4 == 0})};

%--------------------------------------------------------------------------
switch(obsno)
    case 2
        obstacles = atPoints(linspace(0,tf,300), {
(x1 - xrnd(1))^2 + (x2 - yrnd(1))^2 >= rad(1)
(x1 - xrnd(2))^2 + (x2 - yrnd(2))^2 >= rad(2)});

    case 4
        obstacles = atPoints(linspace(0,tf,300), {
(x1 - xrnd(1))^2 + (x2 - yrnd(1))^2 >= rad(1)
(x1 - xrnd(2))^2 + (x2 - yrnd(2))^2 >= rad(2)
(x1 - xrnd(3))^2 + (x2 - yrnd(3))^2 >= rad(3)
(x1 - xrnd(4))^2 + (x2 - yrnd(4))^2 >= rad(4)});

    case 6
        obstacles = atPoints(linspace(0,tf,300), {
(x1 - xrnd(1))^2 + (x2 - yrnd(1))^2 >= rad(1)
(x1 - xrnd(2))^2 + (x2 - yrnd(2))^2 >= rad(2)
(x1 - xrnd(3))^2 + (x2 - yrnd(3))^2 >= rad(3)
(x1 - xrnd(4))^2 + (x2 - yrnd(4))^2 >= rad(4)
(x1 - xrnd(5))^2 + (x2 - yrnd(5))^2 >= rad(5)
 (x1 - xrnd(6))^2 + (x2 - yrnd(6))^2 >= rad(6)});

    case 8
obstacles = atPoints(linspace(0,tf,300), {
(x1 - xrnd(1))^2 + (x2 - yrnd(1))^2 >= rad(1)
(x1 - xrnd(2))^2 + (x2 - yrnd(2))^2 >= rad(2)
(x1 - xrnd(3))^2 + (x2 - yrnd(3))^2 >= rad(3)
(x1 - xrnd(4))^2 + (x2 - yrnd(4))^2 >= rad(4)
 (x1 - xrnd(5))^2 + (x2 - yrnd(5))^2 >= rad(5)
 (x1 - xrnd(6))^2 + (x2 - yrnd(6))^2 >= rad(6)
 (x1 - xrnd(7))^2 + (x2 - yrnd(7))^2 >= rad(7)
 (x1 - xrnd(8))^2 + (x2 - yrnd(8))^2 >= rad(8)});

    case 10
        obstacles = atPoints(linspace(0,tf,300), {
(x1 - xrnd(1))^2 + (x2 - yrnd(1))^2 >= rad(1)
(x1 - xrnd(2))^2 + (x2 - yrnd(2))^2 >= rad(2)
(x1 - xrnd(3))^2 + (x2 - yrnd(3))^2 >= rad(3)
(x1 - xrnd(4))^2 + (x2 - yrnd(4))^2 >= rad(4)
 (x1 - xrnd(5))^2 + (x2 - yrnd(5))^2 >= rad(5)
 (x1 - xrnd(6))^2 + (x2 - yrnd(6))^2 >= rad(6)
 (x1 - xrnd(7))^2 + (x2 - yrnd(7))^2 >= rad(7)
 (x1 - xrnd(8))^2 + (x2 - yrnd(8))^2 >= rad(8)
 (x1 - xrnd(9))^2 + (x2 - yrnd(9))^2 >= rad(9)
(x1 - xrnd(10))^2 + (x2 - yrnd(10))^2 >= rad(10)});

    case 12    
        obstacles = atPoints(linspace(0,tf,300), {
(x1 - xrnd(1))^2 + (x2 - yrnd(1))^2 >= rad(1)
(x1 - xrnd(2))^2 + (x2 - yrnd(2))^2 >= rad(2)
(x1 - xrnd(3))^2 + (x2 - yrnd(3))^2 >= rad(3)
(x1 - xrnd(4))^2 + (x2 - yrnd(4))^2 >= rad(4)
 (x1 - xrnd(5))^2 + (x2 - yrnd(5))^2 >= rad(5)
 (x1 - xrnd(6))^2 + (x2 - yrnd(6))^2 >= rad(6)
 (x1 - xrnd(7))^2 + (x2 - yrnd(7))^2 >= rad(7)
 (x1 - xrnd(8))^2 + (x2 - yrnd(8))^2 >= rad(8)
 (x1 - xrnd(9))^2 + (x2 - yrnd(9))^2 >= rad(9)
(x1 - xrnd(10))^2 + (x2 - yrnd(10))^2 >= rad(10)
(x1 - xrnd(11))^2 + (x2 - yrnd(11))^2 >= rad(11)
 (x1 - xrnd(12))^2 + (x2 - yrnd(12))^2 >= rad(12)});

    case 14
        obstacles = atPoints(linspace(0,tf,300), {
(x1 - xrnd(1))^2 + (x2 - yrnd(1))^2 >= rad(1)
(x1 - xrnd(2))^2 + (x2 - yrnd(2))^2 >= rad(2)
(x1 - xrnd(3))^2 + (x2 - yrnd(3))^2 >= rad(3)
(x1 - xrnd(4))^2 + (x2 - yrnd(4))^2 >= rad(4)
 (x1 - xrnd(5))^2 + (x2 - yrnd(5))^2 >= rad(5)
 (x1 - xrnd(6))^2 + (x2 - yrnd(6))^2 >= rad(6)
 (x1 - xrnd(7))^2 + (x2 - yrnd(7))^2 >= rad(7)
 (x1 - xrnd(8))^2 + (x2 - yrnd(8))^2 >= rad(8)
 (x1 - xrnd(9))^2 + (x2 - yrnd(9))^2 >= rad(9)
(x1 - xrnd(10))^2 + (x2 - yrnd(10))^2 >= rad(10)
(x1 - xrnd(11))^2 + (x2 - yrnd(11))^2 >= rad(11)
 (x1 - xrnd(12))^2 + (x2 - yrnd(12))^2 >= rad(12)
(x1 - xrnd(13))^2 + (x2 - yrnd(13))^2 >= rad(13)
 (x1 - xrnd(14))^2 + (x2 - yrnd(14))^2 >= rad(14)});

    case 16
        obstacles = atPoints(linspace(0,tf,300), {
(x1 - xrnd(1))^2 + (x2 - yrnd(1))^2 >= rad(1)
(x1 - xrnd(2))^2 + (x2 - yrnd(2))^2 >= rad(2)
(x1 - xrnd(3))^2 + (x2 - yrnd(3))^2 >= rad(3)
(x1 - xrnd(4))^2 + (x2 - yrnd(4))^2 >= rad(4)
 (x1 - xrnd(5))^2 + (x2 - yrnd(5))^2 >= rad(5)
 (x1 - xrnd(6))^2 + (x2 - yrnd(6))^2 >= rad(6)
 (x1 - xrnd(7))^2 + (x2 - yrnd(7))^2 >= rad(7)
 (x1 - xrnd(8))^2 + (x2 - yrnd(8))^2 >= rad(8)
 (x1 - xrnd(9))^2 + (x2 - yrnd(9))^2 >= rad(9)
(x1 - xrnd(10))^2 + (x2 - yrnd(10))^2 >= rad(10)
(x1 - xrnd(11))^2 + (x2 - yrnd(11))^2 >= rad(11)
 (x1 - xrnd(12))^2 + (x2 - yrnd(12))^2 >= rad(12)
(x1 - xrnd(13))^2 + (x2 - yrnd(13))^2 >= rad(13)
 (x1 - xrnd(14))^2 + (x2 - yrnd(14))^2 >= rad(14)
(x1 - xrnd(15))^2 + (x2 - yrnd(15))^2 >= rad(15)
 (x1 - xrnd(16))^2 + (x2 - yrnd(16))^2 >= rad(16)});

    case 18
        obstacles = atPoints(linspace(0,tf,300), {
(x1 - xrnd(1))^2 + (x2 - yrnd(1))^2 >= rad(1)
(x1 - xrnd(2))^2 + (x2 - yrnd(2))^2 >= rad(2)
(x1 - xrnd(3))^2 + (x2 - yrnd(3))^2 >= rad(3)
(x1 - xrnd(4))^2 + (x2 - yrnd(4))^2 >= rad(4)
 (x1 - xrnd(5))^2 + (x2 - yrnd(5))^2 >= rad(5)
 (x1 - xrnd(6))^2 + (x2 - yrnd(6))^2 >= rad(6)
 (x1 - xrnd(7))^2 + (x2 - yrnd(7))^2 >= rad(7)
 (x1 - xrnd(8))^2 + (x2 - yrnd(8))^2 >= rad(8)
 (x1 - xrnd(9))^2 + (x2 - yrnd(9))^2 >= rad(9)
(x1 - xrnd(10))^2 + (x2 - yrnd(10))^2 >= rad(10)
(x1 - xrnd(11))^2 + (x2 - yrnd(11))^2 >= rad(11)
 (x1 - xrnd(12))^2 + (x2 - yrnd(12))^2 >= rad(12)
(x1 - xrnd(13))^2 + (x2 - yrnd(13))^2 >= rad(13)
 (x1 - xrnd(14))^2 + (x2 - yrnd(14))^2 >= rad(14)
(x1 - xrnd(15))^2 + (x2 - yrnd(15))^2 >= rad(15)
 (x1 - xrnd(16))^2 + (x2 - yrnd(16))^2 >= rad(16)
 (x1 - xrnd(17))^2 + (x2 - yrnd(17))^2 >= rad(17)
 (x1 - xrnd(18))^2 + (x2 - yrnd(18))^2 >= rad(18)});
    
    case 20
        obstacles = atPoints(linspace(0,tf,300), {
(x1 - xrnd(1))^2 + (x2 - yrnd(1))^2 >= rad(1)
(x1 - xrnd(2))^2 + (x2 - yrnd(2))^2 >= rad(2)
(x1 - xrnd(3))^2 + (x2 - yrnd(3))^2 >= rad(3)
(x1 - xrnd(4))^2 + (x2 - yrnd(4))^2 >= rad(4)
 (x1 - xrnd(5))^2 + (x2 - yrnd(5))^2 >= rad(5)
 (x1 - xrnd(6))^2 + (x2 - yrnd(6))^2 >= rad(6)
 (x1 - xrnd(7))^2 + (x2 - yrnd(7))^2 >= rad(7)
 (x1 - xrnd(8))^2 + (x2 - yrnd(8))^2 >= rad(8)
 (x1 - xrnd(9))^2 + (x2 - yrnd(9))^2 >= rad(9)
(x1 - xrnd(10))^2 + (x2 - yrnd(10))^2 >= rad(10)
(x1 - xrnd(11))^2 + (x2 - yrnd(11))^2 >= rad(11)
 (x1 - xrnd(12))^2 + (x2 - yrnd(12))^2 >= rad(12)
(x1 - xrnd(13))^2 + (x2 - yrnd(13))^2 >= rad(13)
 (x1 - xrnd(14))^2 + (x2 - yrnd(14))^2 >= rad(14)
(x1 - xrnd(15))^2 + (x2 - yrnd(15))^2 >= rad(15)
 (x1 - xrnd(16))^2 + (x2 - yrnd(16))^2 >= rad(16)
 (x1 - xrnd(17))^2 + (x2 - yrnd(17))^2 >= rad(17)
 (x1 - xrnd(18))^2 + (x2 - yrnd(18))^2 >= rad(18)
 (x1 - xrnd(19))^2 + (x2 - yrnd(19))^2 >= rad(19)
 (x1 - xrnd(20))^2 + (x2 - yrnd(20))^2 >= rad(20)});
    otherwise
end

% ODEs and path constraints------------------------------------------------

ceq = collocate({                                                               % state equations
    dot(x1) == x3
    dot(x2) == x4
    dot(x3) == u1
    dot(x4) == u2});

tot_cost = integrate(w + (1 - w)*(u1^2 + u2^2));                   
% Objective----------------------------------------------------------------
objective = tot_cost;

% Solution of Differential Equation Set------------------------------------
options = struct;
options.name = 'Optimal Robot Path Planning ';
solution = ezsolve(objective, {cbox, cbnd, ceq,obstacles}, x0 , options);
t_p  = subs(icollocate(t),solution);
x1_p = subs(icollocate(x1),solution);
x2_p = subs(icollocate(x2),solution);
x3_p = subs(icollocate(x3),solution);
x4_p = subs(icollocate(x4),solution);
u1_p = subs(icollocate(u1),solution);
u2_p = subs(icollocate(u2),solution);

% area_p = subs(area,'x3', x3_p,'x4', x4_p, 't', t_p);

x1_var = cat(1,x1_var,x1_p);
x2_var = cat(1,x2_var,x2_p);
x3_var = cat(1,x3_var,x3_p);
x4_var = cat(1,x4_var,x4_p);
u1_var = cat(1,u1_var,u1_p);
u2_var = cat(1,u2_var,u2_p);

area_obstacle = 0;
for ht = 1:obsno
    if yrnd(ht)+ sqrt(rad(ht)) <= i
        area_obstacle = area_obstacle + pi*rad(ht);
    end    
end

path_iter(iter+1) = (2*radr*trapz(sqrt(x1_p(1:length(x1_p)-1).^2+x2_p(1:length(x2_p)-1).^2))/5 - area_obstacle)/100;
time_iter(iter+1) = max(t_p);
energy_iter(iter+1) = 0.1*trapz(sqrt(u1_p.^2+u2_p.^2));

plot(x1_p,x2_p,'b')

if i == 10-radr
    break;
elseif ((mod(round((i-radr)/(2*radr)),2)) == 0)  
    plot([10 10]',[i i+2*radr]','r','LineWidth',1.5);
elseif ((mod(round((i-radr)/(2*radr)),2)) == 1)
    plot([0 0]',[i i+2*radr]','r','LineWidth',1.5);
end

hold on;
% pause(0.001);

if i == radr
    text(x1_p(1),x2_p(1), ' Initial Point \rightarrow', 'HorizontalAlignment', 'right');
end

end
path_cum = cumsum(path_iter);
pass_time_cum = cumsum(time_iter);
energy_cum = cumsum(energy_iter);

path_covered(obsno) = max(path_iter);
pass_time(obsno) = max(pass_time_cum);
energy(obsno) = max(energy_cum);

end


figure(2)
subplot(3,1,1)
box on;
plot(1:10,[path_covered])
title('Coverage Path Length Ratio v/s. Obstacle Number')
% ylim([ 105]);
xlabel('Obstacle Number, N_{obs}')
ylabel('Coverage Path Length Ratio,')
grid on

% xlim([0 iter+2])
% ylim([0 101])


subplot(3,1,2)
box on;
plot(1:10, [pass_time]);
title('Total Coverage Time v/s. Obstacle Number')
xlabel('Obstacle Number, N_{obs}')
ylabel('Time Taken')
grid on

% xlim([0 iter+2])

subplot(3,1,3)
box on;
plot(1:10,[energy])
title('Total Control Energy v/s. Obstacle Number')
xlabel('Obstacle Number, N_{obs}')
ylabel('Energy')
grid on
% xlim([0 iter+2])