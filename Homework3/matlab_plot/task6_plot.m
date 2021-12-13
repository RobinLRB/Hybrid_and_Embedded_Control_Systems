clc
clear
format short;
a = readmatrix("pos_log.csv");
t = linspace(0,700,700);
figure
error_theta = -135 - a(1:700,4);
plot(t ,error_theta)
%%plot((a(:,1)*.1e-12) ,(-135 - a(:,4)))
legend({'error theta'},'Location','northwest')