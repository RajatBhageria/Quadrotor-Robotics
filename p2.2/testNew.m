close all;
clear all;

load('data/studentdata1.mat');

init_script;
V = zeros(6, length(data));
Vc = zeros(size(V));
tc = zeros(length(data),1);
t = zeros(length(data),1);

tic;
for i = 1:length(data)
    [vel, omg] = estimate_vel_handle(data(i));
    V(:,i) = [vel; omg];
    t(i) = data(i).t;
    [~, it] = min(abs(data(i).t-time));
    tc(i) = time(it);
    Vc(:,i) = vicon(7:12,it);
end
toc;

%%
figure(1);

i = 1;
SH1 = subplot(2,3,i);
scatter(t, V(i,:), '*k');
hold on;
scatter(tc, Vc(i,:), '*r');
title('Vx');

%%
i = 2;
subplot(2,3,i);
scatter(t, V(i,:), '*k');
hold on;
scatter(tc, Vc(i,:), '*r');
title('Vy');

%%
i = 3;
subplot(2,3,i);
scatter(t, V(i,:), '*k');
hold on;
scatter(tc, Vc(i,:), '*r');
title('Vz');

%%
i = 4;
subplot(2,3,i);
scatter(t, V(i,:), '*k');
hold on;
scatter(tc, Vc(i,:), '*r');
title('wx');

%%
i = 5;
subplot(2,3,i);
scatter(t, V(i,:), '*k');
hold on;
scatter(tc, Vc(i,:), '*r');
title('wy');

%%
i = 6;
subplot(2,3,i);
scatter(t, V(i,:), '*k');
hold on;
scatter(tc, Vc(i,:), '*r');
title('wz');