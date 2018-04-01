function plot_3d(h, wpt)
%
% plots quad path in 3d
%
figure;
keys = get_plot_keys;
m = containers.Map(keys, [1:length(keys)]);
plot3(h(:,m('pos_des_x')), h(:,m('pos_des_y')), ...
    h(:,m('pos_des_z')),'b');
hold on;
plot3(h(:,m('pos_x')), h(:,m('pos_y')), h(:,m('pos_z')),'r');
if ~isempty(wpt)
    plot3(wpt(:,1),wpt(:,2),wpt(:,3), 'b-o');
end
hold off;
title("Planned Vs Actual Trajectory Task 3");
xlabel("Position X"); 
ylabel("Poistion Y");
zlabel("Poistion Z");

end