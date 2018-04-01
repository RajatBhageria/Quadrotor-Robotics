
function plot_angles(h)
%
% plot angles
%
    fig1 = figure(1);
    fig1.Name = 'roll';
    plot_angle(h, 'roll_des', 'roll');
    fig2 = figure(2);
    fig2.Name = 'pitch';
    plot_angle(h, 'pitch_des', 'pitch');
    fig3 = figure(3);
    fig3.Name = 'yaw';
    plot_angle(h, 'yaw_des', 'yaw');
end

function plot_angle(h, key1, key2)
  plot_data(h, 'angle[rad]', key1, key2);
  xlabel('time [s]');
end
