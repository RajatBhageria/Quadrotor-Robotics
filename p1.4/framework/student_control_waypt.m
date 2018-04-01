function [desired_state] = student_control_waypt(t, qd, path)
  
persistent t_init;
  
if nargin > 1
    path = [qd{1}.pos.'; path];
    trajectory_generator([], [], 0, path);
    
    t_init = t;
    return
end

qn = 1;
desired_state = trajectory_generator(t-t_init, qn);

end
