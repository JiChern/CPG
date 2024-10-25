function R = rotation_mat(theta)
    R = [cos(theta), sin(theta);
        -sin(theta), cos(theta)];
end