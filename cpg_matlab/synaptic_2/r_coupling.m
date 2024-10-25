function r_c = r_coupling(c_theta,z_mat,i)
    rc_mat = zeros(2,6);
    for j=1:6
        rc_mat(:,j) = r_mat(c_theta(i,j))*z_mat(:,j);
    end
    
    r_c = [0;0];

    for j = 1:6
        if j == i
            continue;
        else
            r_c = r_c + rc_mat(:,j);
        end
    end

end