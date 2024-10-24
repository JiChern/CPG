function theta_l = cal_theta_l(z1,z2)
    epsilon = 0.001;
    
    norm_z1 = norm(z1);
    norm_z2 = norm(z2);
    n_z1z2 =  norm_z1*norm_z2 + epsilon;

    c_theta = dot(z1,z2)/(n_z1z2);

    theta_l = acos(c_theta);

    cross_prod = cross([z1;0],[z2;0]);
    
    if cross_prod(3) > 0
        theta_l = -theta_l;
    end
    
    theta_l = theta_l/2;

end