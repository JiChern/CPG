function z_mat = r_mat_hopf(z_mat, dt, c_theta)
    alpha = 10;
    mu = 1;
    omega = 2*pi;
    for i = 1:6
        z_vec = z_mat(:,i);
        r_i2 = z_vec(1)^2+z_vec(2)^2;
        A = [alpha*(mu-r_i2), -omega;
             omega, alpha*(mu-r_i2)];

        z_dot = A*z_vec + r_coupling(c_theta,z_mat,i);  

        z_mat(:,i) = z_vec + z_dot*dt;

    end

end