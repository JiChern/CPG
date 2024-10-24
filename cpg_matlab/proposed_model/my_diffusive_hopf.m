function [z_mat, omega_amps, z_bug1, z_bug2, z_bug3] = my_diffusive_hopf(z_mat, dt, gamma,  theta, t)
    omega_amps = [];
    for i = 1:6
        z_vec = z_mat(:,i);
        R = rotation_mat(theta(i));
        if i < 6
            
            % coeff = 7-i;
            coeff = 1;

            z_vec_ = z_mat(:,i+1);
            z_vec_ = R*z_vec_;

            theta_l = cal_theta_l(z_vec_,z_vec);
            R_theta_l = rotation_mat(theta_l);
            
            vel_vec = z_vec_-z_vec;
            vec_length = norm(vel_vec) + 0.001;
            
            omega_dir = R_theta_l*vel_vec/vec_length;
            omega_length = vec_length*cos(theta_l);
            omega_vec = omega_length*omega_dir;
            
            omega_amps = [omega_amps,omega_length];

            z_dot = hopfOscillator(z_vec) + coeff*gamma*omega_vec; 



            % z_dot = hopfOscillator(z_vec) + coeff*gamma*R_theta_l*(z_vec_ - z_vec);   
        else
            % coeff = 7-1;
            coeff = 1;

            z_vec = z_mat(:,6);
            coeff = 10;
            z_vec_ = z_mat(:,1);
            z_vec_ = R*z_vec_;

            theta_l = cal_theta_l(z_vec_,z_vec);
            R_theta_l = rotation_mat(theta_l);
            % R_theta_l = eye(2,2);
            vel_vec = z_vec_-z_vec;
            vec_length = norm(vel_vec) + 0.001;
            
            omega_dir = R_theta_l*vel_vec/vec_length;
            omega_length = vec_length*cos(theta_l);


            z_bug1 = z_vec_-z_vec;
            z_bug2 = z_vec_;
            z_bug3 = z_vec;

            z_vec_
            z_vec

            sprintf('vec_len: %d.',vec_length)
            sprintf('omega_len: %d.',omega_length)


            omega_vec = omega_length*omega_dir;
           
            z_dot = hopfOscillator(z_vec) + coeff*gamma*omega_vec; 
            
            omega_amps = [omega_amps,omega_length];

        end

        z_mat(:,i) = z_vec + z_dot*dt;

    end

end