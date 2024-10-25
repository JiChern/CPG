function z_mat = diffusive_hopf(z_mat, dt, gamma, theta)

    for i = 1:6
        z_vec = z_mat(:,i);
        R = rotation_mat(theta(i));
        if i < 6
            %coeff = 7-i;
            coeff = 1;
            z_vec_ = z_mat(:,i+1);
            z_dot = hopfOscillator(z_vec) + coeff*gamma*(R*z_vec_ - z_vec);     
        else
            %coeff = 7-1;
            coeff = 1;
            z_vec1 = z_mat(:,1);
            z_dot = hopfOscillator(z_vec) + coeff*gamma*(R*z_vec1 - z_vec); 
        end

        z_mat(:,i) = z_vec + z_dot*dt;

    end

end