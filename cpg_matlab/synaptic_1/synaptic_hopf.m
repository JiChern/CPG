function z_mat_o = synaptic_hopf(z_mat, dt, C, dim)

    alpha = 10;
    beta = 10;
    omega = 2*pi;
    mu = ones(1,dim);
    x = z_mat(1,:);
    y = z_mat(2,:);

    r_2 = x.^2 + y.^2;
    c_y = coupling_terms(y,C, dim);

    dx = alpha * (mu - r_2) .* x - omega * y;
    dy = beta * (mu - r_2) .* y + omega * x + c_y;

    z_mat_o(1,:) = x + dx*dt;
    z_mat_o(2,:) = y + dy*dt;

end
