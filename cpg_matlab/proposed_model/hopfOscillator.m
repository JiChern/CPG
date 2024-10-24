
function z_dot = hopfOscillator(z)
    mu = 1;
    omega = 2*pi;
    alpha = 10;
    beta = 10;

    z_dot = [0;0];
    r_square = z(1)^2+z(2)^2;
    z_dot(1) = alpha*(mu-r_square)*z(1)-omega*z(2);
    z_dot(2) = beta*(mu-r_square)*z(2)+omega*z(1);
    
    

end