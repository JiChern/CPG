function tm = theta_coupling_mat(theta)
tm = zeros(6,6);
for i=1:6
    for j=1:6
        tm(i,j) = theta(j)-theta(i);
    end
end

end
