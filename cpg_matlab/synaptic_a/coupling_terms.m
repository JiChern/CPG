function c_t = coupling_terms(y,C,dim)
    c_y = zeros(dim,dim);
    for i=1:dim
        for j=1:dim
            c_y(i,j)=C(i,j)*y(i);
        end
    end
    c_t = sum(c_y);
end

