function R_near = SearchNearestRotation(R)
    [U,S,V] = svd(R);
    Sigma_new = diag([1,1,det(U*V')]);
    R_near = U*Sigma_new*V';
end