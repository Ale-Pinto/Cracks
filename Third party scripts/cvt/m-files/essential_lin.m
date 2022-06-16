function E = essential_lin(q2,q1,K1,K2)
%ESSENTIAL_LIN  Essential matrix with 8-points algorithm
 
    q1 = K1\ensure_homogeneous(q1); 
    q2 = K2\ensure_homogeneous(q2); 
    % now q1 and q2 are homogeneous NIC
    
    E = eight_points(q2(1:2,:), q1(1:2,:));
    
    % enforce constraints on E
    [U,D,V] = svd(E);
    s = (D(1,1)+D(2,2))/2;
    E = U * diag([s,s,0]) * V';
    
end













