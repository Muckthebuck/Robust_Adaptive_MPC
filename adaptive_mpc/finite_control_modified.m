function [ xstar, ustar, Jstar, exitflag ] = finite_control_modified( P, Q, R, A, B, Ax, bx, Au, bu, Af, bf, N, x0, S )

[H, f, Aineq, bineq, Aeq, beq, n_x, n_u] = construct_QP_modified(P, Q, R, A, B, Ax, bx, Au, bu, Af, bf, N, x0, S);


[zstar, Jstar, exitflag] = quadprog(H, f, Aineq, bineq, Aeq, beq);

if exitflag == 1
    xstar = reshape(zstar(1:n_x), 2, N+1);
    ustar = zstar(n_x+1:end);
else
    xstar = []; ustar = [];
end

end

