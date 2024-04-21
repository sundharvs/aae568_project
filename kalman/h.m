function z = h(red, blue)
    blue(4:7) = eul2quat(blue(4:6)');

    alpha = 333.507935;
    beta = 334.712952;
    u0 = 351.613402;
    v0 = 208.431513;

    Rg = [red(1); red(4); red(7)] - [blue(1); blue(2); blue(3)];
    q = [blue(4), blue(5), blue(6), blue(7)];
    Rb = quat2dcm(q) * Rg;

    r = norm(Rb);
    u = -(Rb(2) / Rb(1)) * alpha + u0;
    v = -(Rb(3) / Rb(1)) * beta + v0;

    z = [r; u; v];
end