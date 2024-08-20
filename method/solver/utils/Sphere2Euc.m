function  [Euc_plane_estimate_vec] = Sphere2Euc(Sphere_plane_estimate_vec, param)

    Euc_plane_estimate_vec = zeros(1, 4 * param.plane_num);


    for plane_i = 1:param.plane_num
        plane_sph_id = (plane_i - 1) * 3;
        plane_euc_id = (plane_i - 1) * 4;

        sph_vec = Sphere_plane_estimate_vec(1, plane_sph_id+1:plane_sph_id+3);

        euc_vec = [sin(sph_vec(1))*cos(sph_vec(2)),sin(sph_vec(1))*sin(sph_vec(2)),cos(sph_vec(1)),sph_vec(3)];

        Euc_plane_estimate_vec(1, plane_euc_id+1:plane_euc_id+4) = euc_vec;
    end
end