function  [Sphere_plane_estimate_vec] = Euc2Sphere(Euc_plane_estimate_vec, param)

    Sphere_plane_estimate_vec = zeros(1, 3 * param.plane_num);


    for plane_i = 1:param.plane_num
        plane_sph_id = (plane_i - 1) * 3;
        plane_euc_id = (plane_i - 1) * 4;

        euc_vec = Euc_plane_estimate_vec(1, plane_euc_id+1:plane_euc_id+4);
        
        phi = acos(euc_vec(3));
        theta = acos(euc_vec(1) / sin(phi));

        if euc_vec(2) < 0
            theta = -theta;
        end
        
        sph_vec = [phi, theta, euc_vec(4)];

        Sphere_plane_estimate_vec(1, plane_sph_id+1:plane_sph_id+3) = sph_vec;
    end
end