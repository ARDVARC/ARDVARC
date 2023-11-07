function plot3DVectorsComingFrom(trix_vec_comeFroms, trix_vec_vecs, fignum, linespec)
    arguments(Input)
        trix_vec_comeFroms (:,3) double
        trix_vec_vecs (:,3) double
        fignum (1,1) double
        linespec (1,1) string
    end

    figure(fignum)
    count = size(trix_vec_comeFroms,1);
    for i = 1:count
        plot3([trix_vec_comeFroms(i,1), trix_vec_comeFroms(i,1) + trix_vec_vecs(i,1)], [trix_vec_comeFroms(i,2), trix_vec_comeFroms(i,2) + trix_vec_vecs(i,2)], [trix_vec_comeFroms(i,3), trix_vec_comeFroms(i,3) + trix_vec_vecs(i,3)], linespec)
        hold on
    end
end