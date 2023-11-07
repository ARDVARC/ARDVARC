function p = plot3DVectorsComingFrom(trix_vec_comeFroms, trix_vec_vecs, fignum, linespec, displayName)
    arguments(Input)
        trix_vec_comeFroms (:,3) double
        trix_vec_vecs (:,3) double
        fignum (1,1) double
        linespec (1,1) string = "k"
        displayName (1,1) string = ""
    end

    figure(fignum)
    count = size(trix_vec_comeFroms,1);
    for i = 1:count
        p = plot3([trix_vec_comeFroms(i,1), trix_vec_comeFroms(i,1) + trix_vec_vecs(i,1)], [trix_vec_comeFroms(i,2), trix_vec_comeFroms(i,2) + trix_vec_vecs(i,2)], [trix_vec_comeFroms(i,3), trix_vec_comeFroms(i,3) + trix_vec_vecs(i,3)], linespec, DisplayName=displayName);
        hold on
    end
end