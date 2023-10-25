function a = ardvarc_steervec(angs, numel, distance, lambda)
    %Outputs:
        %a: Matrix of steering vectors for all possible angles
    %Inputs:
        %angs: column vector of all possible angles of arrival
        %numel: number of elements in antenna array
        %d: distance between elements in meters
        %lambda: wavelength in meters
        
    a1 = zeros(length(angs), (numel-1));
    for i = 1:(numel-1)
       if i == 1;
           a1(:,i) = ones(length(angs), 1);
       else
           a1(:,i) = exp((j*2*pi*d*sin(angs))/lambda);
       end
    end
    
    a = a1;