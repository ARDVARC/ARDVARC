function frame = paintFrame(frame,xy,radius,colorIndex,colorValue)
    for x = max(1,xy(1)-radius):min(size(frame,1),xy(1)+radius)
        for y = max(1,xy(2)-radius):min(size(frame,2),xy(2)+radius)
            frame(x,y,colorIndex) = colorValue;
        end
    end
end