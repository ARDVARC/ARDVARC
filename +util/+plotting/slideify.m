function slideify(fignum, bgColor, textColor)
    % Changes the colors of a figure to match the ARDVARC slideshow theme
    arguments(Input)
        fignum (1,1) double = -1
        bgColor (1,1) string = "#37474f"
        textColor (1,1) string = "white"
    end
    
    if (fignum == -1)
        f = gcf;
    else
        f = figure(fignum);
    end
    
    f.Color = bgColor;
    for i = 1:length(f.Children)
        g = f.Children(i);
        if (isa(g, "matlab.graphics.axis.Axes"))
            % Make axes child match slide color theme
            g.XColor = textColor;
            g.YColor = textColor;
            g.ZColor = textColor;
            g.Color = bgColor;
            g.Title.Color = textColor;
            for j = 1:length(g.Children)
                gg = g.Children(j);
                if (isa(gg, "matlab.graphics.primitive.Text"))
                    gg.Color = textColor;
                end
            end
        elseif(isa(g, "matlab.graphics.illustration.Legend"))
            % Make legend child match slide color theme 
            g.Color = bgColor;
            g.TextColor = textColor;
            g.EdgeColor = textColor;
        elseif(isa(g, "matlab.graphics.illustration.subplot.Text"))
            % Make text child match slide color theme 
            g.Color = textColor;
        else
            fprintf("Slideify - Unrecognized child type '%s'\n", class(g))
        end
    end
end