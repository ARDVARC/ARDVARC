close all
figs = findall(0,'type','figure');
close(figs)
clc
clear

fig = uifigure;
g = uigridlayout(fig);
ax = uiaxes(g);
ax.XLim = [0 11];
ax.YLim = [0 11];
grid(ax,"minor")
plot(ax,[0,1],[0,1]);
sld = uislider(g,"Limits",[0 10]);
sld.ValueChangingFcn = @(~,event) myfunc(event, ax);
%%
function myfunc(event,ax)
    x = event.Value;

    plot(ax,[x,x+1],[x,x+1])
end