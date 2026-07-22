function y = DALTONICS_dibujo(x)

persistent ax hT tr X t0
y = 0; x = x(:);

if isempty(ax) || ~isgraphics(ax)
    f = figure(99); clf(f); set(f,'Name','Dron Daltonics en vivo (STL)','Color','w');
    ax = axes('Parent',f); hold(ax,'on'); grid(ax,'on'); view(ax,45,20);
    axis(ax,'equal');
    [gx,gy] = meshgrid(-3:1:3);
    surf(ax,gx,gy,0*gx,'FaceColor',[0.9 0.95 0.9],'EdgeColor',[0.8 0.85 0.8]);

    [F, Vb] = DALTONICS_cargar_stl();

    hT = hgtransform('Parent',ax);
    patch('Parent',hT,'Faces',F,'Vertices',Vb, ...
        'FaceColor',[0.25 0.55 0.85],'EdgeColor','none', ...
        'FaceLighting','gouraud','AmbientStrength',0.4);
    camlight(ax,'headlight'); lighting(ax,'gouraud'); material(ax,'dull');

    tr = plot3(ax,nan,nan,nan,'b-');
    xlabel(ax,'x [m]'); ylabel(ax,'y [m]'); zlabel(ax,'z [m]');
    X = []; t0 = tic;
end

if toc(t0) < 0.08, return; end
t0 = tic;

pos = x(1:3);

% *** CLAMPING: nunca dibujar bajo el suelo ***
pos(3) = max(pos(3), 0);

ph = x(7); th = x(8); ps = x(9);
R = [cos(ps)*cos(th), cos(ps)*sin(th)*sin(ph)-sin(ps)*cos(ph), cos(ps)*sin(th)*cos(ph)+sin(ps)*sin(ph);
     sin(ps)*cos(th), sin(ps)*sin(th)*sin(ph)+cos(ps)*cos(ph), sin(ps)*sin(th)*cos(ph)-cos(ps)*sin(ph);
     -sin(th), cos(th)*sin(ph), cos(th)*cos(ph)];

Tm = eye(4); Tm(1:3,1:3) = R; Tm(1:3,4) = pos;
set(hT,'Matrix',Tm);

X(:,end+1) = pos;
if size(X,2) > 400, X = X(:,2:end); end

set(tr,'XData',X(1,:),'YData',X(2,:),'ZData',max(X(3,:),0));

axis(ax,[pos(1)-1.5 pos(1)+1.5  pos(2)-1.5 pos(2)+1.5  0  max(2, pos(3)+1)]);
title(ax,sprintf('Dron Daltonics en vivo (STL)  |  altura z = %.2f m', pos(3)));
drawnow limitrate
end
