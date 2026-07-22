function DALTONICS_animacion(out)

if nargin < 1
    out = evalin('base','out');
end
t = out.x_log.time;
X = out.x_log.signals.values;      % N x 12

global P
if isempty(P), DALTONICS_parametros; end

[F, Vb] = DALTONICS_cargar_stl();

% ---------- figura ----------
fig = figure('Name','Dron Daltonics - Gemelo Digital','Color','w');
ax  = axes('Parent',fig); hold(ax,'on'); grid(ax,'on'); view(ax,45,20);
axis(ax,'equal');
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
title('Simulacion Dron Daltonics');

% suelo
[gx,gy] = meshgrid(-3:0.5:3);
surf(ax,gx,gy,0*gx,'FaceColor',[0.92 0.95 0.92],'EdgeColor',[0.8 0.85 0.8]);

hT = hgtransform('Parent',ax);
patch('Parent',hT,'Faces',F,'Vertices',Vb, ...
    'FaceColor',[0.25 0.55 0.85],'EdgeColor','none', ...
    'FaceLighting','gouraud','AmbientStrength',0.4);
camlight(ax,'headlight'); lighting(ax,'gouraud'); material(ax,'dull');

h_trail = plot3(ax,nan,nan,nan,'b-','LineWidth',1);

% submuestreo para ~25 FPS de vuelo
paso = max(1, round(numel(t)/(t(end)*25)));

for k = 1:paso:numel(t)
    if ~isvalid(fig), return; end

    pos = X(k,1:3)';
    eul = X(k,7:9)';

    % *** CLAMPING: el dron nunca se dibuja bajo el suelo ***
    pos(3) = max(pos(3), 0);

    R = rotZYX(eul);
    Tm = eye(4); Tm(1:3,1:3) = R; Tm(1:3,4) = pos;
    set(hT,'Matrix',Tm);

    trail_z = max(X(1:k,3)', 0);
    set(h_trail,'XData',X(1:k,1),'YData',X(1:k,2),'ZData',trail_z);

    axis(ax,[pos(1)-1.5 pos(1)+1.5  pos(2)-1.5 pos(2)+1.5  0  max(2, pos(3)+1)]);
    title(ax,sprintf('Dron Daltonics  t = %.1f s   altura = %.2f m', t(k), pos(3)));
    drawnow limitrate
end
end

function R = rotZYX(eul)
phi=eul(1); th=eul(2); psi=eul(3);
cph=cos(phi); sph=sin(phi); cth=cos(th); sth=sin(th); cps=cos(psi); sps=sin(psi);
R = [ cps*cth,  cps*sth*sph - sps*cph,  cps*sth*cph + sps*sph ;
      sps*cth,  sps*sth*sph + cps*cph,  sps*sth*cph - cps*sph ;
     -sth    ,  cth*sph              ,  cth*cph               ];
end
