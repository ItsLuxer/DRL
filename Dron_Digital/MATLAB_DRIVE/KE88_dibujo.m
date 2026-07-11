function y = KE88_dibujo(x)


persistent ax hT tr X t0
y = 0; x = x(:);

% ---------- parametros de ajuste del modelo STL ----------
% Si el modelo se ve "de cabeza" y/o el frente apunta al costado,
% ajusta estos 3 angulos (en grados) hasta que se vea bien.
% Se aplican en orden: primero ROLL, luego PITCH, luego YAW.
ESCALA_STL      = 0.0017;
ROLL_OFFSET_DEG  = 180;   % gira el modelo sobre su eje "adelante" (x) -> corrige que vuele de espaldas
PITCH_OFFSET_DEG = 0;     % gira el modelo sobre su eje "izquierda" (y)
YAW_OFFSET_DEG    = 90;   % gira el modelo sobre su eje vertical (z)  -> ya corregia el frente

if isempty(ax) || ~isgraphics(ax)
    f = figure(99); clf(f); set(f,'Name','KE88 en vivo (STL)','Color','w');
    ax = axes('Parent',f); hold(ax,'on'); grid(ax,'on'); view(ax,45,20);
    axis(ax,'equal');
    [gx,gy] = meshgrid(-3:1:3);
    surf(ax,gx,gy,0*gx,'FaceColor',[0.9 0.95 0.9],'EdgeColor',[0.8 0.85 0.8]);

    % --- cargar el modelo STL (una sola vez) ---
    ruta_stl = fullfile(fileparts(mfilename('fullpath')), 'Dron_Equipo_A.STL');
    TR = stlread(ruta_stl);
    V  = TR.Points;                 % vertices en coordenadas nativas del STL
    F  = TR.ConnectivityList;

    % centrar el modelo en el centro de su caja envolvente
    c = (max(V,[],1) + min(V,[],1)) / 2;
    V = V - c;

    % En Dron_Equipo_A.STL el eje vertical del dron YA es la Z nativa
    % (a diferencia de DronNaya.STL, aqui no hace falta reordenar ejes).
    Vb = V;   % body_x=X_nativa, body_y=Y_nativa, body_z=Z_nativa

    % rotacion de ajuste roll-pitch-yaw (ver parametros arriba)
    cr=cosd(ROLL_OFFSET_DEG); sr=sind(ROLL_OFFSET_DEG);
    cp=cosd(PITCH_OFFSET_DEG); sp=sind(PITCH_OFFSET_DEG);
    cy=cosd(YAW_OFFSET_DEG);  sy=sind(YAW_OFFSET_DEG);
    Rx_ = [1 0 0; 0 cr -sr; 0 sr cr];
    Ry_ = [cp 0 sp; 0 1 0; -sp 0 cp];
    Rz_ = [cy -sy 0; sy cy 0; 0 0 1];
    R_ajuste = Rz_ * Ry_ * Rx_;
    Vb = (R_ajuste * Vb')';

    % --- escalar el modelo al tamano real del dron ---
    global P
    if isempty(P), KE88_parametros; end
    diag_real = 2*sqrt(P.dx^2 + P.dy^2);        % separacion diagonal motor-motor [m]
    span      = Vb(:,[1 2]);
    diag_stl  = norm(max(span,[],1) - min(span,[],1));
    if isempty(ESCALA_STL)
        escala = diag_real / diag_stl;
    else
        escala = ESCALA_STL;
    end
    Vb = Vb * escala;

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

% Clamping de estela
set(tr,'XData',X(1,:),'YData',X(2,:),'ZData',max(X(3,:),0));

% Camara: Z siempre desde 0
axis(ax,[pos(1)-1.5 pos(1)+1.5  pos(2)-1.5 pos(2)+1.5  0  max(2, pos(3)+1)]);
title(ax,sprintf('KE88 en vivo (STL)  |  altura z = %.2f m', pos(3)));
drawnow limitrate
end
