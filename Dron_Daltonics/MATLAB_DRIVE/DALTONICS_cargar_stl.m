function [F, V] = DALTONICS_cargar_stl()
% DALTONICS_cargar_stl - carga, centra, ajusta y escala el STL del Dron
% Daltonics. Usada por DALTONICS_animacion.m y DALTONICS_dibujo.m para no
% duplicar la logica de carga entre ambas.
%
% *** El STL todavia no existe en este repo ***
% Hay que exportarlo desde SolidWorks:
%   "Dron Semana 3 Mejorado/Ensamblaje Dron semana 3.SLDASM"
%   -> File > Save As > STL
% y guardarlo EXACTAMENTE como:
%   Dron_Daltonics/MATLAB_DRIVE/Dron_Daltonics.STL

% ---------- parametros de ajuste del modelo STL ----------
% Si el modelo se ve "de cabeza" y/o el frente apunta al costado, ajusta
% estos 3 angulos (en grados) una vez que el STL exista y se pueda ver.
% Se aplican en orden: primero ROLL, luego PITCH, luego YAW.
ROLL_OFFSET_DEG  = 0;
PITCH_OFFSET_DEG = 0;
YAW_OFFSET_DEG   = 0;

% Sin medicion previa del STL, se usa auto-escala a partir de la
% geometria real (dx,dy) en vez de un numero fijo (que en KE88_animacion.m
% se calibro a ojo una vez que el STL ya se podia ver).
ESCALA_STL = [];

ruta_stl = fullfile(fileparts(mfilename('fullpath')), 'Dron_Daltonics.STL');
if ~isfile(ruta_stl)
    error('DALTONICS:stlFaltante', sprintf(['No se encontro Dron_Daltonics.STL.\n', ...
           'Exporta el ensamblaje de SolidWorks a STL:\n', ...
           '  "Dron Semana 3 Mejorado/Ensamblaje Dron semana 3.SLDASM" -> File > Save As > STL\n', ...
           'y guardalo como:\n', ...
           '  Dron_Daltonics/MATLAB_DRIVE/Dron_Daltonics.STL']));
end

TR = stlread(ruta_stl);
V  = TR.Points;
F  = TR.ConnectivityList;

c = (max(V,[],1) + min(V,[],1)) / 2;
V = V - c;

cr=cosd(ROLL_OFFSET_DEG); sr=sind(ROLL_OFFSET_DEG);
cp=cosd(PITCH_OFFSET_DEG); sp=sind(PITCH_OFFSET_DEG);
cy=cosd(YAW_OFFSET_DEG);  sy=sind(YAW_OFFSET_DEG);
Rx_ = [1 0 0; 0 cr -sr; 0 sr cr];
Ry_ = [cp 0 sp; 0 1 0; -sp 0 cp];
Rz_ = [cy -sy 0; sy cy 0; 0 0 1];
R_ajuste = Rz_ * Ry_ * Rx_;
V = (R_ajuste * V')';

global P
if isempty(P), DALTONICS_parametros; end
diag_real = 2*sqrt(P.dx^2 + P.dy^2);
span      = V(:,[1 2]);
diag_stl  = norm(max(span,[],1) - min(span,[],1));
if isempty(ESCALA_STL)
    escala = diag_real / diag_stl;
else
    escala = ESCALA_STL;
end
V = V * escala;
end
