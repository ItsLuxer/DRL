function xdot = DALTONICS_dinamica(u)


global P
if isempty(P)
    DALTONICS_parametros;
end

% ---------- desempacar estado y empujes ----------
x   = u(1:12);
T   = u(13:16);
pos = x(1:3);   v   = x(4:6);
eul = x(7:9);   om  = x(10:12);
phi = eul(1);   th  = eul(2);   psi = eul(3);

% ---------- matriz de rotacion cuerpo->mundo (ZYX, z arriba) ----------
cph=cos(phi); sph=sin(phi);
cth=cos(th);  sth=sin(th);
cps=cos(psi); sps=sin(psi);
R = [ cps*cth,  cps*sth*sph - sps*cph,  cps*sth*cph + sps*sph ;
      sps*cth,  sps*sth*sph + cps*cph,  sps*sth*cph - cps*sph ;
     -sth    ,  cth*sph              ,  cth*cph               ];

% ---------- fuerzas aereas (marco mundo) ----------
Ftot = sum(T);
acc  = [0;0;-P.g] + R*[0;0;Ftot]/P.m - (P.kd_lin/P.m)*v;

% ---------- pares (marco cuerpo) ----------
% *** Orden de motores del Dron Daltonics (NO es el mismo que KE88): ***
%   M1 = delantero-izq (+x,+y)   M2 = delantero-der (+x,-y)
%   M3 = trasero-izq   (-x,+y)   M4 = trasero-der   (-x,-y)
% Estas formulas deben coincidir con la matriz A de DALTONICS_parametros.m.
tau = [ P.dy*( T(1) - T(2) + T(3) - T(4));
       -P.dx*( T(1) + T(2) - T(3) - T(4));
        P.c *(-T(1) + T(2) + T(3) - T(4))]...
      - P.kd_ang*om;

I     = diag([P.Ixx P.Iyy P.Izz]);
omdot = I \ (tau - cross(om, I*om));

% ---------- cinematica de Euler ----------
W = [1, sph*tan(th), cph*tan(th);
     0, cph        , -sph       ;
     0, sph/cth    , cph/cth    ];
euldot = W*om;

% ---------- ensamblar derivada libre ----------
xdot = [v; acc; euldot; omdot];

% ==========================================================
%  RESTRICCION DE SUELO  (igual que KE88_dinamica.m)
%  Modelo resorte-amortiguador:
%    F_suelo = -Kp * penetracion  -  Kd * velocidad_vertical
%  Solo actua cuando z < 0 (penetra el suelo).
% ==========================================================
z  = pos(3);
vz = v(3);

if z < 0
    penetracion = -z;   % > 0

    Kp_suelo = 500;     % N/m  - rigidez (evita penetracion profunda)
    Kd_suelo = 60;      % N*s/m - amortiguamiento de impacto

    F_reaccion = Kp_suelo * penetracion - Kd_suelo * vz;
    F_reaccion = max(F_reaccion, 0);   % nunca jala hacia abajo

    a_reaccion = F_reaccion / P.m;
    xdot(6) = xdot(6) + a_reaccion;   % aceleracion z

    if vz <= 0.05
        xdot(4:5) = xdot(4:5) - 8.0 * v(1:2);    % frena traslacion XY
        xdot(10:12) = xdot(10:12) - 12.0 * om;    % frena rotacion
        xdot(7:8)   = xdot(7:8) - 3.0 * eul(1:2); % tiende a nivelarse
    end
end

% ---------- Capa de seguridad adicional ----------
if pos(3) <= 0 && v(3) < 0
    xdot(3) = max(xdot(3), 0);   % vel z no puede bajar mas
end

end
