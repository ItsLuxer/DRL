%% DALTONICS_parametros.m
% Parametros fisicos del "Dron Daltonics" (dron propio, Equipo A)
% Dron final ensamblado: 530 g, 4 motores 220KV/6T, helices de 16 cm,
% brazos a 15 cm radiales del centro, bateria 3S (11.1V).
%
% Este script se ejecuta automaticamente al iniciar la simulacion
% (esta registrado en el InitFcn del modelo DALTONICS_Sim).
%
% Convencion de ejes (mundo y cuerpo), igual que KE88_parametros.m:
%   x = adelante, y = izquierda, z = arriba
%   Angulos de Euler ZYX: phi (alabeo/roll), theta (cabeceo/pitch), psi (guinada/yaw)
%
% *** IMPORTANTE: orden de motores DISTINTO al gemelo KE88 ***
% Aqui el orden fisico real (pines 1-4 del ESP32, confirmado por el equipo) es:
%   M1 = delantero-izq (+x,+y)   M2 = delantero-der (+x,-y)
%   M3 = trasero-izq   (-x,+y)   M4 = trasero-der   (-x,-y)
% (en KE88 M3/M4 estaban intercambiados). La matriz de mezcladora de abajo
% esta re-derivada para este orden — NO copiar la de KE88 tal cual.

global P    % P queda disponible para Simulink y para DALTONICS_dinamica.m

% ---------- Masa y gravedad (MEDIDO) ----------
P.m  = 0.530;          % kg - dron completo ensamblado (530 g)
P.g  = 9.81;           % m/s^2

% ---------- Geometria (MEDIDO: 15 cm radiales centro->motor) ----------
% Frame en X simetrico: dx = dy = 15cm / sqrt(2), para que la distancia
% radial center->motor de cada uno de los 4 motores sea 15 cm.
P.dx = 0.1061;         % m - distancia del centro a cada motor en x
P.dy = 0.1061;         % m - distancia del centro a cada motor en y

% ---------- Inercias [EST - refinar con banco de inercia] ----------
% Estimadas tratando los 4 conjuntos motor+brazo+ESC+helice (50 g c/u,
% supuesto) como masas puntuales a 15 cm del centro, mas un cuerpo
% central compacto (330 g restantes: ESP32+bateria+frame) modelado como
% disco de ~4 cm de radio. Es una PRIMERA APROXIMACION de orden de
% magnitud, no una medicion.
P.Ixx = 2.38e-3;       % kg*m^2 - alabeo
P.Iyy = 2.38e-3;       % kg*m^2 - cabeceo
P.Izz = 4.76e-3;       % kg*m^2 - guinada

% ---------- Motores y helices [EST - refinar en banco de pruebas] ----------
% Motores brushless 220KV/6T, helices de 16 cm, bateria 3S (11.1V).
% No hay curva de empuje medida: se estima con relacion empuje/peso ~2.1
% (mismo criterio usado en KE88_parametros.m para un dron "normal" bien
% controlable). MEDIR con el banco de pruebas de motores del equipo
% ("Banco de Pruebas para los motores del dron/") y reemplazar este valor
% antes de instalar helices para volar.
P.Tmax  = 2.73;        % N - empuje maximo POR MOTOR (4x2.73=10.9N vs peso 5.20N)
P.tau_m = 0.10;        % s - constante de tiempo del motor+helice [EST, mas lento que KE88 por mayor inercia de la helice]
P.c     = 0.013;       % m - coef. de par de arrastre: par_motor = c * empuje [EST]

% ---------- Arrastre aerodinamico [EST - baja prioridad de refinar] ----------
P.kd_lin = 0.25;       % N*s/m   - arrastre lineal (frena la traslacion)
P.kd_ang = 3e-5;       % N*m*s   - arrastre rotacional

% ---------- Mezcladora (mixer) — RE-DERIVADA para el orden fisico real ----------
% Numeracion de motores (vista superior, x adelante, y izquierda),
% coincide con el cableado real (pines 1-4 del ESP32):
%   M1 = delantero-izq (+x,+y)   M2 = delantero-der (+x,-y)
%   M3 = trasero-izq   (-x,+y)   M4 = trasero-der   (-x,-y)
% [Ftotal; tau_x; tau_y; tau_z] = A * [T1;T2;T3;T4]
% (tau = r x F por motor; pares diagonales para yaw son M1/M4 y M2/M3,
% al reves que en KE88 porque M3 y M4 estan intercambiados)
A = [ 1      1      1      1     ;    % empuje total
      P.dy  -P.dy   P.dy  -P.dy  ;    % alabeo  (izq arriba = +)
     -P.dx  -P.dx   P.dx   P.dx  ;    % cabeceo (adelante = +)
     -P.c    P.c    P.c   -P.c   ];   % guinada (par diagonal M1/M4 vs M2/M3)
P.Mix = inv(A);        % convierte [F;tau] deseados -> empujes por motor

% ---------- Ganancias PID [PUNTO DE PARTIDA PARA SIMULACION, NO DEFINITIVAS] ----------
% Escaladas desde KE88_parametros.m proporcionalmente a la razon de
% inercias (este dron es ~38-43x mas pesado en inercia), solo para no
% arrancar el tuneo desde cero. Control DEBE validar/retunear volando con
% los sliders en DALTONICS_control.slx antes de portar nada a firmware.
P.Kp_ang = 0.077;      % PID de angulo (roll y pitch) -> par (N*m)
P.Ki_ang = 0.019;
P.Kd_ang = 0.023;
P.N      = 40;         % filtro del derivativo
P.Kp_yaw = 0.052;      % PI de velocidad de guinada -> par (N*m)
P.Ki_yaw = 0.022;

% ---------- Estado inicial ----------
% x = [pos(3); vel(3); euler(3); vel_angular_cuerpo(3)]
P.x0 = zeros(12,1);    % en el suelo, en reposo

% ---------- Dato util ----------
% Empuje de hover = m*g = 5.20 N  ->  throttle de hover ~ 47.6 %
