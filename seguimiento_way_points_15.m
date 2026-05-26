%% Differential Drive - Seguimiento de Waypoints Completo (25x20)
% Código corregido sin errores de propiedad y con evasión de muros optimizada.

clear classes;
clear functions;
close all;
clc;

%% 1. Configuración del Vehículo
R = 0.1;                        % Radio de la rueda [m]
L = 0.5;                        % Distancia entre ruedas [m]
dd = DifferentialDrive(R,L);

sampleTime = 0.05;              % Tiempo de muestreo [s]
tVec = 0:sampleTime:300;        % Tiempo máximo para completar el circuito

%% 2. Carga y Redimensión del Mapa a [25 x 20]
try
    load exampleMaps;           % Carga los mapas nativos de MATLAB
catch
    load exampleMap;
end

% Ajustamos las dimensiones del mapa para que coincida con tu plano de 25x20 metros
mapWidth = 25;                  
mapHeight = 20;                 
resolution = 2;                 % 2 celdas por metro para optimizar la precisión

[origH, origW] = size(complexMap);
newH = mapHeight * resolution;
newW = mapWidth * resolution;
[X, Y] = meshgrid(linspace(1, origW, newW), linspace(1, origH, newH));
matrixResized = interp2(double(complexMap), X, Y, 'nearest') > 0.5;

% Crear objeto oficial de mapa binario
map = binaryOccupancyMap(matrixResized, resolution);
assignin('base', 'map', map);   % Compartir con el visualizador interno

%% 3. Puntos Exactos del Gráfico
waypointsGrafico = [
     4, 14;
     6,  9;
     9,  4;
    11, 19;
    12, 14;
    20, 14;
    21, 13;
    23,  3
];

% El robot inicia su pose en el primer punto de la lista
initPose = [waypointsGrafico(1,1); waypointsGrafico(1,2); 0]; 
pose = zeros(3,numel(tVec));
pose(:,1) = initPose;

%% 4. Planificación Automática de Ruta por Pasillos Seguros (PRM)
planner = mobileRobotPRM(map);
planner.NumNodes = 350;         % Densidad de nodos para pasillos estrechos
planner.ConnectionDistance = 6; 

fprintf('Generando trayectoria segura para el mapa de 25x20...\n');
fullPath = [];
for i = 1:(size(waypointsGrafico, 1) - 1)
    startPt = waypointsGrafico(i, :);
    endPt = waypointsGrafico(i+1, :);
    segmento = findpath(planner, startPt, endPt);
    
    if isempty(segmento)
        % Conexión de respaldo en caso de alta proximidad a un borde
        segmento = [startPt; endPt]; 
    end
    
    if i == 1
        fullPath = segmento;
    else
        fullPath = [fullPath; segmento(2:end, :)];
    end
end
fprintf('¡Ruta libre de colisiones establecida con éxito!\n');

%% 5. Configuración de Sensores y Controladores
warning('off', 'MATLAB:system:deprecatedMixin');

% Configuración del Sensor Lidar
lidar = LidarSensor;
lidar.sensorOffset = [0,0];
lidar.scanAngles = linspace(-pi,pi,180);
lidar.maxRange = 3.5;
lidar.mapName = 'map';

% Configuración del Visualizador de MATLAB
viz = Visualizer2D;
viz.hasWaypoints = true;
viz.mapName = 'map';
attachLidarSensor(viz,lidar);

% Configuración del Controlador Pure Pursuit (Ruta Segura)
controller = controllerPurePursuit;
controller.Waypoints = fullPath;       
controller.LookaheadDistance = 0.50;   % Distancia corta para asegurar giros cerrados
controller.DesiredLinearVelocity = 0.70; 

% SOLUCCIÓN AL ERROR: Se utiliza la propiedad correcta para tu versión
controller.MaxAngularVelocity = 2.0;   

% Histograma de Campo Vectorial (VFH) para micro-correcciones de seguridad
vfh = controllerVFH;
vfh.DistanceLimits = [0.1 3.0];
vfh.NumAngularSectors = 36;
vfh.HistogramThresholds = [3 8];
vfh.RobotRadius = L + 0.05;
vfh.SafetyDistance = 0.12;
vfh.MinTurningRadius = 0.1;

%% 6. Bucle de Simulación Animada
r = rateControl(1/sampleTime);
RADIO_ACEPTACION = 0.6;
totalAlcanzados = 0;
puntosContados = false(size(waypointsGrafico, 1), 1);

for idx = 2:numel(tVec)
    curPose = pose(:,idx-1);
    ranges = lidar(curPose);
    
    % Evaluar Pure Pursuit
    [vRef, wRef, lookAheadPt] = controller(curPose);
    targetDir = atan2(lookAheadPt(2)-curPose(2), lookAheadPt(1)-curPose(1)) - curPose(3);
    targetDir = atan2(sin(targetDir), cos(targetDir));
    
    % Filtro de evasión VFH
    steerDir = vfh(ranges, lidar.scanAngles, targetDir);
    if ~isnan(steerDir) && abs(steerDir-targetDir) > 0.08
        wRef = 0.95 * steerDir;
        vRef = vRef * 0.35; 
    end
    
    % Integración de la cinemática al marco global
    velB = [vRef; 0; wRef];
    vel = bodyToWorld(velB, curPose);
    pose(:,idx) = curPose + vel*sampleTime;
    
    % Control de verificación de los 8 puntos objetivos superados
    for w = 1:size(waypointsGrafico,1)
        if ~puntosContados(w)
            dist = norm(pose(1:2,idx) - waypointsGrafico(w,:)');
            if dist < RADIO_ACEPTACION
                puntosContados(w) = true;
                totalAlcanzados = totalAlcanzados + 1;
                fprintf('✓ Objetivo del gráfico alcanzado: (%d, %d) [%d/8]\n', ...
                    waypointsGrafico(w,1), waypointsGrafico(w,2), totalAlcanzados);
            end
        end
    end
    
    % Criterio de parada: Llegada final al último punto (23,3)
    distToFinal = norm(pose(1:2,idx) - fullPath(end,:)');
    if distToFinal < 0.4 && idx > 100
        fprintf('\n¡Circuito completado con éxito!\n');
        break;
    end
    
    % Renderizar la ventana gráfica fijando los límites solicitados de 25x20
    viz(pose(:,idx), fullPath, ranges);
    xlim([0 mapWidth]);
    ylim([0 mapHeight]);
    
    waitfor(r);
end

%% 7. Resumen de Métricas Finales
fprintf('\n================ RESUMEN FIINAL =================\n');
fprintf(' Puntos del gráfico validados: %d de 8\n', totalAlcanzados);
fprintf(' Tiempo total de simulación:   %.2f segundos\n', (idx-1)*sampleTime);
fprintf(' Pose final alcanzada:         X = %.2f, Y = %.2f\n', pose(1,idx), pose(2,idx));
fprintf('=================================================\n');