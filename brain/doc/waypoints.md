# Seuil algo navigation waypoints

pour régler le seuil de calcul de passage des waypoint (pour considerer que le véhicule a atteint un waypoint), il y a un param ros : ```rosparam set dst_thresh <n>```

```<n>``` est à 1 par défaut ce qui est sans doute trop faible. Le symptome typique d'un seuil trop bas: le véhicule tourne en rond autour du waypoint sans jamais pouvoir l'atteindre.

# Injection manuelle mission de test

pour tester le véhicule sans la partie cloud (indispo tant que l'on n'a pas réglé le pb de redéploiement), voici une commande de mission en ligne droite avec point de pickup et delivery :

## ATTENTION
x_lat est en ENU et pas en NED et correspond donc au x Est
y_lon est en ENU et pas en NED et correspond donc au y Nord
```bash
rostopic pub /robocar/mission setpoint_leader/robocars_mission "
departure:
  frame: 4
  command: 16
  is_current: true
  autocontinue: true
  param1: 0.0
  param2: 0.0
  param3: 0.0
  param4: 0.0
  x_lat: 4.27
  y_long: 3.66
  z_alt: 0.0
arrival:
  frame: 4
  command: 16
  is_current: true
  autocontinue: true
  param1: 0.0
  param2: 0.0
  param3: 0.0
  param4: 0.0
  x_lat: 2.44
  y_long: 1.22
  z_alt: 0.0
path:
- frame: 4
  command: 16
  is_current: true
  autocontinue: true
  param1: 0.0
  param2: 0.0
  param3: 0.0
  param4: 0.0
  x_lat: 1.22
  y_long: 2.44
  z_alt: 0.0
- frame: 4
  command: 16
  is_current: true
  autocontinue: true
  param1: 0.0
  param2: 0.0
  param3: 0.0
  param4: 0.0
  x_lat: 3.66
  y_long: 2.44
  z_alt: 0.0
- frame: 4
  command: 16
  is_current: true
  autocontinue: true
  param1: 0.0
  param2: 0.0
  param3: 0.0
  param4: 0.0
  x_lat: 4.27
  y_long: 3.66
  z_alt: 0.0
- frame: 4
  command: 16
  is_current: true
  autocontinue: true
  param1: 0.0
  param2: 0.0
  param3: 0.0
  param4: 0.0
  x_lat: 0.61
  y_long: 3.66
  z_alt: 0.0
- frame: 4
  command: 16
  is_current: true
  autocontinue: true
  param1: 0.0
  param2: 0.0
  param3: 0.0
  param4: 0.0
  x_lat: 4.27
  y_long: 1.22
  z_alt: 0.0
- frame: 4
  command: 16
  is_current: true
  autocontinue: true
  param1: 0.0
  param2: 0.0
  param3: 0.0
  param4: 0.0
  x_lat: 0.61
  y_long: 1.22
  z_alt: 0.0
- frame: 4
  command: 16
  is_current: true
  autocontinue: true
  param1: 0.0
  param2: 0.0
  param3: 0.0
  param4: 0.0
  x_lat: 2.44
  y_long: 1.22
  z_alt: 0.0"
```

```bash
rostopic pub /robocar/mission setpoint_leader/robocars_mission "
departure:
  frame: 4
  command: 16
  is_current: true
  autocontinue: true
  param1: 0.0
  param2: 0.0
  param3: 0.0
  param4: 0.0
  x_lat: 0.61
  y_long: 3.66
  z_alt: 0.0
arrival:
  frame: 4
  command: 16
  is_current: true
  autocontinue: true
  param1: 0.0
  param2: 0.0
  param3: 0.0
  param4: 0.0
  x_lat: 4.27
  y_long: 3.66
  z_alt: 0.0
path:
- frame: 4
  command: 16
  is_current: true
  autocontinue: true
  param1: 0.0
  param2: 0.0
  param3: 0.0
  param4: 0.0
  x_lat: 1.22
  y_long: 2.44
  z_alt: 0.0
- frame: 4
  command: 16
  is_current: true
  autocontinue: true
  param1: 0.0
  param2: 0.0
  param3: 0.0
  param4: 0.0
  x_lat: 3.66
  y_long: 2.44
  z_alt: 0.0
- frame: 4
  command: 16
  is_current: true
  autocontinue: true
  param1: 0.0
  param2: 0.0
  param3: 0.0
  param4: 0.0
  x_lat: 4.27
  y_long: 3.66
  z_alt: 0.0
- frame: 4
  command: 16
  is_current: true
  autocontinue: true
  param1: 0.0
  param2: 0.0
  param3: 0.0
  param4: 0.0
  x_lat: 0.61
  y_long: 3.66
  z_alt: 0.0"
```

Cancel ongoing mission :
```bash
rostopic pub /robocar/mission setpoint_leader/robocars_mission "
departure:
  frame: 4
  command: 16
  is_current: true
  autocontinue: true
  param1: 0.0
  param2: 0.0
  param3: 0.0
  param4: 0.0
  x_lat: 0.0
  y_long: 0.0
  z_alt: 0.0
arrival:
  frame: 4
  command: 16
  is_current: true
  autocontinue: true
  param1: 0.0
  param2: 0.0
  param3: 0.0
  param4: 0.0
  x_lat: 0
  y_long: 0
  z_alt: 0.0"
```

  rostopic pub /robocar/mission setpoint_leader/robocars_mission "
  departure:
    frame: 4
    command: 16
    is_current: true
    autocontinue: true
    param1: 0.0
    param2: 0.0
    param3: 0.0
    param4: 0.0
    x_lat: 3.66
    y_long: 2.44
    z_alt: 0.0
  arrival:
    frame: 4
    command: 16
    is_current: true
    autocontinue: true
    param1: 0.0
    param2: 0.0
    param3: 0.0
    param4: 0.0
    x_lat: 2.44
    y_long: 3.66
    z_alt: 0.0
  path:
  - frame: 4
    command: 16
    is_current: true
    autocontinue: true
    param1: 0.0
    param2: 0.0
    param3: 0.0
    param4: 0.0
    x_lat: 1.22
    y_long: 2.44
    z_alt: 0.0
  - frame: 4
    command: 16
    is_current: true
    autocontinue: true
    param1: 0.0
    param2: 0.0
    param3: 0.0
    param4: 0.0
    x_lat: 3.66
    y_long: 2.44
    z_alt: 0.0
  - frame: 4
    command: 16
    is_current: true
    autocontinue: true
    param1: 0.0
    param2: 0.0
    param3: 0.0
    param4: 0.0
    x_lat: 4.27
    y_long: 1.66
    z_alt: 0.0
  - frame: 4
    command: 16
    is_current: true
    autocontinue: true
    param1: 0.0
    param2: 0.0
    param3: 0.0
    param4: 0.0
    x_lat: 2.44
    y_long: 2.44
    z_alt: 0.0
  - frame: 4
    command: 16
    is_current: true
    autocontinue: true
    param1: 0.0
    param2: 0.0
    param3: 0.0
    param4: 0.0
    x_lat: 2.44
    y_long: 3.66
    z_alt: 0.0"

rostopic pub /robocar/mission setpoint_leader/robocars_mission "
departure:
  frame: 4
  command: 16
  is_current: true
  autocontinue: true
  param1: 0.0
  param2: 0.0
  param3: 0.0
  param4: 0.0
  x_lat: 0.61
  y_long: 2.44
  z_alt: 0.0
arrival:
  frame: 4
  command: 16
  is_current: true
  autocontinue: true
  param1: 0.0
  param2: 0.0
  param3: 0.0
  param4: 0.0
  x_lat: 4.27
  y_long: 3.66
  z_alt: 0.0
path:
- frame: 4
  command: 16
  is_current: true
  autocontinue: true
  param1: 0.0
  param2: 0.0
  param3: 0.0
  param4: 0.0
  x_lat: 1.22
  y_long: 1.22
  z_alt: 0.0
- frame: 4
  command: 16
  is_current: true
  autocontinue: true
  param1: 0.0
  param2: 0.0
  param3: 0.0
  param4: 0.0
  x_lat: 3.66
  y_long: 1.11
  z_alt: 0.0
- frame: 4
  command: 16
  is_current: true
  autocontinue: true
  param1: 0.0
  param2: 0.0
  param3: 0.0
  param4: 0.0
  x_lat: 4.27
  y_long: 3.66
  z_alt: 0.0
- frame: 4
  command: 16
  is_current: true
  autocontinue: true
  param1: 0.0
  param2: 0.0
  param3: 0.0
  param4: 0.0
  x_lat: 0.61
  y_long: 3.66
  z_alt: 0.0"

  ---
  
rostopic pub /robocar/mission setpoint_leader/robocars_mission "
departure:
  frame: 4
  command: 16
  is_current: true
  autocontinue: true
  param1: 0.0
  param2: 0.0
  param3: 0.0
  param4: 0.0
  x_lat: 1.0
  y_long: 2.0
  z_alt: 0.0
arrival:
  frame: 4
  command: 16
  is_current: true
  autocontinue: true
  param1: 0.0
  param2: 0.0
  param3: 0.0
  param4: 0.0
  x_lat: 5.0
  y_long: 6.0
  z_alt: 0.0
path:
- frame: 4
  command: 16
  is_current: true
  autocontinue: true
  param1: 0.0
  param2: 0.0
  param3: 0.0
  param4: 0.0
  x_lat: 1.0
  y_long: 2.0
  z_alt: 0.0
- frame: 4
  command: 16
  is_current: true
  autocontinue: true
  param1: 0.0
  param2: 0.0
  param3: 0.0
  param4: 0.0
  x_lat: 3.0
  y_long: 4.0
  z_alt: 0.0
- frame: 4
  command: 16
  is_current: true
  autocontinue: true
  param1: 0.0
  param2: 0.0
  param3: 0.0
  param4: 0.0
  x_lat: 1.0
  y_long: 2.0
  z_alt: 0.0
- frame: 4
  command: 16
  is_current: true
  autocontinue: true
  param1: 0.0
  param2: 0.0
  param3: 0.0
  param4: 0.0
  x_lat: 3.0
  y_long: 4.0
  z_alt: 0.0
- frame: 4
  command: 16
  is_current: true
  autocontinue: true
  param1: 0.0
  param2: 0.0
  param3: 0.0
  param4: 0.0
  x_lat: 1.0
  y_long: 2.0
  z_alt: 0.0
- frame: 4
  command: 16
  is_current: true
  autocontinue: true
  param1: 0.0
  param2: 0.0
  param3: 0.0
  param4: 0.0
  x_lat: 3.0
  y_long: 4.0
  z_alt: 0.0
- frame: 4
  command: 16
  is_current: true
  autocontinue: true
  param1: 0.0
  param2: 0.0
  param3: 0.0
  param4: 0.0
  x_lat: 1.0
  y_long: 2.0
  z_alt: 0.0
- frame: 4
  command: 16
  is_current: true
  autocontinue: true
  param1: 0.0
  param2: 0.0
  param3: 0.0
  param4: 0.0
  x_lat: 3.0
  y_long: 4.0
  z_alt: 0.0
- frame: 4
  command: 16
  is_current: true
  autocontinue: true
  param1: 0.0
  param2: 0.0
  param3: 0.0
  param4: 0.0
  x_lat: 1.0
  y_long: 2.0
  z_alt: 0.0
- frame: 4
  command: 16
  is_current: true
  autocontinue: true
  param1: 0.0
  param2: 0.0
  param3: 0.0
  param4: 0.0
  x_lat: 3.0
  y_long: 4.0
  z_alt: 0.0
- frame: 4
  command: 16
  is_current: true
  autocontinue: true
  param1: 0.0
  param2: 0.0
  param3: 0.0
  param4: 0.0
  x_lat: 1.0
  y_long: 2.0
  z_alt: 0.0
- frame: 4
  command: 16
  is_current: true
  autocontinue: true
  param1: 0.0
  param2: 0.0
  param3: 0.0
  param4: 0.0
  x_lat: 3.0
  y_long: 4.0
  z_alt: 0.0
- frame: 4
  command: 16
  is_current: true
  autocontinue: true
  param1: 0.0
  param2: 0.0
  param3: 0.0
  param4: 0.0
  x_lat: 1.0
  y_long: 2.0
  z_alt: 0.0
- frame: 4
  command: 16
  is_current: true
  autocontinue: true
  param1: 0.0
  param2: 0.0
  param3: 0.0
  param4: 0.0
  x_lat: 5.0
  y_long: 6.0
  z_alt: 0.0"

  rostopic pub /robocar/mission setpoint_leader/robocars_mission "
departure:
  frame: 4
  command: 16
  is_current: true
  autocontinue: true
  param1: 0.0
  param2: 0.0
  param3: 0.0
  param4: 0.0
  x_lat: 0.0
  y_long: 0.0
  z_alt: 0.8
arrival:
  frame: 4
  command: 16
  is_current: true
  autocontinue: true
  param1: 0.0
  param2: 0.0
  param3: 0.0
  param4: 0.0
  x_lat: 0.0
  y_long: 0.0
  z_alt: 0.2
path:
- frame: 4
  command: 16
  is_current: true
  autocontinue: true
  param1: 0.0
  param2: 0.0
  param3: 0.0
  param4: 0.0
  x_lat: 0.0
  y_long: 0.0
  z_alt: 0.8
- frame: 4
  command: 16
  is_current: true
  autocontinue: true
  param1: 0.0
  param2: 0.0
  param3: 0.0
  param4: 0.0
  x_lat: 0.0
  y_long: 0.0
  z_alt: 0.2"

rostopic pub /robocar/mission setpoint_leader/robocars_mission "
departure:
  frame: 4
  command: 16
  is_current: true
  autocontinue: true
  param1: 0.0
  param2: 0.0
  param3: 0.0
  param4: 0.0
  x_lat: 0.0
  y_long: 0.3
  z_alt: 0.5
arrival:
  frame: 4
  command: 16
  is_current: true
  autocontinue: true
  param1: 0.0
  param2: 0.0
  param3: 0.0
  param4: 0.0
  x_lat: 0.0
  y_long: 0.0
  z_alt: 0.1
path:
- frame: 4
  command: 16
  is_current: true
  autocontinue: true
  param1: 0.0
  param2: 0.0
  param3: 0.0
  param4: 0.0
  x_lat: 0.0
  y_long: 0.3
  z_alt: 0.5
- frame: 4
  command: 16
  is_current: true
  autocontinue: true
  param1: 0.0
  param2: 0.0
  param3: 0.0
  param4: 0.0
  x_lat: 0.0
  y_long: 0.0
  z_alt: 0.1"