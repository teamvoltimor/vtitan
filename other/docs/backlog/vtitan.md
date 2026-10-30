# vtitan backlog

La cola de pendientes del repositorio. Los items salen de las notas de sesión
(working notes en `other/docs/internal`, no versionadas) y de los ADR.

Esto es una cola, no un informe de estado (ADR 0100). Un item dice qué falta y
de dónde viene, nunca qué "está pasando" hoy; lo que se puede medir se mide al
preguntarlo, y un número medido vive en su ADR, no aquí. Un item cerrado se
mueve a `done/YYYY-MM.md` con su fecha.

## doing

- Nada en curso.

## next

- [ ] BAY-DIR-1, implementar la regla de sentido de bahía por LIDAR. Mediana de
  sector de ±15° en lugar de un rayo único, y hasta 5 barridos de reintento
  durante los primeros 0,5 s con el coche parado antes de arrancar. Medido sobre
  bags: 18 de 18, 0 errores, 0,5 s en lugar de 8,6 a 14,6 s (hoy: 15 aciertos, 0
  errores, 3 sin respuesta). Absorbe dos defectos distintos: un rayo perdido que
  `decode_scan` rellena con 12 m, y el LIDAR arrancando (el arco frontal entero
  leyó 12,000 durante 4 barridos y a los 0,51 s pasó a 0,045). El reintento solo
  es seguro con el coche parado: reintentar siempre dispararía la regla a mitad
  del creep en una esquina. Es complementario a BAY-EXIT-1, no lo sustituye.
  Origen: sesión obstacles-simulation-49 (2026-09-20). #P1 #nav

- [ ] BAY-EXIT-1, el buffer de ingesta durante la salida de bahía está construido
  y apagado, y lo que bloquea encenderlo es el instrumento que mide su basura:
  reproducir el buffer sobre las bags y contar cuántas trazas publica el mapa,
  cuántas caen en el retículo legal de posiciones y si el pilar bueno sale con
  el color correcto al traspaso. Sin ese número la bandera no se enciende. En
  clockwise la evidencia ya es fuerte (820 observaciones sobre el pilar correcto,
  9 de 11 rondas); en counterclockwise la corrección de rumbo sube del 5,5 % al
  44,2 % y sigue por debajo del 71 % natural de cw. Ninguna de las soluciones
  arregla la geometría: en 6 de 23 rondas el cruce no cabe. Origen: sesión
  obstacles-simulation-49 (2026-09-20). #P1 #nav

- [ ] RVIZ-1, el coche no aparece en RViz ("No TF data", "Frame [map] does not
  exist") y la prueba decisiva ya descarta el repo: un `ros2 topic pub` y un
  `ros2 topic echo` entre dos procesos independientes tampoco se ven, así que es
  descubrimiento DDS a nivel de máquina. Sospecha: Windows Firewall bloqueando
  los `python.exe` de cada entorno de pixi (cada entorno tiene su ruta, una regla
  no cubre la otra). Siguiente paso: `Get-NetFirewallRule`/`Get-NetFirewallProfile`
  o desactivar el firewall como prueba única, y solo después una regla permanente.
  Ya resueltos en la misma sesión: la carrera de `solve-group` de pixi (preinstalar
  ambos entornos), `VTITAN_HARDWARE_PROFILE` fuera de Task, y `RMW_IMPLEMENTATION`
  sin fijar en viz (ya fijado en `platform.yml`). Origen: sesión rviz-fix. #P1 #tooling

- [ ] FRONTEND-1, la Fase 1 del dashboard (los dos [BLOCKER], sin decisiones):
  en `telemetry_bridge_node.py` renombrar las tres suscripciones muertas
  (`/state_machine/state` a `/robot_state`, `/cmd_vel` a `/ackermann_cmd` como
  `AckermannDriveStamped` y no `Twist`, `/hailo/detections` a `/vision/detections`
  como String/JSON y no `Detection2DArray`) e indexar `/joint_states` por nombre
  (`name.index("steering")`) con las unidades correctas, porque hoy el steering
  lleva los radianes acumulados de la rueda motriz y la posición del encoder es
  siempre 0; y normalizar el bbox a [0,1] de punta a punta como `vision.go:17`.
  Después, sin decisión: emitir `TopicsSnapshot` por el WebSocket existente
  (`ws.go:63` solo manda `RobotSnapshot`, y `source.ts:69` pide topics una vez,
  así que todo queda STALE a los 2 s) y añadir los topics que faltan. Origen:
  sesión frontend-2. #P2 #frontend

- [ ] MASTER-RED-1, `test_shipped_default_skips_the_router` fija un default que ya
  no existe: afirma que el valor enviado NO tickea el router durante una maniobra
  y la sesión paralela lo cambió a `true` en `1fae52fc` sin actualizar el test.
  Master está rojo por un test que solo tiene que decir lo contrario. Origen:
  sesión obstacles-simulation-49 (2026-09-20). #P1 #test

## later

- [ ] GO-PARITY-1, paridad Python y Go. Abierto: el transitorio de la primera
  vuelta en los escenarios blind 0000 a 0002 (Go +301/+145/+127 pasos, laps 2 y 3
  ya coinciden, bootstrap termina hacia el paso 21), la divergencia de
  `scenario_0003` blind (Go más rápido en todas las vueltas), y extender
  `TestNativeRunner_ParityVsPython` a Obstacles (hoy es solo Open y su tolerancia
  de 0,2 s de `sim_time` no vale para Obstacles). Origen: sesión go-port-14.
  #P2 #nav

- [ ] CONFIG-1, `DefaultConfig` no coincide con el TOML enviado
  (`obstacles_contact_dist` 0,04 contra 0,05; `k_turn_max_s` 1,08, 22 frames,
  contra 12), que es exactamente el `TestConfigFor_LoadsRealNavigationTuningFiles`
  que falla de antes. Decidir si se arregla el literal o el TOML. Origen: sesión
  go-port-14 y unified-docs. #P2 #config

- [ ] GO-FLAGS-1, banderas del navegador apagadas que no están portadas a Go (sin
  delta enviado, pero hacen falta para barrer brazos): `TICK_ROUTER_DURING_MANEUVER`
  e `_ingest_sign_observations`, `SIGN_DEFORM_SENSE_GUARD` y
  `_bearing_agrees_with_path`, `SIGN_LANE_DEFORM_FALLBACK_M` y
  `SignRouter.committed_pass_side_offset`, `SIDE_CORRECTION_BLINDS`; la máscara de
  escape por bearing (`RangesBeyondChassis` existe sin consumidor porque falta el
  cluster search). Medio: parking `DERIVE_LOT_FROM_IN_BAY_START` (la ruta de
  hardware no monta ningún `ParkController`). Menor: amortiguación de
  `_corridor_flip_streak`, `propose` del LIDAR sin llamar, vision sidecar/hailo.
  Origen: sesión go-port-14. #P3 #nav

- [ ] SIM-1, lo que dejó abierto la simulación de Obstacles: el emparejamiento
  barrido-yaw a través de un parón (causa real de la U), el suelo de radio mínimo
  de giro en `geometry.py:68` (hoy no existe), `corner_mps` sin poner y la cautela
  de curva cayendo en `slow_mps`, `in_robot_frame` como campo muerto, y el
  localizador de Go dos guardias por detrás del de Python. Origen: sesión
  obstacles-simulation-49 (2026-09-20). #P2 #nav

- [ ] DIAG-AUDIT-1, lo que dejó abierto la auditoría de arquitectura de Python
  (sesión diag-audit-3): 7 nodos ROS 2 construyen `NodeConfig()` en el import (y
  `shared/config` carga el TOML en el import, y `configure_json_logging()`),
  parametrizar `test_collision_avoidance_controller.py` (66 tests),
  `test_sign_discovery.py` y `test_navigation_node.py`, consolidar los
  detection-builders, recortar docstrings por campo, sacar excepciones base para
  `logger`/`state_machine`/`teleop`, `ABC` a `Protocol` en `ports.py`/`detector.py`,
  y los errores de mypy de `sign_router`/`simulator.py`. `diag_yaw_flip_replay.py`
  importa limpio pero trata `NavigatorDebugSnapshot` como dict y no funciona. El
  baseline de ruff de `src/python` ronda los 3300 avisos y los scripts no están en
  el gate. Origen: sesión diag-audit-3. #P3 #quality

- [ ] DIAG-SCRIPTS-1, los tres diagnósticos que la sesión adr-4 dejó
  deliberadamente para otra tanda: `diag_sign_sweep.py` (216 impactos de marker),
  `diag_sign_slot_ab.py` y `diag_bag_escape_counter.py`. Y en la sesión
  diag-audit-3 quedó sin decidir si `common/binning.py`, `common/formats.py` y
  `common/motor_hold.py` se adoptan o se borran (el mapa del README de `common/`
  los agrupa aunque la auditoría los llamó casi huérfanos). #P3 #quality

- [ ] CONFIG-2, lo que dejó abierto la unificación de config: los tiers `speed`
  generados son `float | None`, así que los consumidores leen opcionales (se
  resuelve con `assert`/`cast` o con tiers base no opcionales en el schema), la
  decisión de renombrar `profile` a algo como `config/load`, los tres tipos Go a
  mano (`RobotConfig`, `ClearanceConfig`, `EncoderConfig`), los hardware de Python
  que aún no consumen su DTO (hailo, camera, imu, button, display, mcp2221,
  motors), `constants/track.py` como alias fino, `generated/_models.py` sin
  revisar y los `type =` preexistentes (`natsConn`, `LidarScan`, `Direction`,
  `BelievedStart`). #P3 #config

- [ ] FRONTEND-2, las cuatro decisiones de §14 que bloquean el resto del
  dashboard: de dónde sale `/odom` (la opción barata es renderizar el LIDAR en
  marco robot y quitar la escena en marco mundo), si el control remoto lleva
  e-stop, dónde vive el offset de yaw del LIDAR (lado bridge, un solo sitio) y la
  topología de despliegue (servir el bundle estático desde el backend Go, un
  puerto y sin proxy ni CORS). §14.4 a §14.6 son alcance opcional. Origen: sesión
  frontend-2. #P2 #frontend

- [ ] ADR-GAPS-1, los ADR que faltan por escribir o reconciliar: el contrato de
  serialización de markers del visualizador en vivo (un `int` JSON en un campo
  `float64` de `geometry_msgs` serializa como subnormal y mueve los markers; el
  renderer `TRIANGLE_LIST` tiene su propio límite), la contradicción de `bag_io`
  contra el ADR 0084 sobre cuánto subestima la tasa de yaw (¿la mitad o 4 a 6x?),
  y el crash de orden de `set_parking_corridor`, que no tiene ADR dueño. Además,
  la duplicación de los 45 ADR superseded ya está obsoleta. Origen:
  sesión adr-4. #P2 #docs

- [ ] HUGO-1, lo que queda del sitio de docs: JS inline sin SRI en `baseof.html`,
  sin `og:image` ni JSON-LD, los avisos de deprecación de Hugo (`languageCode`,
  `.Site.LanguageCode`, `.Language.LanguageDirection`, `.Site.Sites`) y el pin de
  CI en 0.156.0 contra 0.163.0 local; y confirmar el `projectName` de Cloudflare
  (`klevor-docs` en `hugo-docs.yml:60`) antes de renombrarlo, porque renombrarlo a
  ciegas rompe el deploy. Ya arreglado: submódulo del tema, `copy_docs.go` y el
  enlace de la imagen. Origen: sesión hugo-review. #P3 #docs

- [ ] DATASHEETS-1, crear el índice de `other/docs/reference/datasheets/` con lo
  presente (los dos PDF de Slamtec) y lo pendiente, cada uno con su ADR asociado, y
  conseguir las hojas que faltan: BNO085 y esquemático GY, Hailo-8L y AI HAT+,
  Pi 5, Camera Module 3 Wide, BTS7960, HPS-3527SG, REV HD Hex, Mini-560 Pro,
  KL89576, las dos versiones de la Ovonic 3S, y las de descarte (SSD1306, L298N,
  XLC4016) como prueba de por qué se retiraron. Nomenclatura propuesta:
  `<fabricante>-<componente>-<doc>-vX.Y.pdf`. Origen: sesión more-datasheets.
  #P3 #docs

- [ ] RELEASE-1, higiene del repositorio y del README: notas de versión de v1.0,
  v1.1 y v1.2, `README.en.md`, badge de CI, logos triplicados (unos 2,9 MB),
  `project-diagram.png` en 0 bytes, el README de `other/apps/gazebo` que menciona
  Humble y Gazebo 11 cuando el proyecto va por Kilted, y las 329 vulnerabilidades
  de Dependabot en la rama por defecto (13 críticas, 148 altas). Añadido de la
  misma sesión: maquetar la "Lista de componentes" a dos columnas, glosar los
  anglicismos sueltos (wheelbase, deadband, throughput, lookahead, feedforward) y
  verificar en GitHub que `align` centra las `<table>` y que las dos tablas LaTeX
  siguen renderizando (en local no se puede comprobar). Origen: sesión github-7.
  #P3 #repo

- [ ] SCHEMATICS-1, planos y piezas que quedaron sin pareja: `pinon-20-dientes-2.webp`
  (recto, sin casar con ninguna pieza) y `pinon-de-correa-33-dientes.webp`, que
  debería alinearse a `pinon-33-dientes-correa-dentada` si el dueño lo quiere.
  Origen: sesión github-7. #P3 #docs

- [ ] DOCS-DRIFT-1, `src/python/docs/robot-physical-constants.md` apunta a
  `src/shared/config/robot.toml` (la ruta real es `src/config/robot.toml`) y su
  tabla canónica no incluye la altura del LIDAR que el ADR 0014 fijó en 0,08 m.
  Origen: sesión github-7. #P3 #docs

- [ ] PI-CLOCK-1, el reloj del Pi 5 va dos días atrasado y las rondas nuevas se
  llaman igual que las viejas (`run_20260915_*`), lo que hace ambiguo cualquier
  análisis por nombre. Arreglar la sincronización horaria cuando el enlace vuelva.
  Origen: sesión obstacles-simulation-49 (2026-09-20). #P2 #hardware

- [ ] WRO-HUECOS-1, los tres HUECO del README que solo el equipo puede rellenar:
  protección eléctrica (fusible o limitador en tracción, unos 20 A de pico, corte
  por bajo voltaje de la LiPo 3S, carga y almacenamiento), manifiesto de impresión
  por pieza (material, capa, relleno, soportes, orientación) y orden de armado con
  tornillería y par de apriete. Origen: sesión github-7. #P3 #docs

## someday

- [ ] DOCS-ARCH-1, completar la adopción de docs-architect: frontmatter en los 101
  ADR, carpetas `guides/` y `methodology/`, `templates/` y el linter
  `docs_lint.py` en CI. El ADR 0100 solo adopta el backlog; mover ficheros de
  `adr/` o reescribir sus cabeceras necesita su propia aprobación. Origen: ADR 0100.
  #P3 #docs

- [ ] ROUNDS-1, cuando lleguen las bags de las rondas nuevas: comprobar si el
  re-anclaje mató el "sale y se da la vuelta" y si la puerta de permanencia actuó
  en algún encalle (hay que dejar el robot atascado 15 a 20 s para poder
  evaluarla). Origen: sesión obstacles-simulation-49 (2026-09-20). #P3 #analysis
