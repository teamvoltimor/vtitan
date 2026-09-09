# Configuración TOML de navegación (`robot` y `robot-go`)

Este documento explica **dónde reside la configuración**, **cómo se resuelve** y **qué hace cada
variable relevante** en las dos implementaciones de navegación: `platform/robot` (Python/ROS 2) y
`platform/robot-go` (Go).

El alcance es deliberado: se documenta lo que **modifica el comportamiento en pista** en el Open
Challenge y en el Obstacle Challenge. Se omiten los ficheros puramente de hardware y de
controladores de dispositivo (pines GPIO, direcciones I2C, rangos de PWM del servo, UART de la IMU,
parámetros del panel OLED, del botón y del grabador), que residen en
`platform/robot/config/hardware/` y solo describen cómo se comunica el sistema con una pieza física.

---

## 1. Los cuatro árboles de configuración

| Árbol | Ruta | Contenido |
|---|---|---|
| Base de navegación | `platform/shared/config/navigation/<tema>/<grupo>.toml` | El ajuste que gobierna la conducción. Un fichero por grupo de parámetros. |
| Hechos físicos | `platform/shared/config/robot.toml`, `track.toml`, `competition_specs.toml` | Geometría del chasis y de los sensores, geometría de la pista y reglas WRO. |
| Perfiles de hardware | `platform/shared/config/profiles/<nombre>/...` | Únicamente las claves que difieren para una pieza concreta (servo, motor). Se seleccionan mediante `VTITAN_HARDWARE_PROFILE`. |
| Capas por reto | `platform/shared/config/navigation-challenges/<open\|obstacles>/<tema>/<grupo>.toml` | Reajuste específico de un reto. Actualmente solo contiene los `README.md`: está vacío de forma intencionada. |

Ambas implementaciones leen **los mismos ficheros**. Go no dispone de un árbol propio:
`internal/config/profile` declara las mismas rutas como constantes (`DefaultClearanceTOMLPath`, etc.)
y las carga con viper desde `--config-root`.

### Orden de precedencia

1. Valores por defecto del modelo (`Field(default=...)` en Pydantic; literales de `DefaultConfig()`
   en Go).
2. El TOML base de `navigation/`.
3. Cada perfil activo de `VTITAN_HARDWARE_PROFILE`, en orden (prevalece el último).
4. La capa del reto (`navigation-challenges/<reto>/`), únicamente en Python.

**Advertencia principal:** si una clave está declarada en el TOML base, modificar el valor por
defecto de Pydantic **no surte efecto alguno**. El TOML prevalece siempre sobre el modelo. La única
forma de que un valor por defecto del modelo llegue al robot es que la clave **no exista** en el
TOML (es el caso de `simulation.MIN_TURN_RADIUS_M = 0.29`, ausente en `simulation.toml`).

**No existe sobrescritura por variable de entorno para el ajuste de navegación.**
`VTITAN_HARDWARE_PROFILE` selecciona perfiles, no valores. Para modificar un número es necesario
editar el TOML o aplicar un perfil.

### Claves con prefijo de reto

Varios grupos admiten una clave por reto dentro del mismo fichero. Si la variante es `None` o está
ausente, se aplica el valor base:

- `obstacles_contact_dist` en `motion/clearance.toml`
- `obstacles_yaw_gain_compensation` y `open_lookahead_long` en `motion/pursuit.toml`
- `open_*_mps` y `obstacles_*_mps` en `motion/speed.toml` (habitualmente en el perfil del motor)

---

## 2. Ficheros y variables clave

### 2.1 `motion/speed.toml` - la escalera de velocidad

El fichero base es conservador; **la escalera efectiva se define en el perfil del motor**
(`profiles/rev-hd-hex-motor-6000rpm/motion/speed.toml`), porque el reparto entre escalones depende
de lo que ese tren motriz es capaz de entregar.

| Clave | Base | Perfil 6000 rpm | Función |
|---|---|---|---|
| `min_mps` | 0.0499 | 0.0499 | Umbral de fricción: por debajo de este valor el robot no se desplaza. |
| `creep_mps` | 0.1014 | 0.1521 | Zona de contacto, límite inferior del limitador de rumbo, impulso de escape. |
| `slow_mps` | 0.117 | 0.22 | Obstáculo próximo (0.10-0.25 m) y límite superior cuando el riesgo no es `SAFE`. |
| `medium_mps` | 0.1326 | 0.32 | Holgura moderada y seguimiento a ciegas antes de fijar la dirección. |
| `fast_mps` | 0.156 | 0.42 | Pista despejada (>0.50 m). |
| `max_mps` | 0.156 | 0.50 | Límite superior estricto aplicado sobre cualquier escalón. |
| `open_slow/medium/fast/max_mps` | - | 0.26/0.38/0.50/0.55 | Escalera exclusiva del **Open Challenge**. |

Observaciones que previenen errores costosos:

- Elevar `open_fast_mps` sin elevar `open_max_mps` **no produce ningún efecto**: el navegador limita
  cada escalón contra `max_mps()`.
- El rendimiento en Obstacles se degrada de forma monótona con la velocidad (medición: 38/256 a
  0.156 m/s, 16 a 0.50), motivo por el cual conserva la escalera base.
- La escalera de holgura (`clearance.toml`) está expresada en **metros**, no en tiempo: cada
  incremento de velocidad reduce el margen de reacción que esas distancias proporcionan.

### 2.2 `motion/clearance.toml` - zonas de holgura

| Clave | Valor | Función |
|---|---|---|
| `contact_dist` | 0.10 | Peligro inmediato: la velocidad desciende a `creep`. |
| `obstacles_contact_dist` | 0.05 | El mismo umbral, aplicado solo en Obstacles (los pilares obligan a una aproximación mayor). |
| `slow_dist` / `medium_dist` / `fast_dist` | 0.25 / 0.50 / 1.00 | Fronteras de la escalera de velocidad. |
| `path_margin` | 0.10 | Margen adicional sobre la semianchura del chasis que sigue considerándose "en la trayectoria". |
| `forward_path_ahead_of_bumper` | false | Medir la holgura frontal desde el parachoques en lugar de desde el LIDAR. |
| `forward_no_data_is_degraded` | true | Un cono frontal sin retornos se interpreta como degradado, no como despejado. |

### 2.3 `motion/pursuit.toml` - *pure pursuit* (el controlador de trayectoria)

| Clave | Valor | Función |
|---|---|---|
| `lookahead_short` / `lookahead_long` | 0.16 / 0.32 | Distancia de anticipación en curva cerrada y en recta. |
| `open_lookahead_long` | 0.24 | Anticipación en recta, únicamente para Open. |
| `lookahead_transition` + `lookahead_blend_start` | 0.30 / 0.70 | Error lateral en el que se conmuta de anticipación larga a corta, mediante rampa en lugar de escalón. |
| `corner_preview_distance_m` / `corner_turn_threshold_rad` | 0.80 / 0.35 | Activan la anticipación corta **antes** de la curva, dado que el error lateral se manifiesta con retraso. |
| `steer_kp` | 1.2 | Ganancia proporcional de dirección. |
| `max_steering_rate` | 1.2 rad/s | Límite de velocidad angular del servo. **Sin medición**: un servo real de 35 kg se aproxima a 5. |
| `yaw_gain_compensation` | 1.0 | Fracción del giro previsto que el chasis entrega realmente, dividida fuera de la demanda. 1.0 equivale a desactivado (Open). |
| `obstacles_yaw_gain_compensation` | 0.55 | Compensación completa, únicamente en Obstacles. |
| `wall_margin_safety_m` / `min_lookahead_transition_m` | 0.03 / 0.10 | El umbral efectivo es el menor entre `lookahead_transition` y el que permite la distancia real al muro exterior. |

### 2.4 `motion/heading.toml` y `motion/control.toml`

- `crawl = 1.0` rad (~57°): error de rumbo por encima del cual la velocidad desciende al límite
  inferior de `creep`. Se trata de un **único umbral binario**, no de una escalera: los escalones
  intermedios se eliminaron tras comprobar que suponían un 33% del tiempo de vuelta.
- `control_hz = 20.0`: frecuencia del bucle de control, tanto en el robot como en simulación. La
  práctica totalidad de los parámetros expresados en "ticks" se convierte contra este valor.

### 2.5 `waypoint/waypoints.toml` - generación de la trayectoria

| Clave | Valor | Función |
|---|---|---|
| `arc_radius` | 0.45 | **Límite superior** del radio del arco de esquina, no el radio en sí. Es también la distancia a la que se engrana la maniobra de estacionamiento. |
| `corner_arc_assume_wide` | true | Dimensionar cada arco como si ambos pasillos fueran anchos, prescindiendo de la estimación de anchura. |
| `wide_center_bias_m` / `wide_center_bias_side` | 0.10 / `inner` | Desplazamiento de la línea central en pasillo ancho, hacia el interior. |
| `narrow_center_bias_m` / `narrow_center_bias_side` | 0.0 / `inner` | Equivalente para pasillo estrecho (actualmente sin sesgo). |
| `unconfirmed_width_inner_bias_m` | 0.05 | Sesgo aplicado mientras la anchura sigue siendo una estimación a priori y no una medición. |
| `defer_current_corridor_replan` | true | Retener un cambio de anchura hasta abandonar el pasillo que describe. Esta medida resolvió la oscilación del primer giro (596 -> 638/640). |
| `obstacles_center_bias_m` | 0.15 | Sesgo de la trayectoria hacia el interior en Obstacles. |
| `finish_approach_m` | 0.40 | Distancia previa a la meta en la que se reduce a `slow_mps` durante la última vuelta. |
| `first_lap_corner_caution` | true | Reducir la velocidad de aproximación a curva en la primera vuelta. |
| `replan_blend_ticks` | 0 | Mezcla entre la trayectoria anterior y la nueva. **Refutado en dos ocasiones**; permanece en 0. |

### 2.6 `blind_nav/` - navegación previa al conocimiento de la pista

`corridor_follower.toml` es el fichero de mayor tamaño y el que determina el arranque y las esquinas
a ciegas:

| Clave | Valor | Función |
|---|---|---|
| `turn_clearance_m` | 0.60 | Holgura frontal a partir de la cual se inicia el giro de esquina. |
| `narrow_turn_clearance_m` | 0.40 | Equivalente cuando el pasillo actual se interpreta como estrecho. |
| `heading_gain` | 0.767945 | Grados de rueda por unidad de error de rumbo respecto al eje del pasillo. |
| `max_centering_steer_deg` / `max_corner_steer_deg` | 13.75 / 21.25 | Límites de dirección en centrado y en esquina. |
| `steer_cap_from_commit_distance` | true | Escalar dicho límite según la distancia a la que se compromete cada rama (redujo las colisiones de 18 a 3). |
| `corner_speed_scale` / `reverse_speed_scale` | 0.6 / 0.6 | Fracción de `creep` al girar y al retroceder. |
| `min_forward_clearance_m` / `min_reverse_clearance_m` | 0.30 / 0.30 | Condición para retroceder y espacio requerido en la parte posterior. |
| `assume_bay_start` | true | Iniciar una ronda de Obstacles asumiendo que el robot se encuentra dentro de la bahía. |
| `bay_exit_clearance_guard` + `bay_exit_clearance_margin_m` | true / 0.001 | Delimitar cada tramo de salida por holgura **prevista** y no por contacto: un contacto constituye la infracción 9.24.7. |
| `bay_exit_target_yaw_deg` | 70.0 | Rotación respecto a la pose de colocación con la que se da por concluida la salida. |
| `bay_exit_latch_direction` + `bay_exit_open_side_votes` | true / 5 | Determinar el lado abierto una sola vez, mediante votación por sector, en lugar de en cada ciclo. |

Restricción cruzada verificada durante la carga: `corridor_follower.turn_clearance_m` <
`direction_estimator.corner_clearance_m`. El intervalo entre ambos valores constituye la única
ventana en la que el robot, todavía alineado con el pasillo, puede determinar qué lado está abierto.
Si se solapan, la dirección no llega a fijarse.

`direction_estimator.toml`: `corner_clearance_m = 1.00`, `min_votes = 5`, `min_asymmetry_m = 0.20`,
`alignment_tolerance_rad = 0.436` (25°), `max_in_track_range_m = 4.5`.

`corridor_estimator.toml`: `min_samples = 12` lecturas para dar por válida una anchura,
`decision_boundary_m = 0.80` (punto de corte entre 0.6 y 1.0), `max_start_samples = 20`.

`localization.toml`: `max_speed_mps = 0.60` actúa como salvaguarda frente a saltos imposibles (se
descarta toda pose que implique una velocidad superior); `relocalize_cost_threshold = 0.03` y
`relocalize_after_scans = 15` gobiernan la relocalización global. Este bloque determina si el robot
es capaz de recuperarse de una pose corrupta.

### 2.7 `sensors/lidar_sectors.toml`

| Clave | Valor | Función |
|---|---|---|
| `front_half_fov_deg` | 30.0 | Cono con el que se calcula la holgura frontal. |
| `threat_half_fov_deg` | 45.0 | Sectores de detección de amenaza lateral. |
| `min_valid_range_m` | 0.05 | Umbral inferior de lectura válida. **Debe situarse por debajo del mínimo real del sensor** (0.045 en el C1): en caso contrario, un robot encajado registra riesgo crítico de forma incondicional. |
| `self_detection_threshold_m` | 0.08 | Reflexiones del propio chasis y del cableado. |
| `rear_self_detection_from_chassis` | true | Filtrar el sector posterior por geometría del chasis y no mediante un umbral constante. |
| `blind_wedge_*` | ±120..160 | Cuñas ocluidas por la propia estructura. |
| `direction_arc_half_fov_deg` | 8.0 | Cono estrecho de detección de fin de pasillo (distinto del frontal). |

### 2.8 `signs/` - Obstacle Challenge

`sign_discovery.toml` (percepción de pilares):

| Clave | Valor | Función |
|---|---|---|
| `max_ingest_range_m` | 1.5 | Distancia máxima a la que se acepta una observación. |
| `min_hits` | 3 | Observaciones confirmatorias requeridas antes de publicar un pilar. |
| `association_dist_m` | 0.25 | Distancia máxima para considerar que dos observaciones corresponden al mismo pilar. |
| `range_scale` | 1.95 | Corrección empírica sobre el rango del modelo estenopeico. Solo resulta eficaz acompañada de `vision_latency_s`. |
| `vision_latency_s` | 0.85 | Latencia cámara -> pose, aplicada cuando la detección no incorpora marca temporal de captura. |
| `max_pillar_aspect` | 1.0 | Toda caja delimitadora más ancha que alta se descarta como pilar. |
| `lidar_range_fusion` | false | Tomar el rango del rayo LIDAR en la latencia de la cámara. |

`sign_router.toml` (evitación):

| Clave | Valor | Función |
|---|---|---|
| `activation_dist_m` / `passed_dist_m` | 1.40 / 1.60 | Punto en el que se activa la deformación y punto en el que el pilar se considera superado. |
| `sign_clearance_margin_m` | 0.10 | Margen sobre la suma de semianchura de chasis y pilar. |
| `sign_lane_planner` | true | El planificador de carril es la vía de evitación destinada a producción. |
| `sign_lane_offset_frac` / `sign_lane_ramp_m` / `sign_lane_hold_m` | 1.0 / 0.9 / 0.25 | Magnitud del desplazamiento del carril, distancia de entrada y distancia de mantenimiento. |
| `sign_lane_corner_entry_m` | 0.5 | Distancia a la esquina a partir de la cual se suprime la entrada en carril. |
| `sign_aware_speed` | true | Reducir a `slow` mientras el enrutador mantiene una deformación activa. |
| `sign_aware_lookahead` | true | Emplear la anticipación **corta** mientras haya un pilar comprometido. |
| `commit_hysteresis` | true | Una vez comprometido un pilar, mantenerlo hasta haberlo superado. |
| `escape_mask_radius_m` | 0.12 | Los retornos LIDAR próximos a un pilar en curso no activan el escape reactivo. |
| `min_confidence` | 0.25 | Confianza mínima para aceptar el color procedente de la cámara (el color determina el lado de paso). |

Este fichero contiene además un bloque de experimentos **refutados y desactivados** que no conviene
reactivar sin repetir la medición: `retrace_escape`, `sign_contact_evade`, `sign_lidar_align`,
`stale_target_rescue`.

### 2.9 `escape/escape.toml` - maniobras de desatasco

`k_turn_min_s` 0.54 / `k_turn_max_s` 1.08 y `max_escape_s` 1.8 son **límites superiores**, no
duraciones: la rotación real medida por episodio se sitúa en torno a los 19°. `rev_speed` -0.20 y
`rev_steer_deg` 44.0 definen el retroceso; `stuck_timeout_s` 2.0 y `stuck_move_threshold` 0.03
definen qué se considera un atasco; `escalate_after_attempts` 3 y `escape_side_commit_attempts` 2
impiden que el escalado se cancele a sí mismo alternando de lado en cada intento.

### 2.10 `parking/parking.toml`

`attempt_after_final_lap = false` es la variable de mayor impacto del fichero: determina si, tras la
última vuelta, se persigue la bahía o se detiene el robot en la sección de meta. Perseguirla
constituía el coste dominante del Obstacle Challenge (in-time 62 -> 158, colisiones 51 -> 4 al
desactivarla). El resto del fichero define la maniobra: `speed` 0.12, `wall_standoff_m` 0.05,
`marker_standoff_m` 0.01, `parallel_tolerance_m` 0.02 (regla WRO), `default_max_frames` 400 (20 s a
20 Hz).

### 2.11 `simulation/simulation.toml` - exclusivo del simulador

Define la fidelidad del banco de pruebas, no el robot: `no_progress_window_s` 30.0 y
`no_progress_displacement_m` 0.08 (abandono por ausencia de progreso), `lidar_invalid_ray_rate` 0.01,
`start_collision_window_s` / `grace_s`, y `vision_through_pinhole` (emular las cajas delimitadoras de
la cámara y decodificarlas con el código real, en lugar de proporcionar coordenadas verdaderas).

`MIN_TURN_RADIUS_M = 0.29` **no figura en el TOML**: existe únicamente como valor por defecto del
modelo. Corresponde al radio mínimo de giro medido del chasis; el modelo cinemático que prescindía de
él predecía 1.5 cm y sobregiraba en un factor de 12.7.

### 2.12 Hechos, no ajuste: `robot.toml`, `track.toml`, `competition_specs.toml`

- `robot.toml`: chasis 0.30 x 0.194 m, `wheelbase` 0.19, `yaw_gain = 0.55` (fracción del giro
  previsto que el chasis entrega realmente), LIDAR con `min_range` 0.045 y `mount_x_offset` 0.1222
  con `inverted = true`, cámara con `hfov` 1.7802 rad y `mount_pitch` 0.1745 rad. `max_speed_mps`
  está **deliberadamente ausente**: describe un motor concreto y reside en su perfil.
- `track.toml`: pista de 3.0 m, pasillos de 0.6 / 1.0 m, pilares de 0.05 x 0.05 x 0.10 m sobre una
  retícula de profundidades 1.0 / 1.5 / 2.0 m, bahía de estacionamiento de 0.20 m.
- `competition_specs.toml`: `round_time_limit_s = 180.0` y 3 vueltas por reto. El límite de 180 s
  constituye el presupuesto que toda ejecución debe respetar.

### 2.13 Ficheros ajenos al árbol compartido pero relevantes

- `platform/robot/config/hardware/state_machine/state_machine_node.toml`:
  `challenge_mode_timeout_sec = 180.0` y `challenge_mode_samples_required = 3`. Determinan el tiempo
  de espera del puente físico que selecciona el reto. Con un tiempo de espera reducido, una ronda de
  Obstacles llegó a ejecutarse como Open.
- `platform/robot/config/hardware/vision/detector.toml`: `min_confidence = 0.45` del detector.
- `platform/robot/config/launch/race.toml`: retención de registros (`bag_max_runs`,
  `bag_max_total_gb`).

---

## 3. Qué lee cada implementación

Python (`NavigationTuning.load_default(challenge=...)`) carga **los 19 grupos** y es la única que
aplica la capa por reto.

Go carga cada fichero de forma independiente y **recurre a sus literales** si un fichero falta o su
lectura falla (`ConfigFor(logger, configRoot, profiles)`). Lee: `clearance`, `speed`, `heading`,
`pursuit`, `waypoints`, `control`, `lidar_sectors`, `escape`, `sign_router`, `corridor_estimator`,
`corridor_follower`, `direction_estimator`, `localization`, `parking`, `wall_heading`,
`start_measurement`, `simulation`, `robot`, `track`.

Divergencias reales que conviene tener presentes:

1. **`sign_discovery.toml` no se lee en Go.** `signrouter.DefaultDiscoveryConfig()` está codificada
   en el propio código y su `MaxIngestRangeM` vale 2.0 frente al 1.5 del TOML. Modificar el TOML no
   altera el comportamiento de la implementación Go.
2. **`sensor.toml` y `state_estimator.toml` carecen de equivalente en Go.**
3. **Go no admite `navigation-challenges/`**: no dispone de capa por reto.
4. `speed.toml` y `heading.toml` se cargan fuera de `internal/config/profile`, en
   `internal/nav/navigator/config_profile.go`, porque la capa del motor reside en
   `profiles/<nombre>/motion/speed.toml`, formato que `profile.Load` no interpreta.
5. Un cargador de configuración en Go puede devolver **éxito sin haber leído la clave** que necesita
   (por ausencia de la etiqueta `mapstructure`). Es un fallo sistemático ya observado: tras modificar
   un TOML, conviene verificar que el valor llega comparando dos ejecuciones, y no limitarse a
   comprobar que el proceso arranca.

---

## 4. Cómo modificar un valor sin introducir regresiones

1. Editar el TOML base, no el valor por defecto del modelo (el TOML prevalece sobre el modelo).
2. Si el cambio describe una **pieza física**, corresponde al perfil de esa pieza, no al fichero
   base.
3. Si el cambio solo es aplicable a un reto, emplear la clave con prefijo (`open_*` / `obstacles_*`)
   o la capa `navigation-challenges/<reto>/`. Conviene recordar que Go ignora dicha capa.
4. Medir mediante A/B contra el **commit padre**, con idéntico conjunto de casos en ambas ramas: los
   resultados de los barridos no son comparables entre fechas distintas.
5. Antes de interpretar un resultado, comprobar que ambas ramas difieren efectivamente. Un cableado
   inactivo devuelve ramas idénticas byte a byte, lo que se interpreta erróneamente como ausencia de
   efecto.

---

## 5. Las variables que más determinan el resultado

**Open Challenge**

1. `speed.open_*_mps` (escalera de velocidad del reto).
2. `heading.crawl` (corte binario de velocidad por error de rumbo, activo durante buena parte de la
   vuelta).
3. `pursuit.lookahead_*` y `corner_preview_distance_m` (momento en que se anticipa la curva).
4. `corridor_follower.turn_clearance_m` / `narrow_turn_clearance_m` frente a
   `direction_estimator.corner_clearance_m` (la ventana en la que se fija la dirección).
5. `waypoints.defer_current_corridor_replan` y los sesgos de línea central.
6. `clearance.*` (expresados en metros, no en tiempo: acotan la velocidad utilizable).

**Obstacle Challenge**

1. `parking.attempt_after_final_lap` (perseguir la bahía o detenerse en la meta).
2. `sign_router.activation_dist_m`, `sign_lane_*` y `sign_aware_speed` (la evitación en su
   conjunto).
3. `sign_discovery.range_scale` junto con `vision_latency_s` (la pose del pilar; por separado
   carecen de efecto).
4. `pursuit.obstacles_yaw_gain_compensation` (compensación del sobregiro en esquina).
5. `clearance.obstacles_contact_dist` y `escape.*` (contacto y desatasco).
6. `corridor_follower.assume_bay_start` y el bloque `bay_exit_*` (arranque desde el interior de la
   bahía).
