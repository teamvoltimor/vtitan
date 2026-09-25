# Reutilización de vtitan si cambia el reto WRO

Fecha: 2026-09-24. Estado: catálogo de POSIBILIDADES, nada decidido ni implementado.

Pregunta de partida: si el reto WRO (Future Engineers 2026) cambiara, ¿qué se aprovecha
del proyecto, qué se puede extender para que sea reutilizable y qué habría que añadir?

**Cómo leer este documento.** Nada de lo que sigue es un compromiso. Son opciones que
se activan según lo que traiga el reto nuevo (o un uso fuera de WRO). La sección 5
cruza escenarios de cambio con las opciones que cada uno exige, y la sección 6 da un
orden posible si se decide avanzar. La única opción que conviene aunque el reto no
cambie es separar la frontera del reto (4.1), porque ordena el código actual.

## 0. Método y medida de acoplamiento

Para cada paquete Go se contó cuántos archivos de producción (sin `_test.go`) mencionan
conceptos del reto (`wro`, `pillar`, `parking`, `obstacle`, `lap`). Es una heurística
gruesa: un acierto no implica acoplamiento real (por ejemplo `pkg/transport` lo menciona en
un comentario), pero un 100% sí indica código que ES el reto.

| Acoplamiento | Paquetes (archivos acoplados / total) |
|---|---|
| Nulo | `pkg/backoff` 0/2, `pkg/foxglove` 0/5, `pkg/protoschema` 0/1, `internal/node/{button,imu,lidar,capture,telemetry}`, `internal/nav/stateestimator` 0/3, `internal/statemachine/command` 0/4 |
| Bajo (comentarios o un punto de contacto) | `pkg/geom` 1/6, `pkg/control` 1/3, `pkg/supervise` 1/5, `pkg/transport` 1/5, `pkg/recording` 3/11, `pkg/portable` 4/25, `pkg/driver` 9/40, `internal/nav/localization` 1/4, `internal/nav/wallheading` 1/2, `internal/sim/kinematics` 1/5, `internal/sim/sensorerrors` 1/1, `internal/node/{motor,picolink,statemachine}` |
| Medio | `internal/sim/collision` 5/8, `internal/statemachine/core` 5/7, `internal/simgen/simconfig` 5/7, `internal/simgen/generate` 4/6, `internal/nav/trackmodel` 4/7, `internal/nav/controllers` 7/11, `internal/nav/waypoints` 6/9, `internal/sim/scenario` 8/11 |
| Total (es el reto) | `internal/nav/navigator` 13/13, `internal/nav/signrouter` 10/10, `internal/nav/parking` 7/7, `internal/nav/bayexit` 5/5, `internal/nav/racetracker` 4/4, `internal/nav/corridorfollower` 4/4, `internal/node/nav` 3/3, `internal/sim/harness` 3/3 |

Supuestos estructurales detectados además del vocabulario:

- `[4]Section` escrito a mano en `internal/nav/startmeasurement/startmeasurement.go`,
  `internal/sim/opencorpus/space.go` y `internal/sim/opencorpus/balanced.go`: la pista es un
  anillo cuadrado de cuatro secciones.
- El modelo de paredes del localizador es simétrico de orden 4 (los cuatro pasillos de
  Obstacles miden 1000 mm y el lote de parking no está modelado). La búsqueda global de pose
  está mal planteada en esa pista y puede teleportar a la copia rotada 180 deg.
- `src/config/track.toml` fija la geometría del tapete WRO 2026 (3x3 m, esquinas 1-2 m,
  pasillos 0,6 / 1,0 m, líneas de división 0,40 / 0,60).
- `src/config/competition_specs.toml` fija las reglas (180 s, 3 vueltas por reto).

## 1. Se reutiliza tal cual

### 1.1 Superficie pública `pkg/` (ADR 0094)

| Paquete | Qué aporta |
|---|---|
| `pkg/backoff` | Reintentos con backoff |
| `pkg/supervise` | Supervisión de procesos, `sd_notify`, watchdog de systemd |
| `pkg/transport/nats` | Transporte de mensajes entre nodos |
| `pkg/protoschema` | Esquemas protobuf para herramientas |
| `pkg/foxglove` | Servidor/cliente Foxglove para visualización en vivo |
| `pkg/recording` | Grabación MCAP (ROS CDR), foto y video (ADR 0071) |
| `pkg/geom` | Ángulos, rayos, abanicos, scans |
| `pkg/control` | Pure pursuit y modelo de dirección (casi genérico) |
| `pkg/driver/*` | LIDAR Slamtec C1, IMU, cámara, servo, motor, encoder, botón, display |
| `pkg/portable/*` | Núcleos sin TinyGo del Pico: `boardloop`, `boardlink`, `quadrature`, `hbridge`, `servo`, `actuation`, `bno085rvc` (ADR 0098). Desde `55d854f8` el encoder ya corre en el Pico sobre `quadrature` (ver 4.5) |

Todo esto sirve para cualquier robot con el mismo hardware, compita o no.

### 1.2 Arquitectura del sistema

- **Reparto en dos placas** (ADR 0066): Pi 5 para percepción y navegación, Pico 2 para
  actuación en tiempo real, enlazadas por USB gadget (ADR 0067) con el protocolo
  `picolink`/`boardlink`.
- **Nodos por proceso** que hablan por NATS con contratos protobuf versionados en
  `src/go/proto/vtitan/{sensor,actuation,state,ui,vision,nav}/v1`. Salvo `nav` y parte de
  `state`, los contratos no dependen del reto.
- **Raíz de composición** de la Pi 5 (ADR 0095) y binarios en `src/go/cmd/` (`lidar-node`,
  `imu-node`, `motor-node`, `capture-node`, `telemetry-node`, `foxglove-bridge`, `vt`).
- **Configuración gobernada** (ADR 0069/0070): perfiles de hardware apilables en
  `src/config/profiles/`, overlays por reto, esquemas JSON, `configgen`, carga en runtime
  (ADR 0020).
- **Vision en NPU Hailo 8** con su ruta de datos (ADR 0072) y captura de dataset (ADR 0091).

### 1.3 Hardware y su calibración

- Chasis 30 x 19,4 cm, 1,3 kg, dirección de cuatro ruedas en contrafase (ADR 0075/0076),
  rieles de potencia (ADR 0077), montajes de cámara, IMU y LIDAR (ADR 0078-0080), piezas
  STEP/STL (ADR 0081) y arnés como código (ADR 0082).
- Constantes MEDIDAS que siguen valiendo con otra pista:
  - radio de giro como curva de velocidad `R = 0,053 + 1,86 v` (ADR 0013);
  - `yaw_gain = 0,55` en conducción limpia (ADR 0012);
  - tasa de giro del servo (ADR 0030);
  - envolvente de velocidad, unos 0,58 m/s a `max_duty = 0,5` (ADR 0085);
  - huella de ruido del C1: pierde un 25% de rayos frente al 1% del sim, suelo de rango
    válido (ADR 0041).

### 1.4 Base del simulador

- `internal/sim/kinematics`: cinemática del chasis con la curva de radio real.
- `internal/sim/sensorerrors`: errores realistas de LIDAR e IMU.
- `internal/sim/collision`: detección por rectángulos orientados y contacto que desliza por
  la superficie, en Go desde el ítem 2.11 del plan de plataforma y en Python desde
  `86bee47f` (`contact_slides_along_surfaces = true`, ADR 0062/0086). Invariantes físicos
  (teletransporte, movimiento inventado, penetración) anulan una corrida inválida. La parte
  geométrica es genérica, pero el deslizamiento es por eje, exacto solo con paredes
  alineadas a los ejes; la parte que sabe de pilares y paredes WRO no es genérica.

### 1.5 Metodología (probablemente lo más valioso)

- Bags MCAP de cada ronda real y herramientas `diag_bag_*` para analizarlas sin fiarse de
  la pose.
- A/B sobre un corpus de escenarios deterministas antes de tocar hardware (ADR 0087), con
  la lista de knobs refutados para no repetirlos (ADR 0088).
- ADRs (0001 a 0098 a fecha de hoy), `doc_audit.py` contra la deriva de documentación (ADR 0097),
  convenciones de commits y constantes (ADR 0090).
- CLI `vt` que envuelve los Taskfiles (ADR 0096) y el despliegue a la Pi.

## 2. Se reutiliza si se generaliza

### 2.1 Localización (`internal/nav/localization`, `wallheading`)

- **Ata al reto:** el modelo de paredes describe el cuadrado WRO; con pasillos iguales es
  simétrico de orden 4 y la relocalización global puede aceptar la copia rotada.
  `_relocalize_globally` (Python) cuesta unos 4,8M raycasts por llamada.
- **Generalizar a:** mapa como polígonos o rejilla de ocupación cargada de datos; filtro de
  partículas o ajuste por scan-matching con un término de desempate por landmarks
  (color, objetos únicos, IMU absoluta) cuando el mapa es simétrico; presupuesto de tiempo
  por tick para que la relocalización no bloquee el bucle.

### 2.2 Modelo de pista (`trackmodel`, `startmeasurement`, `opencorpus`)

- **Ata al reto:** cuatro secciones fijas, esquinas en 1-2 m, anillo cerrado.
- **Generalizar a:** una descripción de pista en datos (grafo de tramos o polígono con
  zonas etiquetadas) que salga de un único archivo, como ya hace `track.toml` para las
  constantes. Quitar los `[4]` fijos y derivar secciones del mapa.

### 2.3 Generador de mundos (`internal/simgen`)

- **Ata al reto:** `simconfig` y `generate` producen mapas WRO.
- **Generalizar a:** generar SDF/escenario desde la descripción de mapa del punto 2.2, con
  la validación (`simgen/validate`) y la vista previa (`simgen/preview`) reutilizadas.

### 2.4 Máquina de estados (`internal/statemachine`)

- `command`, `outbox` y `robotcmd` son casi genéricos.
- `core` mezcla el motor de estados con los estados de una ronda WRO (arranque en bahía,
  vueltas, parking).
- **Generalizar a:** motor genérico más una definición de misión que aporte el reto.

### 2.5 Escenarios y puntuación (`internal/sim/scenario`, `corpus`, `harness`)

- El harness y el formato de corpus son reutilizables.
- El puntuador conoce pilares, parking, pases por el lado malo y desplazamiento de
  obstáculos (ADR 0005, 0034, 0043, 0059).
- **Generalizar a:** interfaz `Scorer` por reto; el corpus guarda qué reto y qué versión
  de reglas usa cada escenario.

### 2.6 Control (`pkg/control`, `nav/controllers`, `corridorfollower`, `stateestimator`)

- Pure pursuit, dirección y estimador de estado son casi genéricos.
- Selección de objetivo (ADR 0052), límites de giro por velocidad y guardas de sentido
  (ADR 0063) se pueden extraer.
- **Generalizar a:** mover lo genérico a `pkg/control` y dejar en `nav` sólo la política.

### 2.7 Visión (`internal/sim/visionsim`, pipeline Hailo)

- **Ata al reto:** clases rojo, verde y magenta.
- **Generalizar a:** clases y colores por configuración; pipeline de captura, etiquetado,
  entrenamiento y compilación HEF documentado de extremo a extremo.
- Además, el emulador de visión idealiza: habla en el 54,5% de ticks contra el 11,6% del
  hardware, latencia cero contra 0,85 s medidos, nunca se equivoca de color. Corregirlo
  sirve para cualquier reto.

## 3. Específico del reto: se rehace

| Paquete | Qué hace |
|---|---|
| `internal/nav/navigator` | Orquestación de ronda Open/Obstacles |
| `internal/nav/signrouter` | Regla de pase por lado según color, planificador de carril (ADR 0051) |
| `internal/nav/parking` | Aparcamiento en el lote |
| `internal/nav/bayexit` | Salida de la bahía de arranque |
| `internal/nav/racetracker` | Conteo de vueltas |
| `internal/nav/waypoints` | Waypoints del anillo |
| `internal/nav/directionestimator` | Sentido de giro (cw/ccw) |
| `src/config/competition_specs.toml` | Reglas de la ronda |

Lo que SÍ se transfiere de aquí son las lecciones, ya recogidas en ADRs:

- Un cambio de banda es un problema de ESPACIO, no de velocidad: a 0,22 m/s exige 0,698 m
  y el robot comprometía a 0,36 m.
- El K-turn es el 88% del tiempo de maniobra y la mitad del sobretiempo.
- Un localizador divergido no se recupera a mitad de carrera (19/19 rondas con 0 vueltas).
- Las guardas que arbitran milímetros con errores de pose de centímetros son ruido
  (ADR 0060).
- Los fallos de pase por el lado malo fueron casi todos de SEGUIMIENTO, no de plan.

## 4. Qué se podría añadir

Cada punto es una opción independiente; su prioridad depende del escenario (sección 5).

Ordenado por retorno.

### 4.1 Frontera explícita del reto (útil en todos los escenarios)

Hoy el reto está repartido entre `nav`, `sim` y `statemachine`. Propuesta:

```go
// Esbozo, no API final.
type Challenge interface {
    Name() string
    Track() TrackSpec          // geometría (2D, 2.5D o 3D, ver 4.8) y zonas etiquetadas
    Rules() RuleSpec           // tiempo, vueltas, penalizaciones
    Mission() MissionSpec      // estados y transiciones de la ronda
    Scorer() Scorer            // puntuación en sim y en análisis de bags
    Policies() PolicySet       // navegación específica (router, parking...)
}
```

- Mover lo WRO 2026 a `internal/challenge/wro2026/`.
- El resto del código sólo conoce la interfaz.
- Documentarlo en un ADR ("el reto es un plugin") antes de mover código.
- Criterio de aceptación: el corpus actual da el mismo resultado antes y después del
  movimiento (refactor sin cambio de comportamiento).

### 4.2 Mapa genérico y localización con desempate

Ver 2.1. Arregla el problema de simetría actual y cualquier pista futura. Conviene
añadir el lote de parking y los objetos fijos al modelo para romper la simetría ya en
WRO 2026.

### 4.3 Planificador consciente de la cinemática

Sustituir los carriles precalculados por un planificador genérico (Hybrid A* o lattice)
que use la curva `R(v)` medida y el hueco trasero para maniobras con reversa. El
`signrouter` pasaría a ser una capa de restricciones ("pasar este objeto por la
izquierda") sobre ese planificador.

### 4.4 Fidelidad del simulador

Abiertos a fecha de hoy:

- rotación durante escapes: el sim gira un 40-50% de menos;
- el sim nunca satura el volante (hardware 8-17%);
- visión: tasa, latencia y errores de color (2.7);
- LIDAR: bandas frontales sin declarar y pérdida de rayos.

### 4.5 Placa de actuación (Pico 2) como plataforma

#### Estado a 2026-09-23 (ya en `origin/master`)

- `948f49dc`: el Pico reenvía `ButtonHold` por `picolink` además de `ButtonEvent`.
  `Evaluator.Held(now)` en `pkg/driver/button` da la duración en curso en cada tick;
  `internal/node/picolink.Session` publica cada 50 ms mientras se mantiene pulsado (con
  los umbrales "long" y "shutdown") y una trama vacía al soltar, en el sujeto NATS
  `vtitan.ui.v1.button_hold`. Paridad con `button_node.py._publish_hold_progress`.
- `55d854f8`: encoder de rueda decodificado en el Pico con interrupciones GPIO por flanco
  (`encoderPins` en `firmware/pico2/hw.go`) que alimentan
  `pkg/portable/quadrature.Decoder`, con sección crítica porque un `int64` no es atómico
  en el núcleo de 32 bits. El firmware compila para `pico2` (78.484 B de flash).
- ADR 0098 registra la desviación: pedía contar con PIO por hardware, pero TinyGo 0.42.0
  no tiene API de PIO (ni cargador de programas ni envoltorio de registros).

#### Por qué importa para la reutilización

- **La lógica ya es portable.** `pkg/portable/boardloop` (sesión y protocolo),
  `quadrature`, `hbridge`, `servo`, `actuation` y `bno085rvc` no dependen de TinyGo y se
  prueban en el host (`boardloop` ya tiene `fakes_test.go`). `firmware/pico2` es una capa
  fina de adaptadores.
- **Cambiar de microcontrolador (escenario G) es escribir otro adaptador**, no reescribir
  el firmware: otro chip compatible con TinyGo implementa las mismas cinco interfaces.
- **El protocolo `picolink`/`boardlink` es genérico**: botón, encoder, IMU, motor y servo
  viajan como mensajes; un robot nuevo añade mensajes, no un protocolo nuevo.
- **La mayor debilidad es de verificación, no de diseño:** no hay placa física aquí, así
  que nada del Pico está verificado en banco.

#### Posibilidades abiertas

| Opción | Qué es | Estado | Valor para reutilizar |
|---|---|---|---|
| Arnés Go nativo | Falsos (sin TinyGo) de las cinco interfaces de `firmware/pico2` sobre `boardloop`, siguiendo el patrón del harness de `internal/node/picolink` | Hecho: `pkg/boardsim` (`233f94ae`, `f6e67f89`), con emulación del enlace configurable | Alto: prueba de extremo a extremo Pi-Pico en CI sin dependencias nuevas; sirve para cualquier placa futura |
| Spike con Bramble | Emulador en C de RP2350 (Cortex-M33, GPIO/PWM/USB/watchdog, carga ELF), `github.com/Night-Traders-Dev/Bramble` | Propuesto, no hecho | Medio: ejecutaría el ELF real, pero es de un solo mantenedor, tiene 10 meses y sus afirmaciones no están verificadas |
| PIO real para el encoder | Contar cuadratura por PIO en vez de interrupciones | Pendiente (`TODO(encoder-pio)`) | Medio: libera CPU a altas tasas de flancos; exige codificar a mano las instrucciones PIO o esperar soporte en TinyGo |
| Verificación en banco | Probar botón, encoder, motor y servo con la placa real | Pendiente, sin placa | Imprescindible antes de fiarse del Pico en un reto nuevo |

Descartadas (no reabrir sin un hecho nuevo):

- **Renode:** no soporta RP2350; el único simulador comunitario es de RP2040 (otro chip),
  está congelado y le faltan PWM y USB, que aquí son esenciales.
- **rp2040js (Wokwi):** mantenido y con PWM y USB, pero sólo RP2040; el soporte RP2350 es
  un issue abierto sin implementar.

Orden razonable si se decide avanzar: arnés Go nativo primero (cero dependencias, valor
permanente), verificación en banco en cuanto haya placa, y Bramble o PIO sólo si el
arnés o el banco muestran una necesidad concreta.

### 4.6 Cerrar la migración a Go

Decidir el objetivo de paridad con Python (ADR 0068) y hacer el corte, para no llevar dos
implementaciones al siguiente reto. Es una decisión pendiente del equipo, no técnica:
qué parte del comportamiento de Python debe reproducir Go antes del corte. El seguimiento
detallado vive en el plan de migración local del equipo (no versionado), que ya
lista los pendientes abiertos de 4.5. Actualización 2026-09-24: ADR 0068 ya define
el objetivo como paridad funcional por resultado (`7f21e65f`).

Relación con la reutilización: todo lo que se generalice (4.1 a 4.8) debería hacerse sólo
en Go. Generalizar también la versión Python duplicaría el trabajo.

### 4.7 Otras piezas genéricas útiles

- Teleoperación y grabación de dataset como modo de primera clase (ya hay
  `hardware/teleop.toml`).
- Plantilla de "nuevo reto": `track`, `rules`, overlay de config, corpus inicial y ADR.
- Herramientas de bags independientes del reto (pose, contactos, tiempos) separadas de
  las específicas (lado de pase, parking).

### 4.8 Soporte de mapas 3D

Hoy todo el stack es 2D: pose `(x, y, yaw)`, pista plana, LIDAR de un solo plano,
cinemática plana. Si el reto o un uso futuro trae rampas, desniveles, varios pisos u
obstáculos que no cortan el plano del LIDAR, hace falta un camino a 3D. Conviene
diseñarlo desde la frontera del reto (4.1) para que el 2D sea un caso particular y no
una rama aparte.

#### Qué ya existe y sirve

| Pieza | Estado |
|---|---|
| `internal/simgen/sdf` (`world.go`, `track.go`, `objects.go`, `robot.go`) | Ya emite mundos SDF con geometría 3D (paredes con altura, objetos). Es la puerta natural a un simulador físico 3D (Gazebo) |
| `pkg/portable/bno085rvc` | La IMU entrega yaw, pitch y roll y construye cuaterniones (`QuaternionFromEuler`). Hoy sólo se usa el yaw |
| `src/config/track.toml` | Ya declara alturas (`wall.height`, alturas de señales y objetos) |
| ADR 0014 / 0080 | El plano de escaneo del LIDAR está medido (altura y montaje), necesario para razonar qué ve el LIDAR en 3D |
| `pkg/geom` | Primitivas 2D; ampliables a 3D sin romper lo existente |

#### Niveles de ambición

Se proponen tres niveles incrementales; cada uno es útil por sí solo.

**Nivel 1: 2.5D (mapa de elevación).**

- Representación: rejilla 2D donde cada celda guarda altura y pendiente (o una lista de
  planos inclinados en la descripción de pista).
- Pose: sigue siendo `(x, y, yaw)` más `z`, `pitch` y `roll` DERIVADOS del mapa y
  corroborados por la IMU.
- Cubre rampas, badenes y plataformas sin cambiar la navegación de fondo.
- Coste bajo: el planificador y el localizador siguen trabajando en el plano, proyectando
  sobre la superficie.

**Nivel 2: 3D con un solo nivel transitable.**

- Representación: vóxeles u octree (estilo OctoMap) para obstáculos con altura
  arbitraria (voladizos, túneles, objetos bajo el plano del LIDAR).
- Pose completa 6DoF `(x, y, z, roll, pitch, yaw)` en el estimador y en los mensajes.
- Colisión 3D en el simulador (OBB 3D o malla simplificada del chasis).

**Nivel 3: 3D multinivel.**

- Varios pisos conectados por rampas: grafo de superficies transitables sobre el mapa
  3D.
- Localización que distinga pisos con la misma planta (el problema de simetría de 2.1,
  ahora vertical): altura integrada, pendiente recorrida y landmarks.

#### Problemas concretos del hardware actual

- **El LIDAR C1 es de un solo plano.** En una rampa el plano se inclina con el chasis:
  subiendo mira al suelo y pierde paredes; bajando pasa por encima de paredes de 10 cm.
  El localizador actual interpretaría eso como paredes que aparecen y desaparecen.
  Mitigaciones: compensar cada rayo con pitch/roll de la IMU y proyectar sobre el mapa 3D
  (sólo mide lo que corta el plano inclinado); marcar como no fiables los rayos que
  cortan el suelo; o añadir un sensor 3D.
- **Obstáculos fuera del plano.** Lo que queda por debajo del plano recesado
  (ADR 0014) o por encima es invisible para el LIDAR; sólo la cámara lo ve.
- **La IMU es la referencia de inclinación.** Hay que medir el ruido y la deriva de
  pitch/roll de la BNO085 igual que se hizo con el yaw (ADR 0012/0079) antes de fiarse
  de ellos.
- **La dinámica cambia en pendiente.** El modelo de velocidad (`max_duty`, envolvente
  ADR 0085) y la curva `R(v)` se midieron en plano; en rampa cambian el par necesario y
  el deslizamiento. Hace falta control de velocidad en lazo cerrado con el encoder
  (el decoder del Pico ya existe) y re-medir en pendiente.

#### Sensores a considerar

| Opción | Aporta | Coste |
|---|---|---|
| Compensar el C1 con IMU (sin hardware nuevo) | Nivel 1 en rampas suaves | Bajo; sólo software |
| LIDAR inclinado o un segundo LIDAR vertical | Perfil del suelo y obstáculos bajos | Montaje y otra fuente de datos |
| Cámara de profundidad (estéreo o ToF) | Nube de puntos frontal para niveles 2 y 3 | CPU en la Pi 5 (ya ~82 C con carga) o uso de la NPU |
| Visión monocular + IMU (VIO) | Pose 6DoF sin sensor nuevo | Complejo, sensible a textura y luz |
| LIDAR 3D | Lo más completo | Precio, peso y potencia |

La restricción térmica de la Pi 5 (82,5 C con ventilador al máximo, `vision_node` al 105%
de un núcleo) es relevante: cualquier procesado 3D denso debe ir a la NPU Hailo o a una
tasa reducida.

#### Cambios de software por capa

| Capa | Cambio |
|---|---|
| Contratos (`src/go/proto/vtitan/*/v1`) | Pose 6DoF y nube de puntos como mensajes nuevos (`v2` o campos opcionales), manteniendo compatibles los consumidores 2D |
| Descripción de pista (`TrackSpec`) | Superficies con altura y pendiente; objetos con volumen; el caso plano es `z = 0` en todo el mapa |
| Mapa | Interfaz `Map` con consultas `RayCast`, `Occupied`, `SurfaceAt(x, y)`; implementaciones 2D (actual), elevación y vóxeles |
| Localización | Estado 6DoF con pitch/roll de la IMU; modelo de observación del LIDAR como plano inclinado |
| Planificación | Costes por pendiente, límites de inclinación del chasis, zonas no transitables por altura |
| Simulador | `kinematics` y `collision` con `z` y pendiente; o delegar física 3D a Gazebo a partir del SDF de `simgen` y mantener el sim propio rápido para el corpus 2D |
| Visualización | Foxglove ya soporta escenas 3D y nubes de puntos: publicar mapa y pose 3D en `pkg/foxglove` |
| Grabación | MCAP ya admite cualquier esquema; añadir los nuevos mensajes |

#### Decisión clave: simulador propio o Gazebo para 3D

- **Simulador propio extendido:** rápido, determinista, calibrado contra bags. Es lo que
  hace posible el A/B por corpus. Extenderlo a 2.5D es razonable; a 3D completo con
  contactos es mucho trabajo.
- **Gazebo desde el SDF de `simgen`:** física 3D resuelta y sensores 3D simulados; más
  lento, menos determinista y sin la calibración ya hecha.
- **Recomendación:** sim propio para 2D y 2.5D (sigue siendo la puerta del corpus);
  Gazebo sólo para validar el nivel 2 o 3 mientras no exista un corpus 3D propio.

#### Criterios de aceptación

- El corpus 2D actual da el mismo resultado con la interfaz `Map` genérica.
- Una pista de prueba con rampa se genera, se simula y el localizador no pierde la pose
  al subir ni al bajar.
- Bag real en rampa: la pose `z`/pitch estimada coincide con la geometría medida dentro
  de una tolerancia fijada de antemano.

## 5. Escenarios posibles y qué exige cada uno

Posibles cambios del reto (o usos fuera de WRO) y qué opciones de la sección 4 harían
falta. "Base" es lo de la sección 1, que se reutiliza en todos.

| Escenario | Qué cambia | Se reutiliza | Opciones necesarias | Esfuerzo relativo |
|---|---|---|---|---|
| A. Mismo tapete, reglas nuevas | Tiempo, vueltas, puntuación, colores o reglas de pase | Base + casi toda la navegación | 4.1; ajustar `competition_specs.toml`, `signrouter` y puntuador | Bajo |
| B. Nuevo tipo de objeto o señal | Objetos distintos a pilares (formas, colores, señales de texto) | Base + localización + control | 4.1; visión configurable (2.7); reentrenar modelo; nueva política | Medio |
| C. Pista 2D con otra forma | No cuadrada, más o menos secciones, pasillos variables, sin anillo | Base + control + visión | 4.1, 2.2 (pista en datos), 4.2 (mapa genérico), 2.3 (simgen) | Medio-alto |
| D. Misión distinta en 2D | Recoger y dejar objetos, seguir ruta, aparcar varias veces, navegar a metas | Base + localización | 4.1, 2.4 (máquina de estados genérica), 4.3 (planificador genérico) | Alto |
| E. Terreno con relieve | Rampas, badenes, plataformas | Base + casi todo lo 2D | 4.8 nivel 1 (2.5D), re-medir dinámica en pendiente | Medio-alto |
| F. Entorno 3D completo | Obstáculos fuera del plano del LIDAR, túneles, varios pisos | Base (drivers, transporte, grabación, metodología) | 4.8 niveles 2-3, sensor 3D, pose 6DoF, Gazebo | Muy alto |
| G. Cambio de hardware permitido | Otro sensor, otro chasis, otro microcontrolador | Arquitectura, contratos, metodología | Perfiles de hardware (ya existen), nuevos drivers en `pkg/driver`, nuevo adaptador de `firmware/` sobre `pkg/portable` si cambia el microcontrolador (4.5), recalibrar | Variable |
| H. Uso fuera de WRO | Robot educativo, investigación, otra competición | Base completa | 4.1 y publicar `pkg/` como librería estable | Medio |

Observaciones:

- Los escenarios se combinan (por ejemplo C + E). Las opciones son acumulativas: E
  presupone C, y F presupone E.
- 4.1 aparece en todos. Es la única opción con retorno garantizado.
- 4.2 (mapa genérico) aparece desde C en adelante. Si se hace, conviene diseñar su
  interfaz pensando en 3D (4.8) aunque se implemente en 2D: el coste extra es pequeño y
  evita rehacerla.
- 4.4 (fidelidad del sim), 4.5 (firmware del Pico sin placa) y 4.6 (migración a Go) son
  independientes del escenario: mejoran la plataforma en cualquier caso.

### Señales para decidir

Qué mirar en un reglamento nuevo para saber en qué escenario estamos:

- ¿Cambian las dimensiones o la forma del tapete? Si sí, C.
- ¿Aparecen objetos nuevos o cambian las reglas de interacción con ellos? B o D.
- ¿Hay tareas distintas de dar vueltas? D.
- ¿Hay elementos con altura relevante, rampas o superficies no planas? E o F.
- ¿Cambian las restricciones de hardware (tamaño, sensores, controladores)? G.

## 6. Orden posible si se decide avanzar

No es un plan comprometido: es el orden en que las opciones se apoyan unas en otras.
Cada fase se puede detener sin dejar el sistema a medias.

| Fase | Trabajo | Resultado verificable |
|---|---|---|
| 1 | ADR de la frontera del reto e inventario de qué se mueve | ADR aceptado |
| 2 | Mover WRO 2026 a `internal/challenge/wro2026/` sin cambiar comportamiento | Corpus idéntico, CI verde |
| 3 | Pista en datos; quitar los `[4]Section` | Corpus idéntico; una pista de prueba no cuadrada genera y corre |
| 4 | Mapa genérico y localización con desempate | Sin teleports a la copia rotada en el corpus y en bags |
| 5 | Planificador genérico con `R(v)` | Corpus igual o mejor en Obstacles |
| 6 | Fidelidad de sim y visión configurable | Métricas bag-vs-sim dentro de tolerancia |
| 7 | Mapas 3D nivel 1 (2.5D): interfaz `Map` con `SurfaceAt`, pitch/roll de la IMU, LIDAR compensado | Pista con rampa en sim y bag real en rampa dentro de tolerancia |
| 8 | Mapas 3D niveles 2 y 3: pose 6DoF, vóxeles, sensor 3D, validación en Gazebo | Decidido según el reto; requiere ADR de sensor |

La fase 4 debe diseñar la interfaz `Map` pensando ya en 3D (consultas por superficie y
rayos con `z`), aunque sólo implemente la versión 2D; así la fase 7 no rompe nada.

Qué fases activar según el escenario de la sección 5: A y B, fases 1-2; C, 1-4; D,
1-5; E, 1-4 y 7; F, todas. Las fases 6 (fidelidad) y las opciones 4.5 y 4.6 se pueden
hacer en paralelo en cualquier momento.

Las fases 1 y 2 valen aunque el reto no cambie: separan lo que es plataforma de lo que es
WRO 2026 y facilitan el trabajo actual.
