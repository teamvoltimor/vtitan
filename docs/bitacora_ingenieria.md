# Bitácora de Ingeniería - V-Titan, WRO Future Engineers 2026

Esta bitácora recoge las decisiones de ingeniería de V-Titan tal como se tomaron: qué nos
obligó a decidir, qué alternativas teníamos, qué elegimos y qué midió el cambio. Está
construida sobre el historial real del proyecto, los commits y las notas que fuimos dejando
mientras desarrollábamos, en vez de reconstruida de memoria al final.

Cada entrada sigue la misma plantilla: contexto o restricción, opciones consideradas, qué
hicimos, por qué, resultado medido y la referencia al commit o a la nota donde vive el
detalle. Varias entradas registran hallazgos incómodos -mediciones que refutaron una
hipótesis nuestra, o riesgos que siguen abiertos- porque son parte del proceso tanto como los
aciertos.

Las cinco secciones siguen los criterios de evaluación del reglamento. Como muchas decisiones
son transversales, cada entrada lleva además una línea **Categorías**, y el índice siguiente
agrupa las mismas entradas por tema técnico para quien quiera leer "todo lo del motor" o
"todo lo del LIDAR" de corrido.

---

## Índice por categoría

Un fallo del puente H, por ejemplo, es a la vez hardware eléctrico y una decisión de
seguridad, y aparece bajo ambas etiquetas.

| Categoría | Entradas |
|---|---|
| `motor` | Cambio de motor de tracción (2026-08-27) · `counts_per_rev` es 60 (2026-08-29) · Techo de velocidad 0.58 m/s (2026-08-30) |
| `puente-h` / `eléctrico` | Puente H recableado + fallo del servo (2026-08-27→28) · GPIO flotante, tres capas (2026-08) · PWM compartido → RPWM/LPWM independientes · Pull-down `LPWM` pendiente (riesgo abierto) |
| `mecánica` / `dirección` | Giro a tope de dirección irreal (2026-08-27) · `MAX_STEERING_RATE` sin medir (2026-09-02) · Puente H recableado + fallo del servo (2026-08-27→28) |
| `sensores` / `lidar` | Recalibración de holgura por el desplazamiento del LIDAR (2026-08-22) · Montaje invertido necesita espejo (2026-08-31) · Indicador de localización invertido (2026-08-22) |
| `sensores` / `cámara` | Cámara montada invertida (fecha no registrada) |
| `calibración` | Cambio de motor (2026-08-27) · `counts_per_rev` (2026-08-29) · Techo de velocidad (2026-08-30) · Holgura del LIDAR (2026-08-22) |
| `software-navegación` / `obstáculos` | Dos bay-exits por modelo de contacto (2026-09-02) · Escape no avanza a ciegas (2026-08-28) · Pass-side es seguimiento, no enrutado (2026-09-01) · `OBSTACLES_CONTACT_DIST` (2026-08-31) · Corredor de escape degenerado (2026-09-02) · Escalera de Open (2026-09-02) |
| `parking` / `reglamento` | Modelo de contacto por defecto elegido por reglamento (2026-09-03) · Dos bay-exits (2026-09-02) |
| `arquitectura` / `software` | Perfiles de hardware por componente (2026-08-21) · Montaje invertido LIDAR (2026-08-31) · Migración a Go/NATS (en curso) |
| `decisión-sistema` | Cambio de motor (2026-08-27) · Modelo de contacto por reglamento (2026-09-03) · `OBSTACLES_CONTACT_DIST` (2026-08-31) · Dos bay-exits (2026-09-02) · Migración a Go/NATS |
| `riesgo-abierto` | `MAX_STEERING_RATE` sin medir (2026-09-02) · Pass-side, convención bajo sospecha (2026-09-03) · Pull-down `LPWM` pendiente |
| `metodología` / `pruebas` | Barridos no comparables entre corridas concurrentes (2026-09-03) |
| `seguridad` | GPIO flotante del puente H, tres capas (2026-08) · Pull-down `LPWM` pendiente |

---

## 1. Mobility and Mechanical Design

### [2026-08-27] El giro a tope de la dirección es físicamente irreal en simulación

**Categorías:** `mecánica` `dirección` `simulación`

**Contexto/restricción:** el simulador permitía radios de giro de dirección extremos
(virtualmente hasta ~8 mm de radio de giro), muy por debajo de lo que un servo Ackermann real
puede lograr.

**Opciones consideradas:** (a) seguir tuneando el software de navegación contra ese límite de
simulación tal cual, o (b) medir el ángulo de rueda real del chasis y limitar el modelo al
valor físico.

**Qué hicimos:** medimos el ángulo de giro máximo real del chasis y lo usamos como límite
duro tanto en simulación como en el controlador.

**Por qué:** cualquier controlador tuneado contra un límite de dirección irreal producirá
comportamiento que no es reproducible en el robot físico - el criterio de "reproducibilidad"
exige que las decisiones de ajuste tengan base física, no solo numérica.

**Resultado medido:** el límite de dirección pasó a estar acotado por la medición física del
chasis, no por el valor por defecto del simulador.

**Referencia:** nota interna `sim_full_lock_steering_is_unphysical_2026_08_27`.

---

### [2026-09-02] El `MAX_STEERING_RATE` de software es una suposición, no una medición

**Categorías:** `mecánica` `dirección` `riesgo-abierto`

**Contexto/restricción:** el limitador de velocidad de giro de la dirección (que actúa tanto
como limitador de software como parámetro de "slew" en simulación) estaba fijado en 1.2
rad/s, un valor asumido, no medido en el servo real (~35 kg·cm).

**Opciones consideradas:** dejarlo como estaba (arriesgando que el ajuste del zigzag ("weave")
en simulación esté ajustado a un actuador más rápido de lo real) vs. medirlo en banco.

**Qué hicimos:** documentamos explícitamente que el valor es no medido y que probablemente el
servo real ronda ~5 rad/s; quedó marcado como pendiente de banco de pruebas antes de seguir
tuneando el comportamiento de zigzag ("weave") con ese parámetro.

**Por qué:** el mismo parámetro alimenta dos sistemas (limitador real y física de
simulación); si el valor es incorrecto, cualquier conclusión sobre causas del "weave" en
simulación queda contaminada.

**Resultado medido:** N/A todavía - pendiente de medición en banco. Se documenta como riesgo
abierto, no como hallazgo cerrado (ver también sección 4, pensamiento sistémico).

**Referencia:** nota interna `max_steering_rate_is_unmeasured_2026_09_02`.

---

### [2026-08-29] `counts_per_rev` real del encoder es 60, no 86

**Categorías:** `motor` `calibración`

**Contexto/restricción:** todo el modelo de velocidad (feedforward, tope de velocidad,
odometría) dependía de la constante de cuentas por revolución del encoder, que estaba fijada
en 86 sin verificación en hardware.

**Opciones consideradas:** confiar en el valor de datasheet/estimado (86) o medirlo
directamente en el robot físico.

**Qué hicimos:** medimos en vivo y confirmamos 60 cuentas/rev.

**Por qué:** un valor de encoder incorrecto sesga toda la cadena de control de velocidad -
mejor corregirlo en la fuente que compensarlo aguas abajo con ganancias ad-hoc.

**Resultado medido:** corrección propagada a feedforward (`drive_feedforward_is_affine`) y al
tope de velocidad real del chasis (ver siguiente entrada).

**Referencia:** nota interna `encoder_counts_per_rev_is_60_not_86_2026_08_29`.

---

### [2026-08-30] El techo de velocidad real es ~0.58 m/s, no ~0.45 m/s

**Categorías:** `motor` `calibración`

**Contexto/restricción:** tras corregir `counts_per_rev`, el "techo" de velocidad de 0.45 m/s
que se usaba como referencia quedó identificado como un artefacto de la constante de encoder
vieja, no como un límite físico real del chasis.

**Qué hicimos:** re-caracterizamos la curva duty→rpm con la constante corregida y
recalculamos el techo real: ~0.58 m/s a `max_duty=0.5`, hasta ~0.9 m/s en lazo abierto a
máximo duty.

**Por qué:** usar un techo subestimado deja rendimiento sobre la mesa (velocidad de crucero
más baja de la necesaria) y puede sesgar el ajuste del controlador de velocidad.

**Resultado medido:** techo actualizado a 0.58 m/s; también se corrigió un límite de
velocidad del localizador (`max_speed_mps`) que congelaba silenciosamente la pose por encima
de 0.25 m/s, muy por debajo del nuevo techo real (subido a 0.60 m/s).

**Referencia:** `drive_ceiling_058_supersedes_045_2026_08_30`,
`localizer_speed_gate_froze_pose_2026_08_29`. Commit relacionado:
`654e57e6 fix(config): max_speed_mps is the measured 0.58, not the feel-based 1.0`.

---

### [2026-08-27] Cambio de motor de tracción y por qué obligó a recalibrar todo el lazo de control

**Categorías:** `motor` `calibración` `decisión-sistema`

**Contexto/restricción:** se reemplazó el motor de tracción original por un motor nuevo (REV
HD Hex). El motor retirado había sido caracterizado en banco el 2026-07-25
(`counts_per_rev`/`max_rpm` del encoder, ganancias PID), y esa caracterización quedó
obsoleta de golpe con el cambio físico.

**Opciones consideradas:** (a) mantener la caracterización vieja y solo ajustar ganancias a
ojo hasta que "se sintiera bien", o (b) tratar el cambio de motor como un evento que invalida
toda la cadena de calibración aguas abajo y volver a medir desde cero.

**Qué hicimos:** primero, como medida de seguridad inmediata, se limitó el duty cycle en lazo
cerrado a un techo de 50% (`EncoderConfig.max_duty`, independiente de la calibración vieja de
encoder), porque el motor nuevo midió en hardware ~1 m/s, muy por encima de lo que la
matemática de RPM (basada en el motor retirado) y el límite `drivetrain.max_speed_mps=0.234`
(un valor "adivinado" que solo alimentaba navegación/simulación) podían anticipar. Con el
robot ya protegido de una sobrevelocidad peligrosa, se rehizo la caracterización completa:
`counts_per_rev` pasó de 676.0 a 86.0 y `max_rpm` de 42.5 a 123.0, derivados de datos de
banco (velocidades comandadas sostenidas, medidas con cinta métrica, convergencia
objetivo y medido en estado estable, adelante y reversa, con acuerdo dentro de ~3%). Las
ganancias PID (`pid_kp`/`pid_ki`) se reescalaron proporcionalmente (~7.9x, redondeado a 8)
porque bajar `counts_per_rev` hace que la lectura de RPM sea mucho más sensible por unidad de
movimiento real de la rueda, y las ganancias viejas (dimensionadas para la escala menos
sensible) producían oscilación visible una vez aplicada la corrección.

**Por qué:** un cambio de motor no es un ajuste de parámetro - invalida silenciosamente cada
constante que dependía de las características físicas del motor anterior (relación
encoder/vuelta, RPM máximo, ganancias de control). Tratar el duty-cap como medida de
contención *antes* de recalibrar, en vez de recalibrar bajo presión con el robot ya
acelerando fuera de rango, fue la decisión de seguridad correcta.

**Resultado medido:** la nueva caracterización quedó marcada explícitamente como
**provisional** (derivada de temporización, no de conteos crudos) hasta reverificarla con
`scripts/hardware/calibrate_encoder.py`; el reescalado de ganancias PID también quedó
marcado como **no verificado en vivo** en esa misma sesión (batería desconectada antes de
poder probarlo). Se agregó un registro de depuración por cada paso de PID
(`target_rpm`/`measured_rpm`/`duty`) específicamente para poder *ver* la oscilación en vez de
inferirla de síntomas externos - esa instrumentación fue lo que hizo visible tanto la
oscilación nueva como la saturación cerca de `max_duty` que tenía la calibración vieja.

**Resultado final (nota de mantenimiento):** una sesión posterior (2026-08-29) volvió a medir
y encontró que el valor real de `counts_per_rev` es 60, no 86 - es decir, incluso esta
recalibración "post motor nuevo" quedó superada por una medición más precisa (ver entrada
anterior). Se documenta ambas para dejar clara la cadena completa: motor viejo (676/42.5) →
motor nuevo, primera recalibración por temporización (86/123, provisional) → medición directa
en vivo (60 cpr, confirmada). Es un buen ejemplo de **iteración honesta**: cada paso se marcó
con su nivel real de confianza en vez de presentarse como definitivo.

**Referencia:** commits `a30fe05c`/`51c9f914 fix(robot): cap closed-loop drive duty at 50%
after motor swap`, `fe07b2cf`/`4f254ef4 fix(robot): recalibrate encoder for new drive motor,
add PID-step debug log`, `bbd6803f feat(hardware): raise the speed ladder for the REV HD Hex
motor`. Ver también `encoder_counts_per_rev_is_60_not_86_2026_08_29` y
`motor_servo_bench_session_2026_08_28`.

---

### [2026-08-27→28] El puente H se recableó y el firmware de calibración de servo tenía un fallo en los nombres de carpeta

**Categorías:** `puente-h` `eléctrico` `servo` `mecánica`

**Contexto/restricción:** junto con el cambio de motor, el puente H (BTS7960) se recableó
físicamente, lo que invirtió el sentido eléctrico esperado de la señal `drive.reversed`
(commits `8e83945c`/`8fccb633 fix(robot): flip drive.reversed for the BTS7960 rewire`) - un
recordatorio de que un cambio de cableado físico también es una entrada que debe
propagarse al software, no solo un detalle de electrónica.

**Qué hicimos (fallo del servo, hallado por separado):** se descubrió que la capa de
configuración para el servo de dirección (`range_deg=270`, para el servo Hiwonder de 35 kg
realmente instalado) nunca se había aplicado en hardware. La función que carga las capas por
nombre de perfil (`_profile_overlay_paths()`) construye la ruta a partir del nombre de
`VTITAN_HARDWARE_PROFILE` y **si la carpeta no existe, la omite en silencio, sin error ni
señal**. La carpeta estaba nombrada `servo270`, que nunca coincidió con el nombre real del
perfil activo (`270deg-hiwonder-35kg`) - así que cada comando de servo estuvo usando
matemática de ancho de pulso para un servo de **180°**, en un servo físico de **270°**, desde
que se instaló.

**Por qué importa:** este es un cambio de comportamiento real una vez desplegado, no solo una
corrección cosmética - el mismo ángulo comandado ahora mapea a un swing de ancho de pulso
distinto (ej. 67.5° pasa de 2250 µs a 2000 µs), así que la dirección se siente **menos
agresiva por grado** en ángulos intermedios que antes. Se documentó explícitamente como algo
que requiere reverificación en hardware, no solo confiar en la corrección de código.

**Resultado medido:** corrección de nombre de carpeta aplicada; además se corrigieron
referencias obsoletas a "servo270" en docs, Taskfile y texto de ayuda del generador Go, y un
error en `docs/servo-comparison-180-vs-270.md` que indicaba reiniciar el servicio equivocado
(`vtitan-pi5.service` en vez de que `ackermann_motor_node` corre en el Pi Zero).

**Referencia:** commit `3a7d7cbe fix(robot): rename servo270 profile folder to match active
profile name`.

### Lo que falta de mecánica

Nos falta material gráfico. Hace falta un diagrama dimensionado del chasis (30 × 19.4 cm,
1.3 kg) y de la disposición de las ruedas (7 cm de diámetro), tomando `robot.toml` como fuente
de verdad para las cotas. Hacen falta también fotos o el CAD de la dirección Ackermann que
fabricamos, y conviene decir de forma explícita que **no** es el chasis LEGO Bugatti Bolide,
porque a simple vista se parecen y la diferencia es justo lo que fabricamos nosotros.

Queda pendiente además escribir la sesión de banco de motor y servo del 28 de agosto como una
iteración con datos de antes y después de la retonificación del PID. Esa entrada debe incluir
la reverificación en vivo del reescalado de ganancias, que sigue sin hacerse.

---

## 2. Power and Sensor Architecture

### [2026-08-21] Los perfiles de hardware son por componente, no globales

**Categorías:** `arquitectura` `software`

**Contexto/restricción:** el sistema soporta simulación y hardware real, y cada
componente (LIDAR, cámara, motores) necesita saber si está hablando con hardware real o con
un simulador.

**Qué hicimos:** implementamos perfiles de hardware **por componente** en vez de un único
indicador global, y documentamos que un perfil sin activar no tiene efecto aunque el archivo esté
editado.

**Por qué:** un indicador global obliga a que todo el sistema esté en modo simulación o todo en modo
hardware; en la práctica necesitamos combinaciones (ej. LIDAR real + resto simulado) durante
la puesta a punto.

**Resultado medido:** N/A (decisión de arquitectura); efecto colateral documentado: si el
perfil no está seteado, el import falla en vez de fallar en silencio con comportamiento
incorrecto.

**Referencia:** `hardware_profiles_are_per_component_2026_08_21`,
`hardware_profile_not_activated_2026_08_20`.

---

### [2026-08-22] Recalibración de la distancia de giro por el desplazamiento físico del LIDAR

**Categorías:** `sensores` `lidar` `calibración`

**Contexto/restricción:** el LIDAR no está montado en el centro geométrico del chasis, así
que las distancias de holgura para las maniobras de giro medidas "desde el LIDAR" no
coincidían con el espacio real disponible para el chasis.

**Qué hicimos:** recalibramos las distancias de holgura de giro teniendo en cuenta el desplazamiento
físico del sensor respecto al centro del chasis.

**Por qué:** ignorar ese desplazamiento produce un sesgo sistemático - el robot "cree" tener más o
menos espacio del que realmente tiene, según el punto del giro.

**Resultado medido:** constante `TURN_CLEARANCE_M` anterior quedó invalidada por el cambio de
calibración.

**Referencia:** `lidar_offset_clearance_recalibration_2026_08_22`.

---

### [2026-08-31] El montaje invertido del LIDAR necesita un espejo, no una rotación de 180°

**Categorías:** `sensores` `lidar` `software`

**Contexto/restricción:** el LIDAR está montado invertido en el chasis. La corrección
"ingenua" para un montaje invertido es sumar 180° al ángulo reportado.

**Opciones consideradas:** corregir con `+180°` (rotación) vs. corregir invirtiendo el eje
angular (espejo).

**Qué hicimos:** identificamos que un montaje invertido requiere **espejar** el ángulo
decodificado, no rotarlo, y corregimos el controlador en la migración a Go
(`2ebe7d7d fix(lidar): mirror decoded angles for an inverted mount, not rotate`).

**Por qué:** rotar 180° preserva la orientación (quiralidad) del barrido, mientras que un
montaje invertido físicamente invierte esa quiralidad - son transformaciones distintas y una
sustituye mal a la otra. Usar la incorrecta produce lecturas de ángulo consistentemente mal
mapeadas, especialmente notorio en los sectores laterales/traseros.

**Resultado medido:** corregido en la implementación Go (`robot-go`); la implementación
Python (`LidarYawOffsetRad()` / `sector.go` / `_LIDAR_YAW_OFFSET_RAD`) quedó identificada como
pendiente de corregir en conjunto para no divergir entre ambas.

**Referencia:** `lidar_inverted_mount_needs_mirror_not_rotation_2026_08_31`.

---

### [2026-08-22] Un indicador de localización por LIDAR estaba invertido en su semántica

**Categorías:** `sensores` `lidar` `software`

**Contexto/restricción:** el indicador `use_lidar_localization` se interpretaba de forma
contraintuitiva: `True` hacía el problema de localización **más difícil**, no más fácil, al
contrario de lo que su nombre sugiere.

**Qué hicimos:** documentamos explícitamente el comportamiento real del indicador para evitar que
futuras pruebas lo usaran con la semántica intuitiva incorrecta.

**Por qué:** un indicador con semántica invertida y sin documentar es una fuente silenciosa de
errores de experimento - cualquier A/B hecho asumiendo la semántica "obvia" mide lo contrario
de lo que cree medir.

**Referencia:** `use_lidar_localization_reads_backwards_2026_08_22`.

---

### [2026-08] El puente H (BTS7960) mantuvo el motor girando a máxima velocidad en reversa al arrancar - historia de un fallo de seguridad en tres capas

**Categorías:** `puente-h` `eléctrico` `seguridad`

**Contexto/restricción:** el controlador de motor emplea un puente H BTS7960 controlado por GPIO
(habilitación `R_EN`/`L_EN`) y PWM (`RPWM`/`LPWM`). Se descubrió en hardware que, en ciertas
condiciones, el motor arrancaba girando a máxima velocidad en reversa sin que el software de
control hubiera comandado eso - un fallo de seguridad grave, no cosmético.

**Capa 1 - secuenciación de habilitación (`f0fc617b`/`d8791b89`):** se identificó que
`R_EN`/`L_EN` podían quedar en HIGH (habilitados) antes de que los canales PWM estuvieran
confirmados en 0% duty. Se corrigió para que ambos canales PWM se confirmen en 0 antes de
habilitar el puente H - invariante que después se preservó explícitamente al portar el controlador
a Go (`511d0a00`/`01e7cb78`), con pruebas dedicadas
(`TestController_Connect_NeverEnablesBeforeZeroingEitherChannel`,
`TestController_SetSpeed_NeverBothChannelsNonzero`) para que nunca se reintroduzca en
silencio.

**Capa 2 - flotación de GPIO al arrancar (`bd60aa82`/`f5797c6a`):** con la capa 1 corregida,
seguía pasando que el motor giraba a máxima reversa **antes** de que el nodo de control
siquiera arrancara. Se midió en hardware que un pull-down *pasivo* de GPIO
(`gpio=...=pd`) no cierra la ventana de arranque: el propio nivel-shifter del BTS7960 tiene
un pull-up interno de ~10 kΩ por canal, que le gana al pull-down interno del SoC (mucho más
débil, ~50 kΩ). Ese hueco dejaba que el motor de tracción girara a máxima velocidad en
reversa antes de que `connect()` del nodo del motor llegara a ejecutarse. La solución que sí
funcionó, verificada en hardware con el servicio systemd deshabilitado: manejar esos 3 pines
como salida activa forzada a bajo (`gpio=...=op,dl`) desde firmware, confirmando en un
arranque limpio que los pines se mantienen bajos desde el arranque mismo, y que `gpiozero`
los puede seguir reclamando después sin conflicto cuando el servicio arranca.

**Capa 3 - flotación de GPIO al detener el servicio (`a55612b4`/`96020000`):** un
`systemctl stop` normal (o cualquier crash/OOM-kill) libera los descriptores de archivo GPIO
que `gpiozero` retiene para `LPWM`/`R_EN`/`L_EN`, y en el momento en que se liberan, el
pull-up del nivel-shifter los vuelve a arrastrar a HIGH - reproducido en vivo, motor a máxima
reversa en un `stop` rutinario, no solo en arranque en frío. La corrección de `config.txt`
(`op,dl`) de la capa 2 solo cubre la ventana de arranque, porque es una instrucción de
firmware de una sola vez, no un valor por defecto que se reaplique al liberar el pin. Se usó
`ExecStopPost` de systemd (que se ejecuta después de que el proceso principal termina por
cualquier razón que systemd conozca) para volver a forzar esos pines a bajo, cerrando la
mayor parte del hueco sin tocar el ciclo de vida/apagado propio del nodo.

**Por qué importa (nivel 4 y no aún nivel 6):** el propio equipo documentó explícitamente que
esta cadena de tres capas **no cubre todos los casos** - una pérdida de energía cruda
seguiría dejando los pines flotando, y la mitigación completa (una resistencia pull-down
física en `LPWM`) **todavía no está instalada**. Esto es exactamente el tipo de "modo de
falla identificado, mitigación parcial, riesgo residual explícito" que el criterio de
pensamiento sistémico busca - documentado aquí y repetido en la sección 4.

**Referencia:** `gpio_boot_float_full_speed_motor_2026_08_27`,
`gpio_float_reverse_on_script_to_service_handoff_2026_08_28`,
`bts7960_connect_ordering_bug_2026_08_28`. Commits: `f0fc617b`, `d8791b89`, `bd60aa82`,
`f5797c6a`, `a55612b4`, `96020000`, `9748b30d fix(robot): template the BTS7960 GPIO pins into
the systemd stop-hook`, `873a1232 fix(robot): re-drive BTS7960 GPIO pins low on driver
disconnect`.

---

### [fecha no registrada] El diseño de cableado del puente H cambió de PWM compartido a RPWM/LPWM independientes

**Categorías:** `puente-h` `eléctrico`

**Contexto/restricción:** el diseño original de `driver.py`/`config.py` alimentaba el puente
H con un PWM compartido hacia un demultiplexor externo, en vez de usar las dos señales
`RPWM`/`LPWM` del BTS7960 de forma verdaderamente independiente.

**Qué hicimos:** se migró a señales `RPWM`/`LPWM` independientes (commit `6aee1ac2`), y
posteriormente se detectó que la documentación (`.env.example`, comentarios de
`bts7960.toml`, el docstring de `ackermann_motor_node`, y los tests) seguía describiendo el
diseño viejo de demux compartido y los pines `R_EN`/`L_EN` intercambiados.

**Por qué:** documentación desincronizada del cableado real es peligrosa específicamente en
un componente de seguridad (control de motor) - alguien depurando con la documentación vieja
llegaría a conclusiones incorrectas sobre qué pin hace qué.

**Resultado medido:** confirmado en vivo en hardware (registros del Pi Zero): `forward=13`,
`reverse=26`, `r_en=6`, `l_en=5`, conecta y activa limpiamente con el diseño independiente.

**Referencia:** commit `dcec0b93 docs(robot): catch up BTS7960 env/toml/test docs to
independent RPWM/LPWM`.

---

### [fecha no registrada] La cámara estaba montada invertida y el indicador de software nunca se activó

**Categorías:** `sensores` `cámara` `software`

**Contexto/restricción:** la cámara está físicamente montada boca abajo en el chasis. La
lógica de rotación y volteo para compensar un montaje invertido ya existía en el controlador, pero el
indicador `camera_inverted` estaba en `false` por defecto en ambos TOML de configuración del
controlador.

**Qué hicimos:** activamos `camera_inverted` por defecto.

**Por qué:** este no era solo un problema estético de imagen volteada - el frame sin rotar
estaba **espejando silenciosamente izquierda/derecha**, lo cual afectaba directamente la
detección del lado de paso de señales (pass-side), uno de los fallos más caros del desafío
(ver Criterio 3 y 4). Es un buen ejemplo de cómo un defecto de montaje mecánico/óptico se
manifiesta como un fallo de "lógica" aguas abajo si no se corrige en la fuente.

**Resultado medido:** corrección directa del defecto de imagen espejada; efecto colateral
positivo documentado explícitamente en la detección de pass-side.

**Referencia:** commit `78c1692e fix(robot): enable camera_inverted for upside-down camera
mount`.

### Arquitectura de sensores

- **Cómputo:** NPU Hailo-8 para aceleración de inferencia.
- **Cámara:** Raspberry Pi Camera Module 3 Wide, 102° de campo de visión horizontal.
- **LIDAR:** Slamtec C1 (modo *dense*; se identificó y corrigió un mismatch de modo de
  escaneo que producía lecturas de rango erróneas - `lidar_c1_dense_mode_scan_mismatch`).
- **IMU:** BNO085.
- **Chasis:** Ackermann personalizado, no un kit LEGO.

### Lo que falta de potencia y sensores

El hueco más grande de este criterio es el presupuesto de potencia: hoy no lo tenemos
calculado formalmente, solo conocemos el tope de velocidad del tren motriz. Hay que medir el
consumo en pico y en reposo, y desglosarlo por rama sobre un diagrama de cableado real que vaya
de la batería a los reguladores y de ahí a motores y electrónica.

El método de calibración de la cámara y de la IMU sí existe, pero vive disperso en notas de
sesión. Falta consolidarlo en un procedimiento paso a paso que otro equipo pueda seguir.

---

## 3. Software Architecture and Obstacle Strategy

> **⚠️ Salvedad de medición (agregado 2026-09-05):** el commit `ecc3cdef fix(sim): only score a
> sign pass the scorer actually watched happen` corrigió el sistema de puntaje de pass-side -
> antes, ser *colocado* más allá del radio de una señal dentro de 1.20 m contaba como paso por
> el lado incorrecto, aunque el robot nunca hubiera hecho esa aproximación. **Todas las cifras
> de Obstacles citadas en esta sección (y en la sección 4) fueron medidas antes de esa
> corrección y no son directamente comparables con mediciones posteriores.** El efecto conocido
> es que corridas que antes terminaban temprano ahora sobreviven hasta chocar contra algo real:
> las colisiones de control suben de 5 a 14. Antes de citar cualquiera de estos números como
> estado actual, volver a medir con el evaluador corregido.

### [2026-09-02] Dos maniobras de salida de bahía, decididas por el modelo de contacto físico

**Categorías:** `software-navegación` `obstáculos` `parking` `decisión-sistema`

**Contexto/restricción:** existían dos implementaciones candidatas para la maniobra de salida
de la bahía de estacionamiento (el "bay exit"), cada una con éxito casi perfecto (254/256)
bajo *distintos* modelos de contacto de simulación - la que ganaba dependía enteramente de
si el modelo de contacto permite "deslizar" contra la pared o la hace "trincar" (ratchet).

**Opciones consideradas:** (a) `0f277aa1` - salida que fija ("latches") el lado abierto y
mantiene el steering, compatible con un modelo de contacto que trinca; (b) `ae15ee3e` -
salida cíclica de 26°, con steering fijado antes de avanzar, compatible con un modelo de
contacto que desliza.

**Qué hicimos:** enviamos `ae15ee3e` (modelo *slide*) como comportamiento por defecto.

**Por qué:** determinamos primero, por separado, que el simulador **nunca desliza** contra
una pared (una limitación de fidelidad conocida - a 20° de incidencia el progreso es 56 veces
menor que con deslizamiento real), y por otro lado que el reglamento WRO 2026 establece que
**tocar los límites del cajón de estacionamiento anula todos los puntos de parking** - por lo
tanto cualquier maniobra que dependa de contacto sostenido con la pared para "funcionar" en
sim está, de hecho, modelando una infracción de reglamento, no una estrategia válida. Se
descartó explícitamente usar el modelo `--solid-walls` (que premia el contacto) porque haría
que el simulador puntuara como éxito algo que en la pista real sería descalificación.

**Resultado medido:** ambas maniobras llegan a 254/256 en simulación bajo su modelo
respectivo; la decisión de cuál enviar no se tomó por la métrica de simulación (empatada)
sino por cuál corresponde a comportamiento legal según el reglamento.

**Referencia:** `bay_exit_solved_by_hold_steer_2026_09_02`,
`wro_2026_scoring_parking_and_bay_start_2026_09_03`. Commits:
`0f277aa1`, `544958be`, `ae15ee3e feat(nav): ship the cycle bay exit`.

Este es un ejemplo directo de razonamiento nivel 6 para el **Criterio 4** también: la
decisión no se tomó por el resultado en simulación, sino por una restricción externa
(reglamento) que la simulación por sí sola no podía revelar.

---

### [2026-08-28] La maniobra de escape no debe avanzar hacia una pared que el robot no puede ver

**Categorías:** `software-navegación` `obstáculos` `sensores`

**Contexto/restricción:** la maniobra reactiva de "escape" (usada al detectar contacto o
proximidad de obstáculo) podía comandar avance hacia adelante incluso cuando el cono frontal
del LIDAR no tenía lecturas válidas (retorno nulo), lo cual el software interpretaba
erróneamente como "espacio libre" en lugar de "sensor degradado".

**Qué hicimos:** distinguimos explícitamente "cono sin retorno de LIDAR" (sensor degradado)
de "cono con retorno indicando espacio libre", y bloqueamos el escape hacia adelante en el
primer caso.

**Por qué:** tratar la ausencia de datos como "vía libre" es la peor interpretación posible
del silencio de un sensor - es exactamente la condición donde más probable es que haya un
obstáculo demasiado cerca o en un ángulo ciego.

**Resultado medido:** la corrección llevó la maniobra de 0 a 2 vueltas completadas en la
prueba de referencia usada en ese momento.

**Referencia:** `obstacles_escape_maneuver_forward_into_wall_2026_08_28`,
`obstacles_lidar_no_return_reads_as_clear_2026_08_28`. Commits:
`bda6b10c fix(nav): don't escape FORWARD into a wall the robot cannot see`,
`d638800b fix(nav): an unreadable forward cone is a degraded sensor, not open road`.

---

### [2026-09-01] El error de "pass-side" (lado de paso de señal) es de seguimiento, no de ruteo

**Categorías:** `software-navegación` `obstáculos`

**Contexto/restricción:** el robot ocasionalmente pasaba una señal (poste/obstáculo) por el
lado incorrecto, lo cual en el reglamento WRO no solo vale 0 puntos sino que **termina la
ronda** (regla 9.24.5) - el fallo más caro posible en este desafío.

**Opciones consideradas:** la hipótesis inicial era que el planificador de ruta (el "enrutador")
elegía el lado incorrecto (fallo de decisión o de enrutado). La hipótesis alternativa era que el
enrutador decidía bien pero el controlador de seguimiento de trayectoria no ejecutaba esa
decisión con suficiente margen.

**Qué hicimos:** medimos: el plan de ruta acertaba el lado correcto en 642 de 642 casos, con
un margen de diseño de 5.6 cm - es decir, el enrutador **no** era el problema.

**Por qué:** esto redirigió todo el esfuerzo de esta rama de trabajo desde "corregir el
enrutador" (que ya estaba bien) hacia "reducir el error de seguimiento" que consumía el
margen de 5.6 cm en la ejecución real.

**Resultado medido:** confirmó y superó una hipótesis anterior ("fallo de enrutado") y otra más
antigua ("puntuado sobre creencia, no verdad de terreno") que ya habían sido corregidas por
separado. Trabajo posterior (`sign_pass_error_is_outward_bias`) encontró que el error
residual es un **sesgo sistemático hacia afuera** (84% de los casos, +7.53 cm vs. 5.6 cm de
margen), no dispersión aleatoria - lo cual implica que es corregible ajustando la puntería
del controlador, no agregando más margen de seguridad.

**Referencia:** `obstacles_pass_side_is_tracking_not_routing_2026_09_01`,
`sign_pass_error_is_outward_bias_not_scatter_2026_09_03`,
`pass_side_is_a_round_ender_and_convention_may_be_wrong_2026_09_03`.

---

### [2026-08-31] `OBSTACLES_CONTACT_DIST` a 0.05: mejora medible, con un costo aceptado

**Categorías:** `software-navegación` `obstáculos` `decisión-sistema`

**Contexto/restricción:** el umbral de distancia de contacto que dispara la maniobra de
escape reactivo (`OBSTACLES_CONTACT_DIST`) estaba en un valor que dejaba muchos casos sin
activar el escape a tiempo.

**Qué hicimos:** movimos el umbral a 0.05 m y medimos el efecto en una corrida de referencia
de 256 casos: vueltas completadas subieron de 68 a 434 (agregado), vueltas-por-run ≥3 subieron
de 11 a 82, y los timeouts bajaron de 126 a 48.

**Por qué:** un umbral más sensible detecta contacto inminente antes, dando más tiempo a la
maniobra de escape para ejecutarse con margen.

**Resultado medido:** mejora neta clara en finalización de vueltas y reducción de timeouts,
**pero** con una regresión aceptada conscientemente: los casos de "pass-side" (el fallo más
caro, ver entrada anterior) empeoraron de 82 a 122. Se documentó explícitamente como
trade-off pendiente de resolver por la vía del seguimiento, no revirtiendo este cambio.

**Referencia:** `obstacles_contact_dist_is_shippable_and_pass_side_is_next_2026_08_31`.
Commit: `1de8094d feat(nav): ship the Obstacles contact zone at 0.05`.

---

### [2026-09-02] El límite del corredor de escape lateral no puede excluir ningún rumbo a distancia de contacto

**Categorías:** `software-navegación` `obstáculos`

**Contexto/restricción:** se investigó si el corredor de escape lateral (la zona angular que
la maniobra de escape considera "segura" para desviarse) podía restringirse geométricamente
para evitar que el escape empujara al robot hacia el lado incorrecto de una señal.

**Qué hicimos:** medimos que, a la distancia de contacto real, el corredor de escape se
degenera - no existe ningún rumbo que el corredor pueda excluir sin también excluir rumbos
necesarios para escapar del obstáculo inmediato. El 82% de los escapes están impulsados por
esta "puerta" geométrica, y el 92% son de tipo lateral.

**Por qué:** esto **refutó** una hipótesis previa de que la capa reactiva de escape podía
usarse para reducir directamente los casos de "pass-side" - la geometría a esa distancia no
lo permite. Se documentó explícitamente como descripción, no como prescripción: la solución
tiene que venir de la capa de seguimiento (ver la entrada anterior), no de la capa reactiva.

**Referencia:** `escape_gate_corridor_degenerates_at_contact_range_2026_09_02`,
`lateral_escape_is_load_bearing_2026_09_02` (que además refutó la opción de simplemente
suprimir el escape lateral: hacerlo reduce pass-side de 122 a 80 pero dispara las colisiones
con señales de 4 a 97 - un trade-off mucho peor).

---

### [2026-09-02] Escalera de velocidad de Open elevada a 0.26/0.38/0.50 m/s

**Categorías:** `software-navegación`

**Contexto/restricción:** la escalera de velocidad del desafío Open (velocidades objetivo en
tramos sucesivos) estaba en valores conservadores; había margen de tiempo sin usar si se
pudiera subir sin introducir colisiones.

**Qué hicimos:** elevamos la escalera a 0.26/0.38/0.50 m/s y verificamos 639/640 corridas sin
colisión en el corpus de referencia. Se probó también 0.60 m/s como siguiente escalón.

**Por qué:** 0.60 m/s resultó ser un **acantilado de colisión** (salto brusco en tasa de
fallos, no degradación gradual) - se documentó explícitamente para que nadie vuelva a subir
la escalera a ese valor asumiendo que la mejora escala linealmente con velocidad.

**Resultado medido:** 639/640 sin colisión con la escalera actual; 0.60 m/s descartado por el
comportamiento de acantilado.

**Referencia:** `open_speed_ladder_raised_to_050_2026_09_02`. Commit:
`a98bcbc4 feat(nav): raise the Open speed ladder, and stop justifying it with a stale lap time`.

---

### Arquitectura del software

- Control de trayectoria por **pure pursuit** (`waypoint_controller`), con un
  navegador central (`core_navigator`) que separa planificación de ruta, seguimiento de
  trayectoria y una capa reactiva de escape ante contacto/proximidad.
- La estrategia de obstáculos emplea un enrutador de "lado de paso" (pass-side) separado del
  controlador de seguimiento - la investigación de este año confirmó que el enrutador es correcto
  y que el error reside en el seguimiento, lo cual cambió el foco del trabajo (ver arriba).
- Migración en curso de partes del software de Python (ROS2/rclpy) a Go (`robot-go`) sobre
  transporte NATS, en fases documentadas.

### Lo que falta de software

La máquina de estados completa, desde el arranque en la bahía hasta el estacionamiento pasando
por Open y Obstacles, solo existe hoy como código y notas dispersas. Falta dibujarla como
diagrama de flujo.

Falta también reunir en un solo lugar las métricas con las que ajustamos: tasa de colisión,
vueltas completadas, error de trayectoria y tasa de paso por el lado correcto. Ahora mismo cada
una se explica dentro de la entrada donde apareció, y no hay un solo sitio donde
consultarlas juntas.

---

## 4. Systems Thinking and Engineering Decisions

### [2026-09-03] El modelo de contacto por defecto se elige por reglamento, no por score de simulación

**Categorías:** `decisión-sistema` `parking` `reglamento`

**Contexto/restricción:** el reglamento oficial WRO 2026 establece que tocar los límites del
cajón de estacionamiento durante el parking **anula todos los puntos de parking**, no solo
resta puntos parciales.

**Qué hicimos:** revisamos todas las estrategias de bay-exit/parking que dependían de
contacto sostenido con la pared para funcionar en simulación (incluyendo una que lograba
254/256 usando `--solid-walls`) y las descalificamos como estrategia válida, aunque su
métrica de simulación fuera excelente. Se fijó el modelo de contacto **por defecto** (no el
de paredes sólidas) como el que se debe usar para cualquier evaluación futura.

**Por qué:** optimizar contra una métrica de simulación que no captura una regla de
descalificación del reglamento real produce un robot que "gana" en simulación y pierde todos
los puntos de parking en la pista real. Es el ejemplo más claro del proyecto de por qué la
simulación **no sustituye** la lectura del reglamento.

**Resultado medido:** N/A (es una decisión de política de ingeniería, no un experimento) -
pero desbloqueó la decisión correcta entre las dos maniobras de bay-exit empatadas en score
(ver Criterio 3, `ae15ee3e` vs. `0f277aa1`).

**Referencia:** `wro_2026_scoring_parking_and_bay_start_2026_09_03`.

---

### [2026-09-03] El "pass-side" es la falla más cara del desafío, y su convención de medición está bajo sospecha

**Categorías:** `decisión-sistema` `riesgo-abierto` `reglamento`

**Contexto/restricción:** el reglamento define el lado de paso correcto **relativo al sentido
de viaje** del robot (rojo a la derecha del sentido de marcha), pero el enrutador y el sistema
de puntaje interno del proyecto lo definen en términos **absolutos**. Ambas convenciones
coinciden en sentido antihorario, pero **difieren en sentido horario** - y la simulación
actual no puede exponer esta diferencia porque no varía el sentido de la pista de forma que
la distinga.

**Qué hicimos:** documentamos el riesgo explícitamente como **no resuelto** y decidimos
**no cambiar la convención unilateralmente** - se marcó como pendiente de confirmar con
criterio humano antes de modificar el enrutador o el evaluador de puntaje, precisamente porque un cambio
incorrecto aquí podría "arreglar" una convención que en realidad estaba bien, y esconder el
problema real.

**Por qué:** dado que un fallo de pass-side termina la ronda completa, el costo de cambiar la
convención incorrectamente es mucho mayor que el costo de dejarlo documentado y pendiente.
Es una decisión consciente de **no actuar todavía** ante incertidumbre de alto impacto, en
vez de "arreglar" algo sin estar seguros de qué lado del problema se está viendo.

**Resultado medido:** riesgo identificado, con nota explícita de "preguntar antes de
cambiar" - ejemplo directo de identificación de riesgo + mitigación por precaución, en vez
de por corrección de código.

**Referencia:** `pass_side_is_a_round_ender_and_convention_may_be_wrong_2026_09_03`.

---

### [2026-09-03] Los resultados de los barridos no son comparables entre sí si hay ediciones concurrentes

**Categorías:** `metodología` `pruebas`

**Contexto/restricción:** el mismo conjunto de pruebas ("arm") dio 21/256 fallos en una
corrida y 5/256 en otra corrida posterior, sin cambios de código propios que lo explicaran.

**Qué hicimos:** investigamos y encontramos que una sesión de desarrollo **concurrente**
estaba editando código de parking/simulación mientras la corrida de referencia se ejecutaba,
contaminando la comparación. Se estableció como regla de proceso: los resultados de un barrido
sólo son comparables **dentro de una misma invocación**, nunca entre corridas separadas en el
tiempo si hubo cambios concurrentes en el repositorio.

**Por qué:** sin esta regla, cualquier mejora o regresión aparente podría ser ruido de
concurrencia, no una señal real del cambio bajo prueba - un riesgo serio para la validez de
todas las conclusiones de ajuste del proyecto.

**Resultado medido:** se adoptó `--strip-parking` como forma de aislar resultados de
obstáculos del ruido introducido por trabajo paralelo en parking.

**Referencia:** `sweep_results_not_comparable_across_time_2026_09_03`.

---

### [en curso, desde ~2026-08] Migrar el software del robot de Python/ROS2 a Go/NATS: una decisión de arquitectura, no una reescritura por gusto

**Categorías:** `arquitectura` `decisión-sistema` `software`

**Contexto/restricción:** el software original se ejecuta sobre Python con ROS2 (rclpy) como
framework de nodos/transporte. El primer commit de esta migración lo describe explícitamente
como el **"reemplazo a largo plazo de ROS2"** (`258f1f9e feat(robot-go): scaffold Go/NATS
workspace for the long-term ROS2 replacement`), no como un experimento paralelo.

**Opciones consideradas:** quedarse en Python/ROS2 e invertir el esfuerzo de mejora en ese
sistema, frente a migrar de forma incremental a Go sobre transporte NATS, nodo por nodo (controladores de
hardware primero: BTS7960, LIDAR, IMU, botón, OLED; luego telemetría y máquina de estados
central), preservando invariantes de seguridad ya verificadas en la implementación Python
(ej. la secuenciación de habilitación del puente H, ver arriba, portada explícitamente con
pruebas dedicadas).

**Qué hicimos:** se ejecutó como migración **por fases**, documentada aparte
(`go_nats_migration_plan`), con cada controlador de hardware portado y re-verificado
individualmente contra el hardware real antes de avanzar (ej. el controlador de LIDAR en modo classic
primero, luego un controlador aparte para el modo Dense cuando se detectó que el modo de escaneo
importaba - ver `lidar_c1_dense_mode_scan_mismatch`). A la fecha de esta entrada, ~99
commits tocan el árbol `robot-go`, cubriendo transporte, controladores de todos los sensores y
actuadores, un supervisor de reinicio con notificación a systemd, y agregación de
telemetría.

**Por qué:** aunque las razones completas de negocio/ingeniería para elegir Go sobre seguir
invirtiendo en Python/ROS2 no están consolidadas en un solo documento (ver el cierre de esta
entrada), la ejecución en sí demuestra dos decisiones de sistema explícitas: (1) migrar
**incrementalmente por componente**, no todo de una vez, para poder validar cada pieza contra
hardware real de forma aislada; y (2) **no perder invariantes de seguridad ya ganadas** en el
camino - el ejemplo más claro es que la corrección de secuenciación de habilitación del puente H
(hallado y corregido en Python) se portó a Go con pruebas explícitas que codifican esa
invariante, en vez de confiar en que "el nuevo código no tendrá el mismo fallo".

**Resultado medido:** migración en curso, no completa (fases documentadas por separado,
casillas de progreso ya desactualizadas respecto al código real - hay que verificar contra
el árbol antes de citar como terminado cualquier fase específica).

**Referencia:** `go_nats_migration_plan`, `go_migration_progress` (a través de Fase 5),
`bts7960_connect_ordering_bug_2026_08_28`. Commit inicial: `258f1f9e`/`43662ee0`.

Falta escribir el razonamiento de **por qué Go y no seguir en Python/ROS2** (rendimiento,
tipado, consumo de recursos en la Pi Zero, concurrencia) como una entrada propia. Hoy vive
implícito en las decisiones de código, y no como la comparación explícita de trade-offs que un
hace falta para juzgar la decisión desde fuera.

---

### [riesgo abierto] El pull-down físico de `LPWM` sigue sin instalarse

**Categorías:** `puente-h` `eléctrico` `riesgo-abierto`

**Contexto/restricción:** la cadena de tres capas de mitigación del fallo de GPIO flotante del
puente H (ver Criterio 2) cierra la ventana de arranque en frío y la ventana de detención de
servicio, pero **no cubre una pérdida de energía cruda** (corte de batería, desconexión
abrupta) - en ese caso los pines vuelven a flotar y el pull-up interno del nivel-shifter
puede volver a llevar el motor a máxima reversa.

**Qué hicimos:** documentamos explícitamente que la mitigación completa (una resistencia
pull-down física soldada en `LPWM`) es conocida, está identificada como la solución que sí
cubre el caso restante, y **todavía no está instalada** físicamente en el robot.

**Por qué se documenta como riesgo y no se cierra en falso:** una mitigación de software de
tres capas se siente "casi completa", y es tentador reportarla como resuelta. Documentar
explícitamente el caso que **no** cubre evita asumir una seguridad
que no existe todavía - especialmente relevante para un componente que puede mover el chasis
a máxima velocidad sin comando del software.

**Referencia:** `gpio_boot_float_full_speed_motor_2026_08_27` (nota: título dice "NOT FULLY
FIXED").

---

### Restricciones del sistema identificadas explícitamente

- **Físicas:** techo de velocidad real 0.58 m/s, radio de giro mínimo real medido en banco,
  chasis 1.3 kg / 30×19.4 cm - todos usados como límites duros del controlador, no solo como
  datos de referencia.
- **De cómputo:** NPU Hailo-8 como aceleración obligatoria para mantener el pipeline de
  visión dentro de presupuesto de tiempo real.
- **De reglamento:** contacto con límites de parking anula puntaje completo (no parcial);
  pass-side termina la ronda - ambas restricciones, no técnicas sino normativas, condicionan
  directamente qué soluciones técnicas son aceptables aunque simulen bien.
- **De fidelidad de simulación:** el modelo de contacto no desliza contra paredes (ver
  Criterio 3) - cualquier ajuste que dependa de ese comportamiento está sobreajustado al
  simulador, no al robot real. Esta es una restricción epistemológica, no física: limita qué
  conclusiones de simulación se pueden confiar sin validación en hardware.
- **De seguridad eléctrica no cerrada del todo:** el fallo de GPIO flotante del puente H (ver
  Criterio 2) está mitigado en software en tres capas, pero el caso de pérdida de energía
  cruda sigue dependiendo de una resistencia física aún no instalada - restricción de
  hardware pendiente, no de software.

---

## 5. Reproducibility and GitHub Quality

El repositorio acumula 1365 commits (medido el 2026-09-09), todos con la convención
`tipo(alcance): mensaje` (`feat(nav):`, `fix(sim):`, `docs(readme):`, `perf(nav):`). El
`README.md` ronda los 110.000 caracteres. Ambas cifras superan con holgura los mínimos
cuantitativos del Apéndice C, aunque
ese no era el objetivo: la convención de commits se adoptó porque el historial es nuestra
primera fuente para reconstruir cuándo y por qué cambió algo, y varias entradas de esta
bitácora se escribieron leyendo `git log`.

La estructura separa `platform/robot` (el sistema de navegación en Python/ROS2), `robot-go`
(la migración en curso), `platform/shared` (la configuración y el ajuste que ambos comparten) y
`docs/`, con sus subcarpetas `internal`, `development`, `proposals`, `reference` y `schemes`.
Las dependencias se gestionan con `uv` en Python y con Pixi/RoboStack para ROS2, y las
operaciones habituales están encapsuladas en el `Taskfile` (`task sim:navigate...`,
`task rpi:stack`) para que nadie tenga que reconstruir de memoria un comando largo.

### Lo que falta de reproducibilidad

El historial es continuo pero no está etiquetado: hay que marcar con tags los hitos de
competencia (`v1.0` para el regional, `v2.0` para el final internacional) y acompañarlos de
notas de versión. Falta también un `tests.md` que explique el flujo de pruebas de forma
centralizada, porque hoy está repartido entre notas de sesión.

El CAD y los diagramas de cableado que faltan en los criterios 1 y 2 tienen que acabar
**dentro** del repositorio, no solo referenciados desde él: una documentación que apunta a
archivos que no viaja con ella no es reproducible.
