# Runbook offline: las dos guardias de sentido

Escrito 2026-09-12 para poder probar esto **sin internet y sin poder preguntar**.
Todo lo que necesitas está aquí: qué son las banderas, cómo se encienden, cómo
se despliega, cómo se verifica, cómo se mide despues de rodar, y con qué
criterio se decide. Al final hay una seccion de trampas que te van a morder si
no las lees.

Este fichero vive en `other/docs/runbook-guardias-de-sentido-offline.md`, versionado en el repo.

---

## 0. Resumen en cinco lineas

> **NADA DE ESTE RUNBOOK SE HA PROBADO EN HARDWARE.** Todo lo que hay
> aqui es codigo, tests y simulacion. Las dos banderas embarcan
> APAGADAS y no han rodado nunca en pista, ni el 2026-09-12 ni despues.
> Lo que la seccion 5.3 llama "midio POSITIVO" es un barrido de sim, y
> el sim NO reproduce la reversion de vuelta ni una sola vez: ese
> barrido mide COSTE, no beneficio. Trata cada numero de aqui como una
> prediccion sin confirmar.

Hay dos banderas nuevas, las dos **apagadas**, contra el fallo de conducir
entre 0.23 y 0.65 de vuelta **al reves**.

**Solo una sirve hoy.** `target_sense_gate` filtra la busqueda de target, esta
verificada alcanzable y midio POSITIVO en sim (ver seccion 5.3).
`sign_deform_sense_guard` es **INERTE en el arbol embarcado** y la medicion que
la motivo estaba MAL LEIDA: lee la seccion 1.3 antes de gastarle un minuto.

Y ninguna de las dos **recupera** un coche ya girado, porque en este arbol no
existe nada que gire un chasis por rumbo.

---

## 1. Las dos banderas

### 1.1 `target_sense_gate` (busqueda)

| | |
|---|---|
| TOML | `src/config/navigation/motion/pursuit.toml` |
| clave | `target_sense_gate` |
| Pydantic | `PurePursuitParams.TARGET_SENSE_GATE` en `src/python/shared/src/shared/config/navigation_tuning/motion.py` |
| codigo | `_agrees_with_path_sense` en `src/python/src/navigation/control/controllers/waypoint_controller.py` |
| embarca | `false` |

Rechaza un waypoint candidato al que el chasis solo llegaria **dando la vuelta
al bucle por el lado contrario**. El test es la proyeccion del rumbo
pose->candidato sobre la direccion de marcha del propio camino en ese
candidato.

Por que la cota de 1.0 m no lo cubria ya: un bucle cerrado tiene **dos**
tangentes en cada punto. `target_search_span_m` limita cuanto **camina** el
barrido, no el **sentido**. Un punto a un metro en el sentido equivocado sigue
estando a un metro y sigue estando en el semiplano delantero de un chasis ya
girado, que es todo lo que comprueba `x_local > 0`.

### 1.2 `sign_deform_sense_guard` (deformacion)

| | |
|---|---|
| TOML | `src/config/navigation/signs/sign_router.toml` |
| clave | `sign_deform_sense_guard` |
| Pydantic | `SignRouterParams.SIGN_DEFORM_SENSE_GUARD` en `.../navigation_tuning/signs.py` |
| codigo | `_bearing_agrees_with_path` en `src/python/src/navigation/core_navigator/navigator.py` |
| embarca | `false` |

Descarta la deformacion **solo** cuando empuja el punto de mira contra la
direccion del camino **y** el punto sin deformar si la respetaba. Un offset
lateral sobre un target **cercano** rota su rumbo sin limite: la magnitud esta
acotada, el angulo no.

Importante: solo se descarta la deformacion. El router **ya se ha llamado**, asi
que la contabilidad de engage/pass, el ingest de descubrimiento ciego y
`routed_sign_positions` (que lee la mascara de escape) quedan intactos.

### 1.3 RETRACTACION: la segunda guardia es inerte, y su medicion estaba mal leida

**No la enciendas esperando nada. Verificado 2026-09-12.**

`sign_lane_planner = true` y `sign_lane_suppress_deform = true` en
`sign_router.toml`, con `sign_lane_deform_fallback_m = 0.0`. Con eso
`navigator.step` **nunca** reasigna `steer_target` al punto deformado. Mi
guardia cuelga de una comprobacion de identidad sobre esa reasignacion, asi que
se evaluo en **0 ticks** de cuatro escenarios con camara y la bandera forzada a
`true`.

Y el numero que la motivo no dice lo que parecia. `sign_deform_magnitude_m` se
calcula **incondicionalmente**, aplicada o no, asi que bajo la supresion
embarcada es un **CONTRAFACTUAL**: el offset que el router *queria* aplicar. La
separacion de 0.554 m contra 0.031 m es una correlacion real y **no** identifica
la deformacion como causa. Lo que dice es que los targets de sentido erroneo
ocurren donde el router queria un offset lateral grande, y aqui quien actua
sobre ese deseo es el **CARRIL**, no la deformacion.

Misma clase de error que leer `min_lidar_range_m` como evidencia de contacto
cuando se calcula antes del filtro de rango. Lei un campo de debug sin
comprobar si refleja una accion aplicada.

**Lo que queda abierto:** el carril REEMPLAZA la coordenada lateral del target,
asi que es el candidato. Eso no esta medido. No lo inventes en pista.

El codigo de la guardia se queda para que la deformacion no se pueda reactivar
sin ella, pero encenderla hoy no hace nada.

---

## 2. Como se encienden

**TRAMPA, y es la que mas veces ha costado una sesion:** el TOML **gana** sobre
el default de Pydantic. Editar el campo Python es **inerte** si el TOML fija el
valor. Edita el TOML.

```bash
# encender las dos
sed -i 's/^target_sense_gate = false/target_sense_gate = true/' \
  src/config/navigation/motion/pursuit.toml
sed -i 's/^sign_deform_sense_guard = false/sign_deform_sense_guard = true/' \
  src/config/navigation/signs/sign_router.toml

# comprobar que quedaron puestas
grep -n "^target_sense_gate" src/config/navigation/motion/pursuit.toml
grep -n "^sign_deform_sense_guard" src/config/navigation/signs/sign_router.toml
```

Para apagarlas, lo mismo con `true` -> `false`. O `git checkout` de los dos
ficheros si no has commiteado nada mas en ellos.

No hay override por variable de entorno. El tuning es un arbol TOML.

---

## 3. Comprobar que la bandera LLEGA al codigo

Hazlo **siempre** antes de gastar un barrido o una ronda. Ya ha pasado que los
dos brazos de un A/B salieran byte-identicos porque el cableado estaba muerto.

```bash
cd src/python
VTITAN_HARDWARE_PROFILE='270deg-hiwonder-35kg,rev-hd-hex-motor-6000rpm' \
PYTHONPATH=. pixi run -e dev python - <<'PYEOF'
import shared.domain.enums  # noqa
from shared.config.navigation_tuning import NavigationTuning
from src.config.tuning_helpers import tuning_with_overrides
from src.navigation.control.controllers.waypoint_controller import WaypointController

base = NavigationTuning.load_default()
print("TOML dice:")
print("  pursuit(obst).TARGET_SENSE_GATE    :", base.pursuit.for_obstacles_challenge().TARGET_SENSE_GATE)
print("  pursuit(open).TARGET_SENSE_GATE    :", base.pursuit.for_open_challenge().TARGET_SENSE_GATE)
print("  sign_router.SIGN_DEFORM_SENSE_GUARD:", base.sign_router.SIGN_DEFORM_SENSE_GUARD)

print("el override de grupo alcanza el codigo que lo lee:")
t1 = tuning_with_overrides({"TARGET_SENSE_GATE": True}, group="pursuit")
for label, flag in (("obstacles", False), ("open", True)):
    c = WaypointController.from_tuning(t1, for_open=flag)
    print(f"  controller({label}).target_sense_gate = {c.target_sense_gate}")
t2 = tuning_with_overrides({"SIGN_DEFORM_SENSE_GUARD": True}, group="sign_router")
print("  sign_router.SIGN_DEFORM_SENSE_GUARD =", t2.sign_router.SIGN_DEFORM_SENSE_GUARD)

# CONTROL NEGATIVO: el mismo mecanismo sobre un campo que se sabe legible, para
# que un False arriba no se pueda confundir con un arnes roto.
t3 = tuning_with_overrides({"TARGET_SEARCH_SPAN_M": 7.77}, group="pursuit")
print("  CONTROL span=7.77 ->", WaypointController.from_tuning(t3, for_open=False).target_search_span_m)
PYEOF
```

Verificado 2026-09-12: las dos alcanzan, en Open y en Obstacles, y el control
negativo devuelve `7.77`.

---

## 4. Los tests

```bash
cd src/python
PYTHONPATH=. RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  pixi run -e dev pytest tests/unit/test_target_sense_gate.py -q -n 0
```

14 tests, pasaban 2026-09-12. Cubren las dos banderas en **OFF y en ON**, el
borde exacto de 90 grados, waypoints duplicados, el wrap del indice, y un test
que fija deliberadamente **el limite** de la guardia de busqueda (ver seccion
8.1).

Para la suite entera usa **la tarea de pixi**, no pytest a pelo:

```bash
cd src/python && pixi run -e dev test
```

`pytest.ini` lleva `-n 6 --dist loadscope` y el pin de RMW vive en la **tarea**
de pixi. Invocar pytest directamente pierde el pin y reporta fallos **falsos**.
Y `VTITAN_HARDWARE_PROFILE` debe estar **SIN definir** para la suite (al
contrario que para los diagnosticos de bag y sim, que la necesitan puesta).

---

## 5. Barridos en simulacion

### 5.1 Obstacles, una bandera

```bash
cd src/python
VTITAN_HARDWARE_PROFILE='270deg-hiwonder-35kg,rev-hd-hex-motor-6000rpm' \
PYTHONPATH=. pixi run -e dev python scripts/sim/diag_sign_router_flag_ab.py \
  --field TARGET_SENSE_GATE --group pursuit --sighted --seeds 8

VTITAN_HARDWARE_PROFILE='270deg-hiwonder-35kg,rev-hd-hex-motor-6000rpm' \
PYTHONPATH=. pixi run -e dev python scripts/sim/diag_sign_router_flag_ab.py \
  --field SIGN_DEFORM_SENSE_GUARD --group sign_router --sighted --seeds 8
```

16 escenarios x 8 semillas x 2 brazos = **256 runs**. El arnes corre los dos
brazos el mismo dia, sobre el mismo corpus y las mismas semillas, que es la
unica forma valida: **los resultados no son comparables entre momentos
distintos**. No compares con una tabla de una sesion anterior.

`--sighted` enciende la camara emulada. Sin ella mides vueltas, no enrutado:
las violaciones de lado de paso y las colisiones con señal **solo** significan
algo con camara.

### 5.2 Open, para saber si afecta a los dos retos

```bash
cd src/python
VTITAN_HARDWARE_PROFILE='270deg-hiwonder-35kg,rev-hd-hex-motor-6000rpm' \
PYTHONPATH=. pixi run -e dev python scripts/sim/diag_open_ab.py \
  pursuit.TARGET_SENSE_GATE=true --sample 128
```

Protocolo: **criba con 128, decide con 640**. Un 128 que sale plano no decide
nada; un 128 que sale mal si.

`sign_deform_sense_guard` **no** puede afectar a Open: Open no tiene señales, el
router no deforma nada y la rama no se ejecuta. Si un A/B de Open sobre esa
bandera sale con diferencia, es ruido de semilla o el arnes esta mal, no un
efecto. (Y en Obstacles tampoco hace nada, ver seccion 1.3.)

### 5.3 LO QUE YA MIDIO, 2026-09-12

256 runs por bandera, los dos brazos el mismo dia sobre el mismo corpus y las
mismas semillas.

| barrido | in_time | laps3 | collided |
|---|---|---|---|
| Obstacles CAMARA, `target_sense_gate` off -> on | 98 -> **100** | 99 -> **100** | 16 -> 16 |
| Obstacles CIEGO, `target_sense_gate` off -> on | 99 -> **102** | 99 -> **102** | 20 -> **17** |
| Obstacles CAMARA, `sign_deform_sense_guard` off -> on | 98 -> 98 | 99 -> 99 | 16 -> 16 |

`pass_side` y `timed` salieron 0 en los seis brazos.

**OPEN, 128 casos, `target_sense_gate`:** `128/128 ok` en los **dos** brazos,
ningun veredicto cambiado, ningun fallo nuevo, regla 1.3 cumplida 128/128 en los
dos. Tiempo de sim medio **-0.12 s** (110 de 128 casos **identicos**, 10 mas
rapidos, 8 mas lentos). Neutral y seguro: la puerta casi nunca ata en Open, que
es lo esperable porque ahi no hay señales y el coche no se da la vuelta.

Respuesta a "afecta solo a Obstacles": **afecta a los dos retos por codigo, pero
solo se nota en Obstacles.** En Open es inocua medida a 128.

Lectura honesta: la puerta de busqueda no empeora nada y mejora las tres
columnas en **ciego**, que es donde la busqueda de target manda porque no hay
carril de señal que la tape. Con camara el movimiento es de 1-2 runs sobre 128,
es decir dentro del ruido. Nada de esto es evidencia de que arregle la
reversion: el sim nunca la reproduce.

El brazo de la deformacion sale **byte-identico**, y eso es lo que hay que
esperar de una bandera inerte (seccion 1.3), no una refutacion de la idea.

---

## 6. Desplegar al Pi

```bash
# desbloquea la clave GPG PRIMERO: pinentry no puede preguntar desde un shell
# de herramienta, y el deploy firma
PI5_HOST=rpi-5-local HEF= bash src/python/scripts/provisioning/deploy-to-pi5.sh
```

Empaqueta los commits, reconstruye el workspace y reinicia **los dos**
servicios.

Hosts SSH: `rpi-5-local` (192.168.0.50), `rpi-5-direct` (192.168.251.2).

### Verificar por CONTENIDO, nunca por hash

Los hashes cambian con un rebase. Comprueba los valores **en el TOML del Pi**,
que es quien gana:

```bash
ssh rpi-5-local 'cd ~/vtitan && git log --oneline -3 && \
  grep -n "^target_sense_gate" src/config/navigation/motion/pursuit.toml && \
  grep -n "^sign_deform_sense_guard" src/config/navigation/signs/sign_router.toml'
ssh rpi-5-local 'systemctl --user is-active vtitan-robot vtitan-vision'
```

`pixi` no esta en el PATH de un SSH no interactivo.

**La WiFi se cae cuando el coche conduce**, asi que `ssh journalctl -f` esta
roto **por diseño**. Lee el bag del disco despues.

---

## 7. Medir despues de rodar

### 7.1 Traer los bags

```bash
RUNS_DIR=/d/Dev/active/projects/teamvoltimor/vtitan/data/live/runs \
  bash scripts/sync/pull-runs-from-pi5.sh run_20260913
```

Pasa **siempre** `RUNS_DIR` absoluto: el script deriva `REPO_ROOT` de su propia
profundidad, asi que una reestructuracion lo redirige **en silencio** fuera del
repo.

El Pi poda a 20 runs / 4 GB. No copies el archivo local hacia alla.

### 7.2 Vista general

```bash
cd src/python
VTITAN_HARDWARE_PROFILE='270deg-hiwonder-35kg,rev-hd-hex-motor-6000rpm' \
PYTHONPATH=. pixi run -e dev python scripts/bag/diag_bag_session_inventory.py \
  ../../data/live/runs --prefix run_20260913
```

### 7.3 La medida que decide: reversion de vuelta

```bash
cd src/python
VTITAN_HARDWARE_PROFILE='270deg-hiwonder-35kg,rev-hd-hex-motor-6000rpm' \
PYTHONPATH=. pixi run -e dev python scripts/bag/diag_bag_lap_drawdown.py \
  ../../data/live/runs/run_20260913_* \
  ../../data/live/runs/run_20260911_152318
```

**El ultimo bag es obligatorio y es el control positivo.** `152318` hizo 3/3
vueltas limpias y debe leer **18 deg**. Si lee otra cosa, el lector esta roto y
tus numeros nuevos no valen. Un drawdown por encima de 25 deg es una reversion;
los tres runs afectados de 2026-09-11 leen 220 / 85 / 235.

### 7.4 De donde viene el target

```bash
cd src/python
VTITAN_HARDWARE_PROFILE='270deg-hiwonder-35kg,rev-hd-hex-motor-6000rpm' \
PYTHONPATH=. pixi run -e dev python scripts/bag/diag_bag_target_loop_sense.py \
  ../../data/live/runs/run_20260913_* \
  ../../data/live/runs/run_20260911_152318 --detail
```

Tambien con el control obligatorio, que debe leer **0.7% target mal sentido /
0.0% heading mal / 0.0% target detras**.

Y para partir un run por su propia frontera de ventana, que es el unico control
que mantiene fija la tasa de escapes:

```bash
... diag_bag_target_loop_sense.py <un bag> --window 134.7-196.5 --detail
```

La ventana la saca `diag_bag_lap_drawdown.py` en su columna `window`.

### 7.5 Lado de paso, y pedido contra logrado

```bash
cd src/python
VTITAN_HARDWARE_PROFILE='270deg-hiwonder-35kg,rev-hd-hex-motor-6000rpm' \
PYTHONPATH=. pixi run -e dev python scripts/bag/diag_bag_pass_side.py \
  ../../data/live/runs/run_20260913_*
```

Mira el bloque **ASKED vs ACHIEVED**. Medido 2026-09-12: los 8 roces por debajo
de 30 mm eran **todos** fallo de seguimiento, cero de plan (el router pedia
0.247 m de media y el chasis lograba 0.014 m). Si eso cambia, el diagnostico
cambia.

---

## 8. Con qué criterio decidir

### 8.1 Lo que estas guardias NO pueden hacer

**No recuperan un coche ya girado.** Hay un test que lo fija a proposito
(`test_a_fully_rotated_chassis_is_not_rescued_by_the_gate`): una vez el chasis
esta del todo al reves, **todos** los candidatos del span son de sentido
erroneo, el filtro vacia el barrido y los tiers de respaldo devuelven el mismo
punto que habria devuelto la busqueda sin filtrar.

Eso no es un defecto de la guardia, es la razon por la que se describe como
**prevencion**. La recuperacion necesitaria algo que reoriente el chasis, y
**no existe**: cada enganche de maniobra de este arbol se dispara por LIDAR del
carril frontal o por 3 cm de odometria en 2 s, y ninguno salta con un robot que
se mueve bien pero apunta al reves. `tangent_rad` se calcula en
`track_geometry.py` y **solo** lo leen scripts de diagnostico.

### 8.2 Criterio para dejarlas encendidas

Esto aplica a `target_sense_gate`. La otra no hace nada (seccion 1.3).

Enciendela si **las tres** condiciones se cumplen:

1. **El sim no empeora.** En Obstacles con camara, `in_time` y `laps3` no bajan
   y `collided` no sube. Plano es aceptable: el corpus del sim tiene el mapa de
   señales **exacto**, asi que la deformacion patologica que motivo la segunda
   guardia casi no aparece ahi. Un plano en sim **no** es evidencia contra.
2. **Open no se toca.** `target_sense_gate` afecta a los dos retos, asi que su
   A/B de Open tiene que salir plano o mejor. Si Open empeora, apaga esa y deja
   solo la de deformacion.
3. **En pista, el drawdown del control sigue en 18 y tus runs bajan.** Es la
   unica medida que importa: el lado equivocado **termina la ronda**, no es una
   deduccion de puntos.

Apagalas si en pista aparece cualquier comportamiento nuevo de mira: el sintoma
seria el coche cortando hacia dentro en curva, porque la guardia de busqueda
puede saltarse un candidato y quedarse con uno mas cercano.

### 8.3 Que esperar de cada una

| bandera | contra que origen | evidencia |
|---|---|---|
| `target_sense_gate` | la busqueda a 2.07 m, mecanismo **pre**-cota | los targets de sentido erroneo no llevaban deformacion (p50 0.000 m, 27-44% deformados en las tres rondas de 2026-09-11) |
| `sign_deform_sense_guard` | **INERTE, ver 1.3** | la deformacion esta suprimida globalmente; el campo que la midio es un contrafactual |

Mismo sintoma, y **un solo origen accionable hoy**. El segundo quedaba cubierto
por una guardia que no puede ejecutarse.

---

## 9. Trampas que te van a morder offline

1. **El TOML gana.** Editar el default de Pydantic es inerte.
2. **Un diagnostico que revienta SALE CON CODIGO 0.** Lee la salida, nunca el
   codigo de retorno.
3. **Prueba un NULO con un control presente en la misma consulta.** Si no, no
   puedes distinguir el hecho de una ruta mal escrita.
4. **`VTITAN_HARDWARE_PROFILE`**: puesta para diagnosticos de bag y sim, **sin
   definir** para la suite de tests.
5. **Verifica por contenido, nunca por hash.** Los hashes cambian con un rebase.
6. **Dos sesiones comparten un solo indice de git y un solo arbol de trabajo.**
   Si aparecen modificados ficheros que no tocaste, no son tuyos.
7. **`git stash push -u` falla a medias** en esta maquina. Para un rebase usa
   `git pull --rebase --autostash`.
8. **`travelled_m` es SIGNADO** y se cancela bajo un vaiven. No lo uses para
   medir progreso.
9. **`min_lidar_range_m` se calcula ANTES del filtro**: es basura siempre, no lo
   cites como evidencia de contacto.
10. **La linea base de un A/B debe ser el commit PADRE**, y el brazo tiene que
    contener de verdad el arreglo que dices medir.
11. **El sim es ciego a la clase de lazo de control.** Ejercita
    `side_correction` en el 1.09% de los ticks contra el 19-25% del hardware. Un
    A/B de sim sobre cualquier cosa reactiva no decide.

---

## 10. Para revertir todo

```bash
# solo las banderas, dejando el codigo
sed -i 's/^target_sense_gate = true/target_sense_gate = false/' \
  src/config/navigation/motion/pursuit.toml
sed -i 's/^sign_deform_sense_guard = true/sign_deform_sense_guard = false/' \
  src/config/navigation/signs/sign_router.toml

# el codigo entero de las guardias (si estaba commiteado, usa git revert)
git checkout -- src/python/src/navigation/control/controllers/waypoint_controller.py \
                src/python/src/navigation/core_navigator/navigator.py \
                src/python/shared/src/shared/config/navigation_tuning/motion.py \
                src/python/shared/src/shared/config/navigation_tuning/signs.py \
                src/config/navigation/motion/pursuit.toml \
                src/config/navigation/signs/sign_router.toml
```

Con las banderas en `false` el arbol es **bit-identico** al de antes: las dos
guardias estan detras de un `if` sobre la bandera y no hay ninguna otra ruta
nueva. Eso es lo que fijan los tests de estado OFF.

---

## 11. Estado al cerrar, 2026-09-12

- Codigo escrito, 14 tests pasando, las dos banderas **apagadas**.
- Alcanzabilidad de las dos banderas **verificada** con control negativo.
- Barridos de sim **lanzados** y sin leer: los resultados quedan en
  `src/python/scripts/sim/output/sense_gate_obstacles_sighted.txt`,
  `deform_guard_obstacles_sighted.txt`, `sense_gate_obstacles_blind.txt` y
  `sense_gate_open_128.txt`. Si esos ficheros estan a medias, el barrido se
  corto: relanzalo con los comandos de la seccion 5.
- **Nada de esto se ha probado en pista.**
