# Plan: CLI unificada sobre los Taskfiles (cobra + lipgloss)

- Estado: propuesto, sin implementar
- Fecha: 2026-09-22
- Alcance: superficie de comandos de desarrollo, no el runtime del robot

Este documento es el plan. Cuando la primera fase aterrice, la decision
arquitectonica se registra como ADR (ver [Paso final](#paso-final-adr)); el ADR
guarda el *por que*, este fichero guarda el *como* y se borra cuando deje de
describir trabajo pendiente.

## 1. Punto de partida (medido, no estimado)

`task --list-all --json` devuelve **303 tareas** repartidas en 15 Taskfiles, y
la raiz las aplana todas en un unico namespace (`flatten: true` en 10 de los 15
includes).

| Taskfile | lineas | tareas |
| --- | ---: | ---: |
| `other/tasks/platform.yml` | 728 | 47 |
| `other/tasks/fleet.yml` | 532 | 40 |
| `other/ml/hailo/Taskfile.yml` | 402 | 54 |
| `other/apps/auto-annotator/Taskfile.yml` | 401 | 51 |
| `src/python/Taskfile.yml` | 415 | 29 |
| `src/go/Taskfile.yml` | 363 | 19 |
| `Taskfile.yml` (raiz) | 190 | 20 |
| `other/deploy/ansible/Taskfile.yml` | 175 | 7 |
| `other/apps/hugo-docs/Taskfile.yml` | 122 | 12 |
| `other/apps/backend/Taskfile.yml` | 109 | 17 |
| resto (frontend, gazebo, weights, infra, src) | 178 | 24 |

Hechos que condicionan el diseno:

- **La discoverability ya se rompio una vez.** `other/tasks/platform.yml`
  contiene una tarea `help` que es un `echo` de varias decenas de lineas con un
  menu curado a mano. Es util justamente porque `task --list` es un muro de 303
  lineas, y es exactamente el tipo de artefacto que deriva en silencio: nada
  falla cuando se anade una tarea y no se anade al menu.
- **Cobra ya esta dentro.** `github.com/spf13/cobra v1.10.2` es dependencia
  directa de `src/go`, y 11 de los 13 binarios de `cmd/` la usan (`capture-node`
  y `pi5` siguen con `flag`). `internal/cmdkit` ya centraliza los flags
  compartidos (`--nats-url`, `--name`, `--profiles`, `--config-root`,
  `--runs-root`). O sea: la convencion existe y esto la extiende, no la inventa.
- **Lipgloss no esta en el arbol.** Seria dependencia nueva.
- **El JSON de Task da el arbol gratis, pero no los parametros.**
  `task --list-all --json` expone `name`, `desc`, `aliases` y `location`. No
  expone `vars` ni `requires`. Solo **30 tareas** declaran `requires.vars`; el
  resto codifica sus parametros en texto libre dentro del `desc`
  (`"usage: task windows:ping PING_HOST=address"`). Los flags tipados hay que
  declararlos a mano, comando por comando: **ese es el trabajo real**, no el
  cableado de cobra.
- **El passthrough ya existe y es incomodo.** 11 tareas usan `{{.CLI_ARGS}}`,
  que obliga a la sintaxis `task x:y -- --flag valor`.
- **La mayoria de las tareas son envoltorios finos.** `uv`, `pixi`, `ruff`,
  `docker`, `ansible`, `ssh`, `scp`, `npx`, con variantes
  `platforms: [windows]` / `[linux, darwin]` para el mismo comando. Reescribir
  esos cuerpos en Go pierde las variantes por plataforma y no compra nada.

## 2. Decisiones ya tomadas

**D1. Hibrido: envolver ahora, migrar despues lo que tiene logica de verdad.**

Cobra pasa a ser el arbol de comandos, los flags y la ayuda; cada hoja envuelta
ejecuta `task <nombre> VAR=... -- <args>`. Los Taskfiles siguen siendo la unica
fuente de verdad sobre *como* se ejecuta cada cosa. Mas adelante, los flujos con
logica real (saltos SSH de flota, cadenas `rpi:provision:*`, seleccion de host)
se reimplementan nativos en Go y dejan de delegar.

Se descarto la migracion completa: 303 cuerpos de tarea reescritos, las
variantes `platforms:` perdidas, y cada one-liner de `docker`/`pixi` convertido
en fontaneria `exec`.

**D2. Primer corte curado, no generado entero.**

Flags tipados a mano para los flujos que se usan a diario (`sim:*`, `robot:*`,
`go:*`, `rpi:*`/`windows:*`, y los paraguas `test`/`lint`/`format`/`clean`),
aproximadamente 40-60 comandos. Las ~250 restantes siguen alcanzables por un
catch-all generado, sin flags tipados.

## 3. Diseno

### 3.1 Binario

`src/go/cmd/vt`, construido por una tarea nueva `cli:build` hacia
`src/go/bin/vt{{.EXE}}`, igual que `simgen` ya hace. Nombre por decidir (ver
[Preguntas abiertas](#7-preguntas-abiertas)).

`vt` es **herramienta de maquina de desarrollo**. En los Pi no se asume Go
instalado: las tareas que corren *sobre* la placa se siguen invocando con
`task`, que es lo que ya hacen hoy a traves de SSH.

### 3.2 Arbol de comandos

La convencion actual `modulo:verbo` se traduce directa a subcomandos anidados:

```
task sim:navigate:visualize:all PROFILE=... -- --challenge open
vt sim navigate visualize all --profile ... --challenge open

task windows:ping PING_HOST=192.168.251.2
vt fleet ping 192.168.251.2

task rpi:provision:pi5 PI5_IP=... TAGS=network,hardware
vt rpi provision pi5 --ip ... --tags network,hardware
```

Dominios de primer nivel propuestos: `sim`, `robot`, `rpi`, `fleet`, `go`,
`py`, `docs`, `apps`, `ml`, `infra`, mas los paraguas sueltos (`test`, `lint`,
`format`, `clean`, `install`).

### 3.3 Dos clases de comando

- **Envuelto (`wrapped`)**: declara flags tipados y los mapea a variables de
  Task. Ejecuta `task` como proceso hijo con stdin/stdout/stderr conectados
  directamente y **propaga el codigo de salida sin tocarlo**. No interpreta la
  salida del hijo.
- **Nativo (`native`)**: implementa la logica en Go, sin `task` de por medio.
  Reservado para la fase 3.

### 3.4 Especificacion declarativa

Una tabla unica (Go, o YAML con `go:embed`) describe cada comando envuelto:
ruta en el arbol, tarea destino, flags tipados con su variable de Task, cuales
son obligatorios, y si acepta passthrough a `CLI_ARGS`. Nada de un fichero por
comando.

### 3.5 Anti-deriva (el punto que decide si esto sobrevive)

El fallo esperable de este diseno es la doble fuente de verdad: alguien renombra
una tarea y la CLI apunta al vacio. Mitigacion, en tests:

1. Toda tarea referenciada por la spec existe en `task --list-all --json`.
2. Toda tarea en un dominio curado esta cubierta por la spec **o** figura en
   una lista explicita de exclusiones. Anadir una tarea a un dominio curado y no
   decidir nada, falla.
3. El paraguas generado cubre las 303 sin excepcion.

Es el mismo patron que ya usa `task config:check` para las referencias
`x-journal` de los schemas: la propiedad es un fallo de build, no una costumbre.

### 3.6 Lipgloss: solo presentacion

Ayuda agrupada por dominio, listados en tabla, errores, y el banner de `vt`
sin argumentos. Reglas duras:

- Respetar `NO_COLOR`, y desactivar estilo cuando stdout no es TTY. CI y los
  logs se leen en texto plano.
- Nada de estilo en salida pensada para pipe. Si un comando emite datos, van
  limpios; el adorno va a stderr o no va.
- Sin TUI interactiva por ahora: nada de bubbletea en este plan.

### 3.7 Lo que se retira

La tarea `help` escrita a mano en `other/tasks/platform.yml` se borra en cuanto
`vt` cubra su contenido. Es el menu curado que el arbol de cobra genera solo.

## 4. Fases

**Fase 0: esqueleto.** `cmd/vt`, spec vacia, catch-all generado desde el JSON
de Task, los tres tests anti-deriva, tarea `cli:build`. Al final de la fase
`vt` ya reemplaza a `task --list` como forma de descubrir, sin flags tipados
todavia.

**Fase 1: dominios curados.** Flags tipados para los flujos diarios, dominio
por dominio, empezando por el que mas se usa. Cada dominio cerrado anade su
entrada en la lista de exclusiones para que el test 2 empiece a morder.

**Fase 2: acabado.** Borrar el `help` manual, generar completions
(`bash`/`zsh`/`fish`, gratis con cobra), documentar en README el nuevo camino
y dejar escrito que `task X` sigue funcionando igual.

**Fase 3: migracion nativa selectiva.** Criterio para migrar una tarea a Go
nativo, tienen que cumplirse los tres:

1. Tiene logica de control real (bucles, condicionales, seleccion de host,
   reintentos), no es un envoltorio de una herramienta externa.
2. Se ejecuta desde la maquina de desarrollo, no sobre un Pi.
3. No depende de las variantes `platforms:` de Task para existir.

Candidatos que los cumplen: los saltos SSH encadenados de
`other/tasks/fleet.yml` (`windows:set-wifi:zero` hace SSH dentro de SSH con
comillas anidadas a tres niveles), la seleccion de host con defaults
(`SSH_HOST | default "rpi-5-direct"`, repetida en ~15 tareas), y los reintentos
de ping (`RPI_PING_RETRIES`, `RPI_PING_INTERVAL`).

**Fase 4, opcional y solo si la fase 3 demuestra valor.** Invertir la relacion
donde tenga sentido: la tarea de Task pasa a invocar `vt`, para que CI y la CLI
compartan implementacion en vez de duplicarla.

## 5. Lo que este plan NO hace

- No reescribe envoltorios de herramientas externas.
- No rompe nada: `task X` sigue funcionando exactamente igual durante todas las
  fases, y CI sigue llamando a `task`. Los dos caminos conviven a proposito.
- No introduce TUI interactiva.
- No toca el runtime del robot ni los binarios de `cmd/*` existentes.

## 6. Riesgos

| Riesgo | Mitigacion |
| --- | --- |
| Doble fuente de verdad entre spec y Taskfiles | Los tres tests de 3.5; sin ellos este plan no merece la pena |
| Hay que compilar antes de usar | `go run ./cmd/vt` como camino de bootstrap, y `cli:build` documentado en el arranque de dev |
| Go no instalado en los Pi | `vt` es solo de maquina de desarrollo; las placas siguen con `task`, sin cambios |
| Estilo ANSI ensuciando logs de CI | `NO_COLOR` y deteccion de TTY desde el primer commit, no como parche posterior |
| La CLI queda a medias y conviven dos formas de hacer lo mismo | Fase 2 borra el `help` manual: si esa fase no llega, el plan no se completo |

## 7. Preguntas abiertas

- Nombre del binario: `vt` (corto, se teclea a diario) o `vtitan` (explicito).
- Ubicacion: `src/go/cmd/vt` junto a los binarios del robot, o separado por ser
  herramienta de desarrollo y no artefacto desplegable.
- Version de lipgloss a fijar al anadir la dependencia.
- Distribucion: binario compilado por `cli:build`, o `go run` documentado.
- Si el catch-all se llama `vt task <nombre>` o `vt run <nombre>`.

## Paso final: ADR

Cuando la fase 0 aterrice, registrar la decision como ADR siguiendo
`other/docs/adr/0000-template.md`: contexto (303 tareas, `help` manual, cobra ya
presente), opciones (envoltorio / hibrido / migracion completa), decision
(hibrido, con el criterio de migracion de la fase 3) y consecuencias (dos
caminos conviviendo, tests anti-deriva como precio de entrada).
