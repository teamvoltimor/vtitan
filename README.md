# Team Voltimor

> 🕊️ Este proyecto está dedicado a la memoria de **Javier Pérez** ([@kaucrow](https://github.com/kaucrow)), amigo y colega, y de **Luna Margarita**, compañera de doce años. La [dedicatoria completa](MEMORIAL.md) vive en [`MEMORIAL.md`](MEMORIAL.md).

<p align="center">
    <img src="assets/voltimor-logo-square.png" alt="" width="400">
    <br>
    <i>Logo del Equipo</i>
</p>

Bienvenidos al repositorio de V-Titan, el robot del Team Voltimor, que compite en la World Robot Olympiad 2026 en la categoría Futuros Ingenieros. Aquí encontrarás toda la información sobre el robot, incluyendo su código, modelos 3D, esquemas y documentación.

<p align="center">
    <img src="t-photos/team-photo.jpeg" alt="" width="400">
    <br>
    <i>Foto del Equipo, de izquierda a derecha: Ramón Álvarez, Sebastián Álvarez, Jesús Pérez (Padre, Mentor), Jesús Pérez</i>
</p>

Actualmente, este equipo está conformado por 3 miembros:

- **Ramón Álvarez**, 20 años. [ralvarezdev](https://github.com/ralvarezdev). Ejerce como líder del equipo y es el encargado de la programación. Actualmente, trabaja en Automation Labs, y finalizó sus estudios en Ingeniería en Computación en URU.
- **Sebastián Álvarez**, 16 años. [salvarezdev](https://github.com/salvarezdev). Encargado tanto de la programación, como de la documentación y la toma de decisiones con respecto a la lógica del robot. Actualmente, cursa el 1er trimestre de Ingeniería en Computación en URU.
- **Jesús Pérez**, 16 años. [JesusPerez15](https://github.com/JesusPerez15). Encargado del diseño, la mecánica y la fabricación del robot. Actualmente, cursa el 5to año de bachillerato en el Colegio Salto Ángel.

## V-Titan en números

| Métrica | Valor | Contexto |
|---------|-------|----------|
| Detección de señales | **15 Hz** punta a punta | Captura 640x640 + inferencia NPU + publicación; 101.5 FPS el modelo solo |
| Corpus de simulación | **638 / 640** escenarios Open | Semilla fija y resultados repetibles; los 2 casos restantes identificados uno por uno |
| Peso del robot | **~1460 g** | Límite reglamentario 1500 g, con ~40 g de margen |
| Techo de velocidad real | **~0.58 m/s** | Descubierto al corregir un error de cuantización que lo limitaba a 0.45 m/s |
| Duración de una ronda | **180 s** sin pausa | Por eso todo lo que ocurre a bordo queda grabado en bags MCAP |
| Lazo de velocidad en pista | **~2% de error** de seguimiento | Tres vueltas limpias en 132.5 s frente a 142.3 s antes del ajuste |

Cada número es medido, no estimado, y puede rastrearse hasta el código y la medición que lo produjo vía el historial de git (ver [Versionado](#versionado)).

## Estructura del repositorio

La raíz del repositorio sigue la estructura que pide la categoría Futuros Ingenieros de la WRO. Cada carpeta obligatoria está en su sitio, y las que apuntan a un monorepo más grande llevan su propio `README.md` con la ruta exacta:

```text
vtitan/
├── README.md          # Este documento: la documentación completa de ingeniería
├── t-photos/          # Fotos del equipo
├── v-photos/          # Fotos de V-Titan y de los prototipos anteriores
├── video/             # Enlaces a los videos de las rondas (video/video.md)
├── schemes/           # Diagramas de flujo y esquemático de conexiones
│   ├── flowcharts/    #   Fuentes Mermaid + renders WebP: common/, open/, obstacles/
│   └── wiring/        #   Esquemático del arnés + proyecto tscircuit que lo genera
├── models/            # Modelos 3D de las piezas impresas
│   ├── current-models/  #   V-Titan: blueprints/ (planos) + step-files/ (CAD)
│   └── old-models/      #   Prototipos previos (Klevor)
├── src/               # → código de competencia (puntero a platform/robot/)
├── other/             # → simulador, telemetría, entrenamiento, provisionamiento
├── docs/              # Bitácora, guía WRO, datasheets, prototipos previos
├── platform/          # El monorepo de código: robot, backend, frontend, config
├── deploy/ansible/ + infra/  # Despliegue: provisionamiento Ansible + tareas task rpi:*
├── data/              # Salidas de ejecución (bags, fotos, videos); vacía en el repo
├── hailo/ ml-models/  # Entrenamiento del detector y pesos publicados
├── apps/              # Herramientas con proceso propio: auto-annotator/, hugo-docs/
├── scripts/ assets/   # Utilidades de desarrollo e imágenes del documento
└── .github/           # Workflows de CI
```

| Carpeta | Contenido |
|---------|-----------|
| `README.md` | Este documento: la documentación completa de ingeniería de V-Titan |
| `t-photos/` | Fotos del equipo |
| `v-photos/` | Fotos de V-Titan y de los prototipos anteriores |
| `video/` | Enlaces a los videos de las rondas y del robot en funcionamiento ([`video/video.md`](video/video.md)) |
| `schemes/` | Diagramas de flujo y esquemático de conexiones. En `schemes/flowcharts/` están las fuentes Mermaid y sus renders WebP, separados en `common/` (lógica compartida por ambos desafíos), `open/` y `obstacles/`; `schemes/flowcharts/_legacy/` conserva los diagramas de versiones anteriores. En `schemes/wiring/` está el esquemático del arnés junto al proyecto tscircuit que lo genera |
| `models/` | Modelos 3D de las piezas impresas: `current-models/` (V-Titan) y `old-models/` (prototipos previos), cada uno con `blueprints/` (planos) y `step-files/` (CAD para imprimir) |
| `src/` | Puntero al código de competencia, que vive en `platform/robot/` dentro del monorepo. Ver [`src/README.md`](src/README.md) |
| `other/` | Puntero al resto del proyecto: simulador, backend de telemetría, entrenamiento del detector y provisionamiento. Ver [`other/README.md`](other/README.md) |

### Cómo explorar este repositorio

Según lo que quieras revisar, esta es la ruta más corta:

- **Código que corre en una ronda**: [`src/README.md`](src/README.md) mapea cada nodo ROS2 de la pila de competencia a su paquete y su rol (percepción, navegación, máquina de estados, drivers).
- **Configuración que gobierna al robot**: `platform/shared/config/`, descrita en [Diseño gobernado por configuración](#diseño-gobernado-por-configuración). Los perfiles de hardware intercambiables están en [Perfiles de hardware intercambiables](#perfiles-de-hardware-intercambiables).
- **Simulador y corpus de escenarios**: [`other/README.md`](other/README.md), sección del simulador; los resultados reproducibles están en [Simulador y corpus de escenarios](#simulador-y-corpus-de-escenarios).
- **Cómo se entrenó el detector**: `hailo/` (entrenamiento y compilación), `ml-models/` (pesos publicados), `apps/auto-annotator/` (anotación asistida).
- **Cómo se instala el sistema en las placas**: [`docs/pi-setup.md`](docs/pi-setup.md) y `deploy/ansible/`; automatizado por los comandos `task rpi:provision:*` de [Arranque rápido](#arranque-rápido-y-reproducibilidad).
- **El historial del proyecto**: bitácora de ingeniería en [`docs/bitacora_ingenieria.md`](docs/bitacora_ingenieria.md), prototipos previos en [`docs/development/previous-prototypes/`](docs/development/previous-prototypes/klevor-v0.1.md), y los tags de git (`v1.0` regional, `v1.1` post-regional) con mensajes de commit convencionales.

Además de las carpetas obligatorias, el repositorio contiene:

- `docs/` con la documentación de apoyo: la [bitácora de ingeniería](docs/bitacora_ingenieria.md), el [documento de ingeniería WRO](docs/documentacion_ingenieria_wro.md), la [guía de instalación de las Raspberry Pi](docs/pi-setup.md), las hojas de datos en `docs/reference/datasheets/` y el historial de prototipos en `docs/development/previous-prototypes/`.
- `platform/` con el código: `platform/robot/` (la pila ROS2 de competencia), `platform/robot-go/` (la segunda implementación en Go), `platform/backend/` y `platform/frontend/` (telemetría), y `platform/shared/config/` (la configuración que gobierna al robot).
- `hailo/` con el entrenamiento y la compilación del detector YOLO, `ml-models/` con los pesos publicados, `apps/auto-annotator/` con la herramienta de anotación asistida.
- `deploy/ansible/` y `infra/` con el despliegue: el provisionamiento de las placas con Ansible, y las tareas de infraestructura (`task rpi:*`, `task windows:provision:*`) que lo ejecutan, definidas en `infra/Taskfile.yml`.
- `scripts/` con utilidades de desarrollo (setup de SSH para el robot), `assets/` con las imágenes que usa este documento, `.github/` con los workflows de CI, y `apps/hugo-docs/` con el sitio de documentación navegable.
- `data/` es la carpeta de salida en runtime: `data/live/` y `data/sim/` guardan los bags, fotos y videos que producen las corridas del robot y del simulador. En el repositorio solo está su estructura (archivos `.gitkeep`); el contenido se llena al ejecutar `task platform:robot:pull-runs` (bags desde la Pi 5), `task platform:robot:pull-videos` (videos por ronda) o las corridas de simulación, y no se versiona.

## Arranque rápido y reproducibilidad

Todo el ciclo de vida del proyecto, desde la simulación y las pruebas hasta el despliegue al robot y la operación en pista, está automatizado con [Task](https://taskfile.dev) (Taskfile) y [Pixi](https://pixi.sh) / [uv](https://docs.astral.sh/uv/). Los comandos principales son `task`, y cada uno admite `task --list` para descubrirlos. Nada de lo que hacemos depende de pasos manuales no documentados: otra persona puede clonar el repositorio y llegar del código al robot con estos comandos.

### Requisitos previos (una sola vez)

- [Task](https://taskfile.dev) (`go install github.com/go-task/task/v3/cmd/task@latest` o el instalador de la página)
- [Pixi](https://pixi.sh) (gestiona los entornos de Python + ROS2 en el robot y la simulación)
- [Go](https://go.dev) 1.25+ (backend de telemetría, generador de escenarios y binarios del robot)
- Node.js 22+ (dashboard, el panel de telemetría)

### Desarrollo y simulación (en el computador de desarrollo)

```bash
task platform:install          # Dependencias de simulación, robot y backend Go
task platform:init:dev         # Setup completo: install + lint

# Generar la pista y el corpus de escenarios (semilla fija → resultados repetibles)
task platform:gen:corpus:all   # 640 escenarios Open + 256 Obstacles, seed 2026

# Visualizar una carrera cerrada en RViz (simulador del navegador)
task platform:sim:navigate:visualize:all -- --challenge open --interactive

# Pruebas: todas, o por subsistema
task platform:test             # Python + Go, todos los módulos
task platform:robot:test SCOPE=navigation
task platform:robot:test SCOPE=unit
```

### Despliegue al robot (desde el computador, por SSH)

```bash
task platform:robot:deploy         # Código + detector HEF → rebuild colcon → restart del servicio
task platform:robot:watch-vision   # Detecciones en vivo, una línea por frame
task platform:robot:pull-runs      # Descargar los bags MCAP de las carreras
```

### Operación en pista (directamente en la Raspberry Pi 5, vía SSH)

```bash
task rpi:stack ACTION=up           # Levantar la pila de servicios en orden correcto
task rpi:stack ACTION=status       # Estado de la pila
task rpi:stack ACTION=logs         # Últimos logs de todos los servicios
task rpi:health                    # Captura de salud: temperatura, throttle, disco, RAM
task robot:calibrate-encoder       # Calibración de pulsos/vuelta contra distancia medida
task robot:test-motors             # Prueba de humo de hardware: rango de servo + pulso de motor
```

### Provisionado desde cero (instalar el sistema en las Raspberry Pi)

```bash
task windows:provision:pi5         # Provisionar la Pi 5 con Ansible (tags opcionales)
task rpi:provision:all             # Ambas placas, en tmux, tras reflashear la SD
task rpi:ansible:check TARGET=pi5  # Dry-run + diff del provisionador
```

### Documentación

```bash
task docs:diagrams                 # Re-renderizar todos los diagramas Mermaid a WebP
cd schemes/wiring/tscircuit && npm run artifacts   # Regenerar el esquemático del arnés
```

### Versionado

Marcamos hitos del proyecto con tags de git: `v1.0` es el estado del robot para el evento regional de WRO 2026, y el historial entre tags es un registro continuo de commits con mensajes convencionales (`fix(robot):`, `docs(readme):`, `perf(nav):`, ...). Cualquier resultado medido en este documento (tasas del corpus, FPS del detector, consumo de potencia) puede rastrearse hasta el código exacto que lo produjo vía el historial.

## Índice

1. **[V-Titan en números](#v-titan-en-números)**
2. **[Estructura del repositorio](#estructura-del-repositorio)**
3. **[Arranque rápido y reproducibilidad](#arranque-rápido-y-reproducibilidad)**
4. **[Historial del equipo](#historial-del-equipo)**
    1. [Klevor (WRO 2025)](#klevor-wro-2025)
        1. [Klevor v0.1](docs/development/previous-prototypes/klevor-v0.1.md)
        2. [Klevor v0.1.1](docs/development/previous-prototypes/klevor-v0.1.1.md)
        3. [Klevor v0.2](docs/development/previous-prototypes/klevor-v0.2.md)
        4. [Klevor v1.0](docs/development/previous-prototypes/klevor-v1.0.md)
    2. [V-Titan (WRO 2026)](#v-titan-wro-2026)
5. **[Arquitectura de energía y sensores](#arquitectura-de-energía-y-sensores)**
    1. [Lista de Componentes](#lista-de-componentes)
        1. [Raspberry Pi 5 (16GB RAM)](#raspberry-pi-5-16gb-ram)
        2. [Raspberry Pi Camera Module 3 Wide](#raspberry-pi-camera-module-3-wide)
        3. [Raspberry Pi AI HAT+ (26 TOPS)](#raspberry-pi-ai-hat-26-tops)
        4. [Raspberry Pi Zero 2 W](#raspberry-pi-zero-2-w)
        5. [RPLiDAR C1](#rplidar-c1)
        6. [Hi Wonder HPS-3527SG 35kg Servo](#hi-wonder-hps-3527sg-35kg-servo)
        7. [HD Hex Motor](#hd-hex-motor)
        8. [9-Axis IMU Gyroscope GY-BNO085](#9-axis-imu-gyroscope-gy-bno085)
        9. [Ovonic Air 11.1V Li-Po Battery](#ovonic-air-111v-li-po-battery)
        10. [Puente H BTS7960 / IBT-2](#puente-h-bts7960--ibt-2)
        11. [Step Down Mini-560 Pro](#step-down-mini-560-pro)
        12. [SSD1306 OLED Display](#ssd1306-oled-display)
        13. [Convertidor KL89576 (DC a USB-C)](#convertidor-kl89576-dc-a-usb-c)
    2. [Diagrama de Conexiones](#diagrama-de-conexiones)
        1. [Consumo Energético](#consumo-energético)
        2. [Calibración](#calibración)
6. **[Movilidad y Diseño Mecánico](#movilidad-y-diseño-mecánico)**
    1. [Métodos de Prototipaje](#métodos-de-prototipaje)
    2. [Evolución y Justificación Del Diseño](#evolución-y-justificación-del-diseño)
        1. [**Restricciones Iniciales**](#restricciones-iniciales)
    3. [Sistema de Transmisión](#sistema-de-transmisión)
    4. [Sistema de Dirección](#sistema-de-dirección)
    5. [Chasis Inferior](#chasis-inferior)
    6. [Monochasis](#monochasis)
    7. [Relación de Torque y Velocidad](#relación-de-torque-y-velocidad)
        1. [Velocidad: teórica contra real](#velocidad-teórica-contra-real)
7. **[Arquitectura de software y estrategia para superar obstáculos](#arquitectura-de-software-y-estrategia-para-superar-obstáculos)**
    1. [Arquitectura ROS2 y reparto entre dos computadores](#arquitectura-ros2-y-reparto-entre-dos-computadores)
    2. [Modelo de Detección YOLO](#modelo-de-detección-yolo)
        1. [Qué pasa cuando la visión falla](#qué-pasa-cuando-la-visión-falla)
    3. [Algoritmo PID](#algoritmo-pid)
    4. [Estrategia en pista](#estrategia-en-pista)
        1. [Inferencia del sentido de la vuelta](#inferencia-del-sentido-de-la-vuelta)
        2. [Seguimiento de pasillo, vueltas y escapes](#seguimiento-de-pasillo-vueltas-y-escapes)
        3. [Vista completa de cada desafío](#vista-completa-de-cada-desafío)
    5. [Grabación y análisis de carreras](#grabación-y-análisis-de-carreras)
    6. [Simulador y corpus de escenarios](#simulador-y-corpus-de-escenarios)
8. **[Pensamiento sistémico y decisiones de ingeniería](#pensamiento-sistémico-y-decisiones-de-ingeniería)**
    1. [Diseño gobernado por configuración](#diseño-gobernado-por-configuración)
    2. [Perfiles de hardware intercambiables](#perfiles-de-hardware-intercambiables)
    3. [Ciclo de trabajo: idea → simulación → pista](#ciclo-de-trabajo-idea--simulación--pista)
    4. [Hallazgos de ingeniería](#hallazgos-de-ingeniería)
    5. [Gestión de riesgos](#gestión-de-riesgos)
    6. [Tecnologías utilizadas](#tecnologías-utilizadas)
9. **[Videos de V-Titan](#videos-de-v-titan)**

# Historial del equipo

En este apartado, discutimos brevemente nuestras experiencias pasadas con la categoría de Futuros Ingenieros y nuestro aprendizaje gracias a prototipos previos a la presente temporada.

## Klevor (WRO 2025)

<table>
        <tbody>
                <tr>
                        <td>
                                <p align="center">
                                        <img src="v-photos/previous-prototypes/klevor-v1.0/klevor-front-view.webp"
alt="Vista delantera de Klevor" width="600">
                                        <br>
                                        <i>Vista delantera de Klevor</i>
                                </p>
                        </td>
                        <td>
                                <p align="center">
                                        <img src="v-photos/previous-prototypes/klevor-v1.0/klevor-back-view.webp"
alt="Vista trasera de Klevor" width="600">
                                        <br>
                                        <i>Vista trasera de Klevor</i>
                                </p>
                        </td>
                </tr>
                <tr>
                        <td>
                                <p align="center">
                                        <img src="v-photos/previous-prototypes/klevor-v1.0/klevor-right-view.webp"
alt="Vista derecha de Klevor" width="600">
                                        <br>
                                        <i>Vista derecha de Klevor</i>
                                </p>
                        </td>
                        <td>
                                <p align="center">
                                        <img src="v-photos/previous-prototypes/klevor-v1.0/klevor-left-view.webp"
alt="Vista izquierda de Klevor" width="600">
                                        <br>
                                        <i>Vista izquierda de Klevor</i>
                                </p>
                        </td>
                </tr>
                <tr>
                        <td>
                                <p align="center">
                                        <img src="v-photos/previous-prototypes/klevor-v1.0/klevor-top-view.webp"
alt="Vista superior de Klevor" width="600">
                                        <br>
                                        <i>Vista superior de Klevor</i>
                                </p>
                        </td>
                        <td>
                                <p align="center">
                                        <img src="v-photos/previous-prototypes/klevor-v1.0/klevor-bottom-view.webp"
alt="Vista inferior de Klevor" width="600">
                                        <br>
                                        <i>Vista inferior de Klevor</i>
                                </p>
                        </td>
                </tr>
        </tbody>
</table>

Klevor es el **predecesor** de V-Titan, participando en la temporada 2025 de la World Robot Olympiad en la categoría de Futuros Ingenieros, con el Team Steel Bot (quienes ahora participan bajo el nombre de Team Voltimor) y como todo proyecto fue evolucionando hasta culminar con la versión que tenemos hoy en día. 

Para conocer a nuestro prototipo actual, V-Titan, mejor, es importante recalcar que muchas de sus características, más específicamente en la electrónica y programación, son **directamente heredadas** de Klevor, con cambios nulos o mínimos entre un prototipo o el otro. Algunas de las **herencias** más importantes son:

- El manejo de la Raspberry Pi 5 como computadora principal

- La detección de los obstáculos gracias a la Raspberry Pi Camera 3 y su Hailo AI+

- La comunicación serial entre una computadora principal y un microcontrolador

Ahora bien, también hay que recalcar que tuvimos algunos fallos en el desarrollo de Klevor, por ejemplo: 

El uso de un ESC para controlar el motor, si bien parecía una idea muy buena en papel, utilizar el "combo" de un carro controlado por radio para nuestro prototipo, ofreciendo una velocidad bastante alta para completar los desafíos, terminó siendo un problema grave debido a la falta de precisión que éste nos ofrecía, acelerando muy rápido, sin ninguna solución en la programación para compensarlo.

Además, optamos por un modelo más robusto y pesado en comparación con los demás prototipos habituales de esta competición, si bien, gracias a esto pudimos incorporar muchos elementos de gran utilidad (como la Raspberry Pi 5), debido a ésto, no podíamos optar por cambios significativos, siendo obligados a reestructurar el prototipo desde cero en caso de necesitar algún cambio.

Debido a la gran cantidad de cambios que necesitamos, por diferentes motivos, teníamos que reestructurar el prototipo múltiples veces, por lo que terminamos confiando ciegamente en algunas características que no pudimos probar completamente.

## V-Titan (WRO 2026)

<table>
        <tbody>
                <tr>
                        <td>
                                <p align="center">
                                        <img src="v-photos/v-titan/v-titan-front-view.webp"
alt="Vista delantera de V-Titan" width="600">
                                        <br>
                                        <i>Vista delantera de V-Titan</i>
                                </p>
                        </td>
                        <td>
                                <p align="center">
                                        <img src="v-photos/v-titan/v-titan-rear-view.webp"
alt="Vista trasera de V-Titan" width="600">
                                        <br>
                                        <i>Vista trasera de V-Titan</i>
                                </p>
                        </td>
                </tr>
                <tr>
                        <td>
                                <p align="center">
                                        <img src="v-photos/v-titan/v-titan-right-view.webp"
alt="Vista derecha de V-Titan" width="600">
                                        <br>
                                        <i>Vista derecha de V-Titan</i>
                                </p>
                        </td>
                        <td>
                                <p align="center">
                                        <img src="v-photos/v-titan/v-titan-left-view.webp"
alt="Vista izquierda de V-Titan" width="600">
                                        <br>
                                        <i>Vista izquierda de V-Titan</i>
                                </p>
                        </td>
                </tr>
                <tr>
                        <td>
                                <p align="center">
                                        <img src="v-photos/v-titan/v-titan-top-view.webp"
alt="Vista superior de V-Titan" width="600">
                                        <br>
                                        <i>Vista superior de V-Titan</i>
                                </p>
                        </td>
                        <td>
                                <p align="center">
                                        <img src="v-photos/v-titan/v-titan-bottom-view.webp"
alt="Vista inferior de V-Titan" width="600">
                                        <br>
                                        <i>Vista inferior de V-Titan</i>
                                </p>
                        </td>
                </tr>
        </tbody>
</table>

V-Titan es el **sucesor** de Klevor, participando en la temporada 2026 de la World Robot Olympiad en la categoría Futuros Ingenieros, con el Team Voltimor (anteriormente Team Steel Bot), y es un proyecto que se encuentra evolucionando hasta el día de hoy.

V-Titan mejora en muchos aspectos con respecto a su predecesor, Klevor, con la mayoría de cambios siendo en el aspecto mecánico, ya que, una de nuestras metas principales era implementar un sistema de giro que permita el giro en 90 grados (o lo más cercano posible) para facilitar la estrategia para completar el Desafío Cerrado, además de esto, V-Titan conserva muchos de los componentes electrónicos que utilizó Klevor, tales como la Raspberry Pi 5, y el RPLiDAR C1.

# Arquitectura de energía y sensores 

En el siguiente apartado, se discute toda la parte electrónica de V-Titan, tales como sus sensores, las razones detrás de su elección, cómo se implementan y el presupuesto energético.

## Lista de Componentes
A continuación, está la descripción de todos los componentes principales de V-Titan.

### Raspberry Pi 5 (16GB RAM)

<p align="center">
	<img src="assets/images/components/raspberry-pi-5.webp" alt="Raspberry Pi 5" 
width="350">
	<br>
	<i>Raspberry Pi 5</i>
</p>

Equipada con un procesador ARM Cortex-A76 de 64 bits a 2.4 GHz. La Raspberry Pi 5 es nuestro controlador principal de elección, decidimos usar a la Raspberry Pi 5 debido a múltiples factores, entre ellos:

- **Compatibilidad**: Existen muchos componentes de V-Titan (como la Camera Module 3 Wide) que a su vez pertenecen al ecosistema Raspberry, lo que hace que implementarlos a la Raspberry Pi 5 no requiera tanto esfuerzo.

- **Potencia**: La Raspberry Pi 5 es uno de los computadores portátiles más potentes actualmente, gracias a esto, funciones demandantes como lo es el procesamiento de imágenes en tiempo real, son fácilmente realizables por una Raspberry Pi 5.

- **Portabilidad**: La Raspberry Pi 5 destaca entre los controladores, ya que no es una computadora bastante pesada, apenas llegando a los 60 g, hace que incorporarlo a V-Titan sea una opción prácticamente segura.

| **Medida** | **Valor** |
|------------|-----------|
| Largo      | 85 mm     |
| Alto       | 58.9 mm   |
| Ancho      | 56 mm     |
| Peso       | 46 g      |

### Raspberry Pi Camera Module 3 Wide

<p align="center">
	<img src="assets/images/components/raspberry-pi-camera-module-3.webp" alt="Raspberry Pi Camera Module 3" 
width="350">
	<br>
	<i>Raspberry Pi Camera Module 3</i>
</p>

La Raspberry Pi Camera Module 3 Wide es nuestra elección de preferencia, como los demás componentes Raspberry, esta se destaca por ser bastante ligera y portátil, ya que es una cámara bastante pequeña, midiendo apenas 25 mm × 24 mm × 12.4 mm y pesando 4 gramos, sin perder absolutamente ni una pizca de eficiencia, porque puede grabar a 1536 x 864p120, ahora bien, decidimos utilizar la versión Wide por su campo de visión horizontal de 102 grados, porque nos permite tener un rango de visión óptimo para poder detectar todos los obstáculos de la pista y lograr una mayor autonomía.

| **Medida** | **Valor** |
|------------|-----------|
| Largo      | 24 mm     |
| Alto       | 25 mm     |
| Ancho      | 12.4 mm   |
| Peso       | 4 g       |

**Montaje.** La cámara va montada directamente sobre el LIDAR (mismo desplazamiento frontal, x = 0.1222 m), a unos **20 cm del suelo** e inclinada **~10° hacia abajo**. La posición alta cumple dos funciones: despeja la línea de visión sobre el propio chasis y sobre los obstáculos bajos de la pista, y junto con la inclinación leve hacia abajo equilibra el cuadro entre la pista cercana (donde aparecen las señales que hay que leer a tiempo para decidir el lado de paso) y el horizonte del pasillo. El ángulo es lo bastante pequeño para que las señales a distancia de decisión (~1.4 m de radio de activación) queden bien dentro del encuadre, sin sacrificar la visión lejana que da la versión Wide. Las constantes de montaje viven en `platform/shared/config/robot.toml` (`[camera]`), y son las mismas que consumen la simulación y la TF estática.

**Calibración.** No hacemos calibración intrínseca de fábrica: el detector no necesita proyectar píxeles con precisión métrica, porque la decisión de la distancia al obstáculo la toma el LIDAR (la visión **no** es la red de seguridad de colisiones). Para las señales, la cámara aporta rumbo (preciso: la posición horizontal en el cuadro no depende de la profundidad) y color, mientras que la distancia por altura del cuadro delimitador (bounding box) es un modelo pinhole (proyección estenopeica) cuyo error crece con el rango (~3.6 cm a 1.5 m, ~14 cm a 3 m). Por eso el sistema fusiona ambas fuentes: cuando hay barrido LIDAR en el ciclo de muestreo, se confía en el rango del rayo más cercano al rumbo de la cámara, y el pinhole queda como respaldo. Su limitación conocida (asume cámara nivelada) está documentada honestamente en `platform/robot/docs/robot-physical-constants.md`.

### Raspberry Pi AI HAT+ (26 TOPS)

<p align="center">
	<img src="assets/images/components/raspberry-pi-ai-hat-plus.webp" alt="Raspberry Pi AI HAT+ 26 TOPS" 
width="350">
	<br>
	<i>Raspberry Pi AI HAT+ 26 TOPS</i>
</p>

Si bien la Raspberry Pi 5 es capaz de procesar imágenes en tiempo real, tras algunas pruebas, descubrimos que su tasa de procesamiento era bastante baja (alrededor de 1 a 2 fotos por segundo, con varias optimizaciones implementadas) por ende, tuvimos en cuenta que necesitaba más capacidad de cómputo, por lo cual decidimos incorporar la AI HAT+ a la Raspberry Pi 5 para poder alcanzar el nivel de procesamiento necesario.

El Raspberry Pi AI HAT+ tiene dos versiones, una de 13 Trillones de Operaciones por Segundo (TOPS) y otra de 26 TOPS. Como se menciona en el índice, V-Titan posee un Raspberry Pi AI HAT+ de 26 TOPS, gracias a este procesador de imágenes, V-Titan puede analizar imágenes de 640 px × 640 px a 15 Hz de punta a punta (captura, inferencia y publicación), con el modelo rindiendo 101 FPS en inferencia pura. La medición completa está en la [sección del modelo de detección](README.md#modelo-de-detección-yolo).

| **Medida** | **Valor** |
|------------|-----------|
| Largo      | 65 mm     |
| Alto       | 5.5 mm    |
| Ancho      | 56 mm     |
| Peso       | 9.07 g    |

### Raspberry Pi Zero 2 W

<p align="center">
	<img src="assets/images/components/raspberry-pi-zero-w.webp" alt="Raspberry Pi Zero W" 
width="350">
	<br>
	<i>Raspberry Pi Zero W</i>
</p>

Construida sobre el chip Broadcom BCM2710A1 de cuatro núcleos, la Raspberry Pi Zero 2 W es un ordenador de placa única ligero y ultra compacto para V-Titan. Al ejecutar un entorno Linux completo, este chip permite una fácil integración con el resto de los componentes Raspberry, haciendo que establecer comunicación de red o serial con una Raspberry Pi 5 sea nativo y sencillo dentro del mismo ecosistema.

Además de ofrecer cuatro núcleos a 1 GHz, supera drásticamente la capacidad de procesamiento de microcontroladores de tamaño similar, como el Arduino Nano que cuenta con una frecuencia de 16 MHz a 20 MHz.

Incorpora conectividad Wi-Fi/Bluetooth y cabezales de pines GPIO soldados. Esto ofrece una gran ventaja a la hora de desarrollar y practicar, ya que permite monitorear exactamente qué está procesando V-Titan en tiempo real a través de la red, sin necesidad de utilizar LED de distintos colores para señalizar decisiones y logrando un acabado final mucho más limpio.

| **Medida** | **Valor** |
|------------|-----------|
| Largo      | 65 mm     |
| Alto       | 13 mm     |
| Ancho      | 30 mm     |
| Peso       | 12 g      |

### RPLiDAR C1

<p align="center">
	<img src="assets/images/components/rplidar-c1.webp" alt="RPLiDAR C1" 
width="350">
	<br>
	<i>RPLiDAR C1</i>
</p>

El RPLiDAR C1 es un escáner de rango láser de 360 grados, el cual puede detectar superficies que están hasta 12 metros de distancia, su punto ciego es de tan solo 5 centímetros alrededor del mismo, todos estos factores hacen que el RPLiDAR C1 sea una gran opción para poder guíar a V-Titan por la pista.

Este RPLiDAR C1 permite a V-Titan poder identificar exactamente dónde está ubicado en la pista, gracias a que nos ofrece una visión de al menos 180 grados para poder manejar la navegación por la pista con una mayor autonomía, la prioridad para el uso apropiado de este sensor, en el caso de la categoría Futuros Ingenieros es colocarlo de tal manera que su láser esté por debajo de los 10 cm sobre el suelo, de tal manera que sea capaz de realizar mediciones a las paredes y los bloques, además de colocarlo lo más hacia el frente posible, y priorizar que nada lo esté tapando para que su visión sea despejada.

| **Medida** | **Valor** |
|------------|-----------|
| Largo      | 55.6 mm   |
| Alto       | 41.3 mm   |
| Ancho      | 55.6 mm   |
| Peso       | 110 g     |

Especificaciones técnicas:

| **Especificación**     | **Valor**                                                                          |
|------------------------|------------------------------------------------------------------------------------|
| Rango de distancia     | Blanco: 0,05-12 m (70 % de reflectividad); Negro: 0,05-6 m (10 % de reflectividad) |
| Frecuencia de muestreo | 5 kHz                                                                              |
| Resolución angular     | 0,72°                                                                              |
| Ángulo de inclinación  | 0°-1,5°                                                                            |

### Hi Wonder HPS-3527SG 35kg Servo

<!-- github-only-start -->
<p align="center">
	<img src="assets/images/components/hi-wonder-hps-3527sg-35kg-servo.webp" alt="Hiwonder HPS-3527SG 35kg Servo" 
width="350">
	<br>
	<i>Hiwonder HPS-3527SG 35kg Servo</i>
</p>

El Hiwonder HPS-3527SG 35kg Servo es el servomotor encargado de controlar la dirección de V-Titan, decidimos utilizar este modelo debido a su reducido tamaño y peso, además de una precisión más que suficiente para poder manejar a V-Titan.

No solo estos aspectos definieron la elección, el Hiwonder HPS-3527SG 35kg ofrece también una gran precisión a pesar de su reducido tamaño, algo esencial en esta competencia.

Gracias a la librería antes mencionada, la `adafruit_motor` con el módulo
`servo`, nos permiten configurar el servo a nuestra elección, convirtiendo el uso de funciones para controlar el servo previamente establecido mucho más fácil de leer sin arriesgar el rendimiento del programa.

| **Medida** | **Valor** |
|------------|-----------|
| Largo      | 40 mm     |
| Alto       | 28.8 mm   |
| Ancho      | 20 mm     |
| Peso       | 50 g      |

### HD Hex Motor

<p align="center">
	<img src="assets/images/components/hd-hex-motor.webp" alt="HD Hex Motor" 
width="350">
	<br>
	<i>HD Hex Motor</i>
</p>

Después de probar distintos modelos de motor, al final optamos por utilizar el motor HD Hex Motor, ya que éste cuenta con todos los requisitos que teníamos en mente para un motor (principalmente que cuente con un encoder y tenga una alta cantidad de RPM) ya que debido a nuestro sistema de transmisición, no era necesario que el motor cuente con un torque alto, ya que éste se puede compensar en nuestro sistema de transmisión con alguna relación de transmisión, además de ser un motor que ya se podía implementar con facilidad en el monochasis que habíamos diseñado, sólamente teniendo que cambiar su encaje.

| **Medida** | **Valor** |
|------------|-----------|
| Largo      | 77 mm     |
| Diámetro   | 37 mm     |
| Peso       | 234 g     |  

### IMU GY-BNO085

<p align="center">
	<img src="assets/images/components/bno08x.webp" alt="Giroscopio BNO085" 
width="350">
	<br>
	<i>Giroscopio BNO085</i>
</p>

El GY-BNO085 es nuestro sensor de orientación inercial (IMU). Lo usamos para que el robot mantenga rumbo en los cruces y cuente las vueltas dadas tanto en el Desafío sin Obstáculos como en el Desafío Cerrado, aunque exista algún problema mecánico que lo desvíe de su trayectoria.

**Cómo lo usamos (y cómo no).** El BNO085 no alimenta un PID de rumbo: alimenta la **pose**. Corre en modo UART-RVC a 100 Hz, una fusión interna de 6 ejes (giroscopio + acelerómetro, sin magnetómetro) que el chip calcula por sí mismo. Elegimos descartar el magnetómetro a propósito: sobre la pista conviven tres motores, un chasis metálico y la electrónica de potencia, y un rumbo por campo magnético sería vulnerable a todo eso. La contrapartida es la deriva del datasheet (~0.5°/min), que acotamos por otras vías (ver abajo). Esta decisión, con su comparación cuantitativa contra el modo de 9 ejes, está documentada en `platform/robot/docs/blind-navigation-evaluation.md`.

**Calibración y referencia de rumbo.** El modo RVC no expone rutinas de calibración al usuario: la calibración de gyro/acelerómetro la hace el chip en su arranque. Nuestra parte del proceso es la **referencia de yaw**, y es deliberadamente simple:

1. El robot se enciende y se coloca en la pose de salida (puede quedar girado 90° o 180° respecto al pasillo; es irrelevante).
2. Al presionar el botón de inicio, el estimador fija un desplazamiento (offset): ese rumbo pasa a ser 0°. Todo el yaw del robot es relativo a esa referencia (`reset_heading_reference` en `platform/robot/src/state_machine/estimator.py`).
3. Durante la ronda, el drift se acota con un filtro complementario contra el mundo "Manhattan" de la pista: cada pared es paralela o perpendicular al pasillo, así que el promedio circular de los ángulos medidos por el LIDAR recupera el heading absoluto y corrige la deriva del IMU.

La implementación maneja dos variables: `yaw_deg` (orientación relativa desde el inicio de la ronda) y `relative_yaw`, que acumula las vueltas sin saltar en ±180°. Dividiendo `relative_yaw` entre 90 y redondeando hacia abajo sabemos cuántos tramos rectos recorrió; cuando el cociente llega a ±12, el robot sabe que está en su zona de estacionamiento y avanza un poco más hasta detenerse (en el Desafío sin Obstáculos).

| **Medida** | **Valor** |
|------------|-----------|
| Largo      | 25.6 mm   |
| Alto       | 22.7 mm   |
| Ancho      | 4.6 mm    |
| Peso       | 3 g       |

### Ovonic Air 11.1V Li-Po Battery

<p align="center">
	<img src="assets/images/components/ovonic-air-11.1v-lipo-battery.webp" alt="Ovonic Air 11.1V Li-Po Battery" 
width="350">
	<br>
	<i>Ovonic Air 11.1V Li-Po Battery</i>
</p>

La batería de 11.1 V de la marca Ovonic es la fuente de alimentación principal: de ella se alimentan la Raspberry Pi 5 y todos sus componentes embebidos, además del motor de tracción. Usamos **dos modelos de la misma serie 3S**, con un rol distinto cada uno:

| **Característica** | **Competencia: Ovonic 3S Short 2200 mAh 120C** | **Prácticas: Ovonic 3S 3000 mAh 50C** |
|--------------------|------------------------------------------------|----------------------------------------|
| Voltaje nominal    | 11.1 V (3S1P, celdas 3.8-4.2 V)                | 11.1 V (3S)                            |
| Capacidad          | 2200 mAh (24.4 Wh)                             | 3000 mAh (33.3 Wh)                     |
| C-rating           | 120C                                           | 50C                                    |
| Conector           | XT60                                           | Deans (T-plug)                         |
| Dimensiones        | 77.17 × 34.06 × 25.12 mm                       | 107 × 24 × 33 mm                       |
| Peso               | 140 g                                          | 186 g                                  |

**Por qué dos.** La de 3000 mAh/50C es la batería de **prácticas**: más capacidad para sesiones largas de calibración y depuración sin recargas, a cambio de más peso y volumen. La de 2200 mAh/120C es la de **competencia**, en formato compacto ("shorty") y con conector XT60: menos capacidad, pero 46 g menos en la balanza (140 g contra 186 g) y un C-rating doble, que es lo que importa en pista.

**Por qué es suficiente.** El presupuesto de potencia del robot (ver la [sección de consumo energético](README.md#consumo-energ%C3%A9tico)) da un total nominal de ~14-16 A y picos de ~31 A, de los cuales la rama de tracción - el motor al 50% del ciclo de trabajo - aporta ~10 A nominales y ~20 A de pico, y el resto del sistema ~4-6 A. Con la batería de competencia:

- **Autonomía**: 2200 mAh contra ~14 A nominales sostenidos da ~9 minutos de operación continua a plena demanda. Una ronda completa dura pocos minutos y la tracción no exige su nominal el 100% del tiempo, así que el margen real es mayor; el límite práctico en un día de competencia no es la descarga de una ronda sino el ciclo de recargas entre rondas.
- **Corriente de pico**: el C-rating de 120C anunciado representa 264 A, cifra de marketing en condiciones ideales; incluso descontando la mitad por realismo continuo, la batería puede entregar más de 100 A, más de 3 veces el pico de ~31 A del presupuesto completo. La entrega de corriente no es el cuello de botella en ninguna parte del sistema.

Usar baterías más pequeñas no tiene sentido (el margen energético ya es holgado), y usar la de prácticas en competencia solo pagaría el peso y el volumen extra de una batería más grande, sin ningún beneficio en pista.

| **Medida** | **Valor** |
|------------|-----------|
| Largo      | 107 mm    |
| Alto       | 24 mm     |
| Ancho      | 33 mm     |
| Peso (medido, con conectores Deans) | 190 g |

> Las medidas son de la batería de prácticas; el peso medido con conectores (190 g) sobre el listado de 186 g explica la diferencia entre ambas cifras.

### Puente H BTS7960 / IBT-2

<p align="center">
	<img src="assets/images/components/h-bridge-bts7960.webp" alt="Puente H BTS7960 / IBT-2" width="350">
	<br>
	<i>Puente H BTS7960 / IBT-2 (el que monta V-Titan actualmente)</i>
</p>

El BTS7960 es el puente H que controla el motor de tracción. **No fue nuestra primera opción: reemplazó al L298N, y el motivo fue puramente de corriente.**

<p align="center">
	<img src="assets/images/components/puente-h-l298n.webp" alt="Puente H L298N" width="300">
	<br>
	<i>Puente H L298N - el diseño anterior, descartado por corriente insuficiente</i>
</p>

El L298N entrega **2 A por canal**. Cuando pasamos a medir realmente lo que consume el tren motriz con el motor actual, los números no cuadraban: la rama de tracción consume del orden de **10 A promedio** al 50% del ciclo de trabajo, con **picos instantáneos cercanos a 20 A** en los arranques y en los cambios de sentido. Eso es un orden de magnitud por encima de lo que el L298N puede sostener, y explicaba los cortes y el calentamiento que veíamos: el puente no estaba fallando, estaba operando muy por encima de su especificación.

El BTS7960 está clasificado a **43 A**, lo que deja un margen amplio incluso sobre los picos. La otra diferencia importante es la caída de tensión: el L298N usa transistores bipolares y pierde cerca de 2 V en el puente, mientras que el BTS7960 usa MOSFET y esa pérdida es mucho menor, de modo que llega más tensión útil al motor con la misma batería.

Este cambio también reordenó el análisis del resto de la ruta de potencia. Con el puente sobredimensionado, **el elemento más débil pasó a ser el interruptor de encendido**, cuya capacidad de conducción continua está muy por debajo del BTS7960 y de lo que puede entregar la batería. Lo dejamos documentado en el esquemático como el punto a vigilar, porque un componente sobredimensionado no elimina un cuello de botella: solo lo mueve de sitio.

| **Característica** | **L298N (anterior)** | **BTS7960 (actual)** |
|--------------------|----------------------|----------------------|
| Corriente máxima   | 2 A por canal        | 43 A                 |
| Tecnología         | Transistor bipolar   | MOSFET               |
| Caída en el puente | ~2 V                 | Muy baja             |
| Control            | ENA + IN1/IN2        | RPWM / LPWM independientes |

### Step Down Mini-560 Pro

<p align="center">
	<img src="assets/images/components/step-down-mini-560-pro.webp" alt="Step Down Mini-560 Pro" width="350">
	<br>
	<i>Step Down Mini-560 Pro (el que monta V-Titan actualmente)</i>
</p>

El Mini-560 Pro es el regulador que alimenta el riel propio del servo de dirección, separándolo del riel de 5V de la Raspberry Pi para que los picos de corriente del servo no lleguen al computador.

<p align="center">
	<img src="assets/images/components/step-down-xlc4016.webp" alt="Step Down XLC4016" width="300">
	<br>
	<i>Step Down XLC4016 - el regulador anterior, descartado por peso</i>
</p>

**También es un reemplazo, y aquí el criterio fue el peso.** El regulador original era un XLC4016, un módulo notablemente más grande y pesado. El peso fue un problema recurrente en nuestros prototipos (llegamos a estar 200 gramos por encima del límite), así que revisamos la lista de componentes buscando piezas que estuvieran sobredimensionadas para su función. El regulador del servo era una de ellas: la corriente que realmente necesita esa rama es muy inferior a lo que el XLC4016 podía entregar, de modo que estábamos pagando peso por una capacidad que nunca íbamos a usar.

El Mini-560 Pro cubre la demanda real del servo en un encapsulado mucho más compacto. La diferencia medida es de **24 g a 5 g: 19 gramos menos, casi un 80% del peso del módulo anterior**, por una capacidad que la rama del servo no necesitaba.

| **Regulador** | **Peso** |
|---------------|----------|
| XLC4016 (anterior) | 24 g |
| Mini-560 Pro (actual) | 5 g |
| **Diferencia** | **-19 g** |

Diecinueve gramos no ganan una carrera por sí solos, y ese es justamente el punto: **el peso no se recupera de un solo golpe, sino sumando decisiones pequeñas**. Llegamos a estar 200 g por encima del límite, y ninguna pieza individual explicaba esos 200 g. Resolver eso consistió en repetir este mismo ejercicio pieza por pieza (¿cuánta capacidad usa realmente esta rama, y cuánto peso estamos pagando por la que sobra?). Es el mismo razonamiento que aplicamos en la transmisión y en el chasis: **dimensionar cada pieza contra la carga medida, no contra el peor caso imaginable.**

### SSD1306 OLED Display

<p align="center">
	<img src="assets/images/components/ssd1306-oled-display.webp" alt="SSD1306 OLED Display" width="350">
	<br>
	<i>SSD1306 OLED Display 128x64</i>
</p>

Pantalla monocroma de 128x64 píxeles conectada por I2C. Cumple una función de diagnóstico en pista: muestra el estado de la máquina de estados, el desafío seleccionado por el jumper y el resultado del autodiagnóstico de arranque, sin necesidad de conectar un computador ni depender de la red. En una mesa de competencia, poder leer en qué estado está el robot antes de pulsar el botón de inicio evita arrancar una ronda con un sensor caído.

### Convertidor KL89576 (DC a USB-C)

Convertidor reductor que toma la tensión de la batería y entrega **5 V a 5 A por salida USB-C**, dedicado exclusivamente a la Raspberry Pi 5. Es una rama independiente de la del servo y la del motor: las tres cuelgan de la batería por separado, de modo que el consumo del tren motriz no puede provocar una caída de tensión en el computador y reiniciarlo a mitad de una ronda.

El dimensionamiento merece una aclaración, porque el pico de la tabla anterior suma por componente y aquí sería una suma engañosa. El AI HAT+, la cámara, el LIDAR y el puente IMU no se alimentan del KL89576 directamente: se alimentan del riel de 5 V de la propia Pi 5, y la Pi Zero entera (motor, nivel-shifter, OLED, encoder) recibe su alimentación por el VBUS del puerto USB de la Pi 5. Es decir, los 5 A de la especificación de la Pi 5 **ya incluyen** a todo lo conectado a la placa, y el pico del AI HAT+ (2.5 A) no se suma dos veces. El presupuesto real de la rama es: pico de la placa con sus periféricos (5 A, valor de especificación oficial que cubre el AI HAT+) más LIDAR (0.6 A) e IMU (0.03 A), ambos casi constantes, contra los 5 A del convertidor.

Ese margen es deliberadamente fino y lo monitoreamos en vez de sobredimensionarlo sin medir: el indicador `vcgencmd get_throttled` de la Pi 5 reporta cualquier caída de tensión, y es la misma señal con la que verificamos (0x0, sin eventos) que la Pi Zero alimentada por VBUS funciona sin caída de tensión (undervoltage) en carrera. Si el margen algún día se cerrara, el punto de vigilancia es el consumo conjunto placa+NPU, no el convertidor.

## Diagrama de Conexiones

El arnés completo de V-Titan está trazado como un esquemático generado por código, no dibujado a mano: la fuente reside en [`schemes/wiring/tscircuit/circuit.tsx`](schemes/wiring/tscircuit/circuit.tsx) y se exporta con [tscircuit](https://tscircuit.com/). Esto nos permite versionar el cableado igual que el resto del código: cualquier cambio de pin queda en el historial de git y el render se regenera desde la misma fuente.

<p align="center">
    <img src="schemes/wiring/harness.schematic.svg" alt="Diagrama de conexiones de V-Titan" width="1000">
    <br>
    <i>Arnés de conexiones de V-Titan - <a href="schemes/wiring/harness.schematic.png">versión PNG</a></i>
</p>

Para regenerar los artefactos tras editar `circuit.tsx`:

```bash
cd schemes/wiring/tscircuit
npm install
npm run artifacts   # netlist legible + SVG (fondo blanco) + PNG a 2400 px
```

Los exportados (`harness.schematic.svg` y `harness.schematic.png`) se comitean en `schemes/wiring/`, ya que son lo que se lee en esta documentación y reconstruirlos exige toda la cadena de herramientas de tscircuit.

### Consumo Energético

| **Componente**                    | **Cantidad** | **Voltaje** | **Corrente sin Carga** | **Corriente Nominal** | **Corriente Pico** |
|-----------------------------------|--------------|-------------|------------------------|-----------------------|--------------------|
| Raspberry Pi 5                    |      1       | 5.0V        | ~0.50A                 | ~1.50A - 2.50A        | 5.00A              |
| Raspberry Pi Zero 2W              |      1       | 5.0V        | ~0.10A                 | ~0.35A - 0.50A        | 0.70A              |
| Raspberry Pi Camera Module 3 Wide |      1       | 3.3V        | ~0.05A                 | ~0.25A                | 0.30A              |
| Raspberry Pi AI HAT+ (26 TOPS)    |      1       | 5.0V        | ~0.10A                 | ~1.00A - 1.50A        | 2.50A              |
| RPLiDAR C1                        |      1       | 5.0V        | ~0.20A                 | ~0.40A                | 0.60A              |
| Hi Wonder HPS-3527SG 35kg Servo   |      1       | 4.8V - 8.4V | ~0.02A                 | ~0.30A - 0.50A        | 1.80A (Stall)      |
| 9-Axis IMU Gyroscope GY-BNO085    |      1       | 3.3V - 5.0V | ~0.003A                | ~0.015A               | 0.03A              |
| Puente H BTS7960 / IBT-2          |      1       | 5V / 6-27V  | ~0.007A (Lógica)       | ~10.00A (tracción)    | ~20.00A (picos)    |
| **TOTAL**                         |    **8**     | **3.3V-5V** | **~0.980A**            | **~13.82A - 15.67A**  | **~30.93A**        |

> **Nota sobre la rama de tracción.** El salto respecto de la tabla anterior no es un cambio de consumo del robot, sino una corrección: el puente anterior figuraba con «según motor» en la columna nominal, de modo que la corriente de tracción, que es la mayor del sistema con diferencia, nunca entraba en el total. Los ~10 A nominales y ~20 A de pico son la rama del motor medida al 50% del ciclo de trabajo, y son exactamente el motivo por el que el L298N de 2 A por canal tuvo que ser reemplazado. El valor de 43 A del BTS7960 es la clasificación de la pieza, no un consumo: no se suma aquí.
>
> **Nota sobre la rama del computador.** Los picos de la Raspberry Pi 5 (5.00 A) y del AI HAT+ (2.50 A) **no se suman**: el AI HAT+ se alimenta del riel de 5 V de la propia Pi 5, y el pico de 5 A de la placa ya cubre por especificación a todo lo conectado a ella, incluida la Pi Zero, que recibe su alimentación por el VBUS de un puerto USB de la Pi 5. Los 5 A del KL89576 dimensionan esta rama completa; ver la [sección del convertidor](README.md#convertidor-kl89576-dc-a-usb-c).
>
> Estas tres ramas (computador, servo y tracción) se alimentan de la batería por separado a propósito. El total sirve para dimensionar la batería y el interruptor, no para dimensionar un único regulador.

### Calibración

Cada sensor del robot tiene una parte calibrada contra medición propia, no contra datasheet. Este es el inventario:

| Qué | Método | Valor |
|-----|--------|-------|
| Pulsos por vuelta del encoder | Cinta métrica: distancia conocida recorrida contra la que el robot cree haber recorrido (`task robot:calibrate-encoder`) | 60 pulsos/vuelta (el valor previo, 676, estaba mal por ~11x) |
| Ley motor-duty en banco | Motor cargado, duty barrido, rpm medidas contra cinta | $\text{rpm} = 434.6 \cdot \text{duty} - 86.7$ ($R^2 = 0.9999$); zona muerta en duty 0.200 |
| Referencia de yaw del IMU | Reset del offset al presionar el botón de inicio: ese rumbo pasa a ser 0° (`reset_heading_reference`) | Todo el yaw de la ronda es relativo a esa referencia |
| Calibración gyro/acelerómetro | Rutina del chip (modo RVC) en su arranque; no intervenimos | De fábrica |
| Latencia cámara→detección | Medida end-to-end sobre bags reales | **0.85 s** (runs 2026-09-06); el rango LIDAR del ciclo actual cubre el hueco |
| Rango de la visión | Modelo pinhole contra barrido LIDAR: cuando hay medición LIDAR al rumbo de la cámara, manda el LIDAR; el pinhole queda de respaldo | Error del pinhole: ~3.6 cm a 1.5 m, ~14 cm a 3 m |
| Radio de giro del chasis | Medido en banco | 0.29 m, usado como límite duro en simulación y control |
| Simulador | Ajustado contra grabaciones reales; conclusiones previas a la calibración descartadas | Ver [Simulador y corpus de escenarios](#simulador-y-corpus-de-escenarios) |

La consecuencia de método: ninguna constante del robot es un número "de fábrica" sin justificación; cada una de estas mediciones tiene una historia de hallazgo documentada en la [sección de hallazgos](#hallazgos-de-ingeniería).

# Movilidad y Diseño Mecánico

En este apartado se discuten todos los aspectos con lo que a movilidad y diseño se refiere, la evolución de éste, los prototipados realizados, etcétera.

## Métodos de Prototipaje

Para realizar nuestros prototipos, decidimos utilizar la impresión 3D como método principal, ya que ya éramos bastante familiares con todo el proceso, si bien el uso de máquinas CNC puede ser beneficioso para prototipos de esta categoría, decidimos optar por piezas pre-fabricadas o impresas en 3D, ya que nos permite minimizar el peso de V-Titan, ya que el peso fue un problema recurrente en nuestros primeros prototipos, llegando a estar 200 gramos por encima del límite establecido.

Para poder diseñar e imprimir dichas piezas, utilizamos el programa de diseño 3D SolidWorks, ya que tiene una gran cantidad de funciones útiles para el diseño de prototipos mecánicos, y, era el programa con el que teníamos mejor afinidad.

## Evolución y Justificación Del Diseño

### **Restricciones Iniciales**

* **Dimensiones y peso límite:** Máximo 300 mm (largo) 200 mm (ancho) 300 mm (alto) y un peso no mayor a 1500 g.

* **Reglamento de tracción y dirección:** Permitido tracción 4x4 impulsada por un **único motor** (o dos conectados en el mismo árbol de transmisión) y sistema de dirección para las 4 ruedas accionado por un **único servomotor**.

Con las reglas aclaradas, nuestras idea principal para la elección de componentes era que queríamos crear un prototipo lo más sencillo posible, es decir, tener la mayor cantidad de herramientas y funcionalidades en pista en la menor cantidad de componentes posibles, con esta idea en mente nos decidimos por implementar el [RPLiDAR C1](README.md#rplidar-c1) y el [Giroscopio BNO085](README.md#9-axis-imu-gyroscope-gy-bno085) como componentes principales para la navegación de V-Titan con el RPLiDAR delimitamos las paredes de la pista, y con el giroscopio obtenemos la orientación de V-Titan para una mejor autonomía a la hora de cruzar, además, optamos por usar la cámara [Raspberry Pi Camera Module 3 Wide](README.md#raspberry-pi-camera-module-3-wide) por su amplio rango de visión para detectar los obstáculos, para manejar este componente, utilizamos la [Raspberry Pi 5](README.md#raspberry-pi-5-16gb-ram) y el [Raspberry Pi AI HAT+ (26 TOPS)](README.md#raspberry-pi-ai-hat-26-tops) para manejar el modelo de detección de obstáculo. Con todo esto en mente, optamos por la [Raspberry Pi Zero 2W](README.md#raspberry-pi-zero-2-w) como microcontrolador para el manejo del [Motor](README.md#hd-hex-motor) y el [Servomotor](README.md#hi-wonder-hps-3527sg-35kg-servo) y, finalmente agregamos tanto la [Batería](README.md#ovonic-air-111v-li-po-battery) como el Adaptador a 5V DC para poder alimentar a la Raspberry Pi 5.

Con todos estos componentes en mente, queríamos implementar esta idea en un sistema de transmisión 4x4 con un sistema de dirección que permita generar el giro de 90 grados (o lo más cercano posible) hacia cualquier lado (izquierda o derecha) para permitir que la salida del estacionamiento en el Desafío Cerrado sea lo más fácil posible de programar, además de, cumplir con todas las reglas que tiene esta categoría, a través de pruebas y diseños, para efectos de esta documentación decidimos dividir el proceso en 4 fases:

#### **Fase 1: Prototipo de Rin Estático, Corona Interna y Guayas Flexibles**

<p align="center">
	<img src="assets/images/development/early-direction-system-design.webp" alt="Sistema de Transmisión" 
width="350">
	<br>
	<i>Primer Prototipo del Sistema de Dirección</i>
</p>

* **Mecanismo de Rueda:** Nuestro primer prototipo fue un rin estático que actúa como soporte/pivote en la tijera, mientras que el caucho exterior móvil incorpora una corona/cremallera interna accionada por piñones para transmitir tracción.

* **Transmisión de Dirección/Potencia:** Se implementaron **guayas flexibles** (tipo mototool/rotamil) para llevar el movimiento de rotación a la rueda soportando el ángulo extremo de 90 grados.

* **Caja de Engranajes Modular:** Diseñada para distribuir el movimiento de un solo motor hacia 4 guayas independientes.

* **Resultado:** Las pruebas aisladas confirmaron la viabilidad de la rotación y el pivoteo a 90 grados.

#### **Fase 2: Pruebas de Integración y Detección de Fallas**

<p align="center">
	<img src="assets/images/development/designing.webp" alt="Diseño CAD del sistema de dirección" 
width="350">
	<br>
	<i>Iteración de diseño en CAD entre prototipos impresos</i>
</p>

* **Sistema de Dirección:** Diseñamos una relación de palancas y piñones para la inversión de movimiento simultáneo. Se integraron **sensores Hall** para monitorear con precisión el ángulo de giro ante la necesidad de usar un servo de más de 360 grados.

* **Problemas Detectados:**
* Las barras de transmisión entre discos eran endebles, se doblaban e incluso una llegó a quebrarse.

* Las guayas generaban una tensión excesiva sobre el servomotor al ejecutar el giro.

* **Decisión:** Descartamos el sistema de guayas y palancas por ser complejo y pesado, buscando un mecanismo más ligero y directo.

#### **Fase 3: Rediseño a Engranajes Perpendiculares, Coronas y Correa Dentada**

<p align="center">
	<img src="assets/images/development/gear-direction-system-bottom-view.webp" alt="Sistema de dirección por engranajes, vista inferior" 
width="350">
	<br>
	<i>Sistema de dirección por engranajes, vista inferior: coronas integradas a los rines</i>
</p>

* **Nuevo Sistema de Tracción:** Eliminación de guayas. Se optó por **engranajes perpendiculares** ajustando el punto de pivote sobre el centro de la rueda, manteniendo los 90 grados de giro sin perder tracción.

* **Optimización de Dirección:**
* La primera prueba con líneas de piñones pequeños generó juego entre dientes (*backlash*) y movimiento errático.

* Se reemplazaron por una **corona más grande integrada al rin**, logrando una conexión directa y precisa accionada por el servomotor único.

* **Sincronización 4x4:** Se unificaron los árboles de transmisión delantero y trasero mediante una **correa dentada con poleas**, logrando accionar las 4 ruedas simultáneamente con un solo motor.

#### **Fase 4: Optimización de Peso, Integración y Chasis Final**

<p align="center">
	<img src="assets/images/development/IMG-20260825-WA0082.webp" alt="Integración de electrónica sobre el monochasis" 
width="350">
	<br>
	<i>Integración de la electrónica sobre el monochasis agujereado</i>
</p>

* **Distribución de Componentes:** Se diseñó una plataforma elevada para separar la electrónica de la mecánica. Esta posición permitió ubicar el RPLiDAR garantizando aproximadamente 270 grados de visión frontal y un espejo de visión trasera.

* **Control de Peso (1500 g):** Al ensamblar el conjunto, se detectó un exceso de 150g.

* **Acciones Correctivas:**
* Reducción de la densidad de relleno en la impresión 3D.

* Disminución de espesores de pared y creación de vacíos estructurales en el chasis, rines y bancadas sin comprometer la rigidez.

* **Resultado Final:** Se logró ingresar dentro del rango de peso reglamentario y consolidar un chasis rígido impreso en 3D con soportes dedicados para la electrónica.

## Sistema de Transmisión

<p align="center">
	<img src="assets/images/development/transmission-system-top-view.webp" alt="Sistema de Transmisión" 
width="350">
	<br>
	<i>Sistema de Transmisión, visto desde arriba</i>
</p>

Para poder diseñar nuestro sistema de transmisión, tuvimos que tener en cuenta nuestra meta inicial de nuestro alcance de dirección, para poder transmitir el movimiento del motor hacia las ruedas aún cuando éstas estén rotadas a un ángulo de 90 grados. 

Nuestro sistema de transmisión es un sistema 4x4, para maximizar la tracción en cada rueda, éste sistema es controlado por un único motor cuyo movimiento es transmitido mediante dos correas dentadas de movimiento (una para las ruedas delanteras, y otra para las ruedas traseras), este movimiento se va a su eje correspondiente (para el cual utilizamos unos pernos de transmisión de LEGO) y, a su vez cada eje transmite a dos sistemas de engranajes perpendiculares (uno por rueda) y este eje tiene un engranaje cónico perpendicular de 15 dientes, y este movimiento luego es transmitido directamente a la rueda (la cual en lugar de ser un caucho regular, recibe la tracción mediante sus dientes internos) de tal manera que cada rueda recibe la misma potencia, como último detalle, el rin cumple la función de ser un soporte para la rueda dentada y los engranajes cónicos perpendiculares.

## Sistema de Dirección

<p align="center">
	<img src="assets/images/development/direction-system-top-view.webp" alt="Sistema de Dirección" 
width="350">
	<br>
	<i>Sistema de Dirección, visto desde arriba</i>
</p>

Como ya se ha mencionado previamente, nuestra meta principal con nuestro sistema de dirección es tener un giro de 90 grados para facilitar la ruta en pista, para lograr esto, tuvimos que replantear la solución mecánica de Klevor desde cero. 

<p align="center">
	<img src="schemes/counter-phase-steering-system.png" alt="Ejemplo de sistema de dirección en Contrafase" 
width="350">
	<br>
	<i>Ejemplo de sistema de dirección en contrafase</i>
</p>

V-Titan cuenta con un sistema basado en un sistema de **dirección en contrafase**, el objetivo principal es que debido a que las ruedas traseras giran en el sentido opuesto a las delanteras se reduzca considerablemente el radio de giro, facilitando maniobras como el estacionamiento o giros cerrados (los cuales son bastante importantes en el Desafío Cerrado), ahora bien, este sistema se basa en que todo el movimiento es transmitido a través de engranajes, y los rines de las ruedas actúan tanto como soportes como actuadores en el movimiento al contar con una base dentada, aunque, al ser un sistema en que la tracción es transmitida a las 4 ruedas, es necesario contar con un servomotor con mucha capacidad de torque para poder ejercer la fuerza necesaria, razón por la cual, tuvimos que cambiar nuestro servo anterior, el cual tenía una capacidad de fuerza de 14kg·cm por uno de 35kg·cm. 

En cuanto al mecanismo, en primer lugar al servo le implementamos un eje de 20 dientes, el cual se conecta luego a otro engranaje de 20 dientes para transmitir ese mismo movimiento pero en dirección opuesta, cada engranaje de 20 dientes luego transmite su movimiento a un engranaje de 40 dientes, el cual conecta con el engranaje individual que conecta finalmente con cada rueda, ya sean delanteras o traseras.

<p align="center">
	<img src="models/current-models/blueprints/piñon-33-dientes-dirección.webp" alt="Piñon de 33 dientes de dirección" 
width="350">
	<br>
	<i>Piñon de 33 dientes de dirección</i>
</p>

También es importante recalcar la base dentada del rin de las ruedas, o mejor dicho, el piñon de dirección de la misma, debido a que el sistema de transmisión de V-Titan en lugar de utilizar engranajes diferenciales estándar, utiliza una transmisión por engranajes a cada rueda, lo que permite que la rueda pueda seguir recibiendo la tracción aún cuando está a 90 grados.

**Radio de giro: predicho contra medido.** El simulador originalmente permitía radios de giro virtualmente ilimitados (hasta ~8 mm), muy por debajo de lo que la geometría real puede cumplir. La medición en banco del chasis real fijó el radio mínimo en **0.29 m**, y ese valor vive ahora como límite duro (`MIN_TURN_RADIUS_M` en `platform/shared/config/`) tanto en la simulación como en el controlador: el simulador ya no aprueba curvas que el chasis no puede trazar. La consecuencia práctica se midió después sobre bags reales: entre 57 y 59% de los pasos del pure pursuit exigían un radio menor al que el chasis puede entregar, lo que disparaba el corte de velocidad por rumbo; el corrector que descarta puntos de mira inalcanzables (`MIN_TARGET_RADIUS_M`, medido y aceptado en A/B sobre 128 casos) nació de esa medición. Es la diferencia entre diseñar contra un chasis que existe y uno que no.

## Chasis Inferior 

<p align="center">
	<img src="models/current-models/blueprints/chasis-inferior.webp" alt="Chasis Inferior" 
width="350">
	<br>
	<i>Chasis Inferior</i>
</p>

Ahora bien, es hora de hablar del chasis inferior y de cómo los sistemas de transmisión y dirección son implementados en V-Titan, el aspecto más resaltante de este chasis es su forma agujereada, la cual, se fabricó de tal manera por las limitaciones de peso que nuestro primer prototipo tenía, además de esto, en el centro del chasis se pueden apreciar dos encajes, uno para el motor y otro para el servomotor, en los extremos del chasis también se pueden apreciar los encajes para los ejes de transmisión (para los cuales utilizamos pernos de LEGO) para asegurar una conexión rígida y estable entre los componentes y el chasis.

## Monochasis 

**Dimensiones.** El conjunto ensamblado mide **300 × 194 × 100 mm** (largo × ancho × alto, medidos), con margen sobre los límites reglamentarios de 300 × 200 × 300 mm. El peso final dependió de la batería: con la de prácticas y sus conectores Deans el conjunto quedó en **~1510 g**, apenas por encima del límite de 1500 g, y el paso a la batería de competencia (shorty XT60, 46 g menos) junto con el cambio de conectores lo bajó a **~1460 g**, dentro del límite con ~40 g de margen. La geometría que consume el control (distancia entre ejes (wheelbase) de 0.19 m, vía de 0.1675 m entre ruedas, ruedas de 0.07 m de diámetro) reside en `platform/shared/config/robot.toml` como fuente única, y es la misma que usan la simulación, la TF estática y el generador de Gazebo.

## Relación de Torque y Velocidad 

Ahora bien, en el caso de V-Titan, éste utiliza un [REV HD Hex Motor](README.md#hd-hex-motor), el cual tiene un torque de bloqueo (es decir, su torque máximo) de 0.105Nm, y una velocidad sin carga de 6000 RPM, ahora bien, ¿cómo podemos saber si este torque es necesario para mover a V-Titan?

La fórmula general para calcular el torque necesario es:

$$T = \frac{m \cdot \left( a + g \cdot \left( \mu \cos\theta + \sin\theta \right) \right) \cdot r}{N}$$

Donde:

- $m$ es la masa del vehículo (en kg; en V-Titan son **~1.51 kg con la batería de prácticas y ~1.46 kg con la de competencia**, medidos en el robot ensamblado). La simulación usa 1.5 kg fijos (`platform/shared/config/robot.toml`: chasis de 1.3 kg más 4 ruedas de 0.05 kg), un punto medio conservador entre ambas configuraciones: calcular con la masa mayor nunca subestima el torque necesario
- $r$ es el radio de la rueda (en metros; en V-Titan mide $0.035\ \text{m}$)
- $a$ es la aceleración deseada. La **medimos sobre bags MCAP de pista real**: la derivada de la velocidad del encoder (`/motor/drive_speed`) sobre 5 carreras recientes da una aceleración sostenida de **~1.0 m/s²** (muy consistente: 0.93-1.09 en los 5 bags) y una rampa de arranque desde reposo de **~0.4 m/s²**. Usamos $a = 1.0\ \text{m/s}^2$, el caso conservador
- $g$ es la gravedad, $9.81\ \text{m/s}^2$
- $\mu$ es el cociente de fricción (estimamos $0.3$ para ruedas de ASA sobre lona de PVC flexible)
- $\theta$ es el ángulo de inclinación ($\theta = 0°$ en esta competición)
- $N$ es el número de motores en tracción (en V-Titan solo hay uno)

Al efectuar toda la operación obtenemos como resultado que se necesita un torque mínimo de $0.207\ \text{Nm}$ para que V-Titan sostenga la aceleración medida ($1.0\ \text{m/s}^2$). Para referencia: con solo fricción ($a = 0$) el requerimiento baja a $0.155\ \text{Nm}$, y con la rampa de arranque ($0.4\ \text{m/s}^2$) a $0.176\ \text{Nm}$.

Así que, como el torque de bloqueo del motor ($0.105\ \text{Nm}$) es menor al torque mínimo ($0.207\ \text{Nm}$), es evidente que el motor por sí solo no podría mover a V-Titan sin utilizar algún método para aumentar el torque del motor de forma mecánica, la manera en la que resolvimos este problema es mediante las relaciones de engranajes, las cuales operan mediante la siguiente formula:

<p align="center">
	<img src="assets/images/misc/relacion-de-engranajes.webp" alt="Relación de Engranajes" 
width="350">
	<br>
	<i>Relación de Engranajes</i>
</p>

El torque final, o de salida será igual a la multiplicación del torque inicial por la misma relación de engranajes total, ahora, simplemente hay que calcular la relación de engranajes total de engranajes, para la cual simplemente calculamos cada relación individual y se efectúa el producto de ese conjunto:

| Etapa | Transmisión | Relación |
|-------|-------------|----------|
| 1 | Eje del motor (50 dientes) → correa hacia cada eje (33 dientes) | $i_1 = 33/50 = 0.66$ |
| 2 | Pernos de transmisión de LEGO con engranaje cónico de 10 dientes → engranaje de 20 dientes | $i_2 = 20/10 = 2$ |
| 3 | Engranaje de 20 dientes → engranaje de 15 dientes | $i_3 = 15/20 = 0.75$ |
| 4 | Engranaje de 15 dientes → engranaje de 20 dientes | $i_4 = 20/15 \approx 1.33$ |
| 5 | Engranaje de 20 dientes → rueda dentada de 50 dientes | $i_5 = 50/20 = 2.5$ |

La relación de transmisión total es el producto de las cinco etapas:

$$R_{total} = i_1 \cdot i_2 \cdot i_3 \cdot i_4 \cdot i_5 = 0.66 \cdot 2 \cdot 0.75 \cdot 1.33 \cdot 2.5 = 3.29$$

Y el torque de bloqueo final:

$$T_{final} = T_{stall} \cdot R_{total} = 0.105\ \text{Nm} \cdot 3.29 = 0.345\ \text{Nm}$$

Un éstandar, o mejor dicho, recomendación para los motores DC es utilizar el 50% de su torque de bloqueo para aceleraciones y tramos cortos, ahora bien, $0.345 \cdot 0.5 = 0.173\ \text{Nm}$, que queda por debajo del requerimiento con la aceleración sostenida medida ($0.207\ \text{Nm}$). Esto no invalida el diseño, y los bags lo confirman: la recomendación del 50% es para **funcionamiento continuo prolongado** (donde el calentamiento del devanado manda), mientras que la demanda real de una ronda es de tramos cortos de aceleración entre cruces; para eso están los picos de torque que los motores DC toleran por breves segundos. Contra el torque de bloqueo completo ($0.345\ \text{Nm}$), el margen es holgado incluso con $a = 1.0\ \text{m/s}^2$. La prueba final es empírica: los mismos bags de donde salió la aceleración muestran al robot sosteniendo esos $1.0\ \text{m/s}^2$ en pista, con este mismo motor y esta misma relación. Además, a medida que el vehículo gana velocidad, el cociente de fricción disminuye considerablemente (alrededor de un 15%), por lo que el torque necesario baja y es más fácil que el vehículo gane aceleración.

### Velocidad: teórica contra real

La mitad de velocidad de esta relación se verifica igual que el torque: predicción, medición, y explicación de la brecha.

**Techo cinemático.** Con el motor a su velocidad sin carga de 6000 RPM y la relación total $R_{total} = 3.29$, las ruedas girarían a $6000 / 3.29 \approx 1824$ RPM; con ruedas de 0.07 m de diámetro:

$$v_{teórico} = \frac{1824}{60} \cdot \pi \cdot 0.07 \approx 6.7\ \text{m/s}$$

**Medición en banco, con carga.** El motor nunca ve 6000 RPM en pista. La ley medida en banco (cargado, cinta métrica contra lo que el encoder cree recorrer) es afín: $\text{rpm} = 434.6 \cdot \text{duty} - 86.7$ ($R^2 = 0.9999$), con zona muerta en duty 0.200 y un techo físico de 348 RPM de rueda a duty 1.0, es decir **1.28 m/s**. El robot opera además con el ciclo de trabajo limitado al 50% por térmica, y los perfiles de velocidad de carrera (CREEP/SLOW/MEDIUM/FAST) viven dentro de ese presupuesto: la prealimentación afín `duty = 0.20 + 0.8 · rpm/max_rpm` (medida, con la misma zona muerta) les asigna duties de 0.295 a 0.419.

**Resultado en pista.** El techo real medido es **~0.58 m/s**. No es un límite físico del motor: es el resultado combinado del tope del 50% de duty, de la zona muerta con carga (20% del duty se gasta en vencer la fricción) y de los perfiles de velocidad que el gobernador impone. La brecha contra el techo cinemático (~11x) queda así explicada: es la diferencia entre el motor sin carga del datasheet y el motor cargado del banco con su ciclo de trabajo limitado. La cadena completa de esta medición (y del error de cuantización que antes la limitaba a 0.45 m/s) está en `platform/robot/config/hardware/motors/profiles/rev-hd-hex-motor-6000rpm/encoder.toml` y en la sección del [lazo de velocidad](#algoritmo-pid).

# Arquitectura de software y estrategia para superar obstáculos

En este apartado, describimos las estrategias que empleamos en pista para poder resolver los desafíos de una manera autónoma y eficiente.

## Arquitectura ROS2 y reparto entre dos computadores

V-Titan no corre sobre un solo computador, sino sobre dos, y el reparto no es por comodidad: es la decisión de arquitectura que sostiene todo lo demás.

La **Raspberry Pi 5** se encarga de percepción y planificación (LIDAR, cámara, inferencia en el AI HAT+, decidir hacia dónde ir), y la **Raspberry Pi Zero 2 W** se encarga exclusivamente del control en tiempo real del motor y del servo. El motivo es que esas dos cargas tienen exigencias temporales incompatibles. La inferencia de visión es pesada y su tiempo de respuesta varía; el lazo de control del motor tiene que ejecutarse a ritmo constante o el robot se vuelve inestable. Si ambas cosas compiten por el mismo procesador, un fotograma lento se traduce en una corrección de dirección tardía. Separándolas, **ningún retraso de visión puede detener el lazo de control**.

El software está organizado en cinco paquetes ROS2:

| Paquete | Responsabilidad |
|---------|-----------------|
| `vtitan_bringup` | Lanzamiento del sistema completo y composición de nodos |
| `vtitan_drivers` | Controladores de hardware: IMU, I2C, UART |
| `vtitan_navigation` | Navegación: seguimiento de pasillo, planificación, escapes |
| `vtitan_state_machine` | Máquina de estados de carrera y grabación de bags |
| `vtitan_vision` | Cámara e inferencia de detección |

Los nodos se comunican por **29 tópicos declarados en un único archivo de configuración** (`ros_topics.toml`) en lugar de estar escritos a mano en cada nodo. Esto evita una clase de error entera: un nodo que publica en `/lidar/scan` mientras otro escucha `/scan` compila, arranca y no funciona, sin ningún mensaje de error. Con los nombres centralizados, esa discrepancia no puede existir.

Un detalle que ilustra el nivel de restricción real: el SoC de la Pi Zero 2 W tiene **exactamente dos generadores de PWM por hardware**. Uno está tomado por el servo de dirección, que necesita mantener una posición absoluta y no tolera fluctuaciones. El otro se asigna a la marcha adelante del motor. La marcha atrás, que solo se usa en maniobras de estacionamiento y recuperación a baja velocidad, funciona con PWM por software y sí tolera esa fluctuación. Es un reparto deliberado de un recurso escaso, no una casualidad.

### La segunda pila (stack) en Go, y por qué no corre en carrera

Existe una segunda implementación de la pila en Go (con NATS como transporte en lugar de ROS2/DDS), y conviene ser explícitos sobre su estado: **no es la que compite**. La pila en carrera es la de Python + ROS2 descrita arriba, en todos los componentes: visión, navegación, máquina de estados y drivers en ambas placas.

La única pieza de Go que corre en producción es el **backend de telemetría** (`vtitan-backend.service`), un binario compilado que la Pi 5 sirve al panel de telemetría (dashboard).

La migración a Go se tomó como un reemplazo a largo plazo de ROS2 (arranque más rápido, menor consumo de recursos y de memoria en las placas), pero con una política deliberada: **migración en vía paralela con corte único, sin híbridos**. La pila de Python sigue siendo el de competencia y ahí continúan los ajustes de temporada; el de Go solo cortará a producción cuando alcance paridad completa, y entonces se conmutará de una vez con `vtitan-robot@go`, con la pila de Python documentada como camino de reversión. Al día de hoy, lo portado (incluida la navegación) está verificado contra bags de carreras reales en un arnés de paridad, pero el navegador de Go todavía no ha corrido dentro de un lazo completo de carrera en el robot, y ese es justamente el criterio de paridad que falta para el corte.

## Modelo de Detección YOLO

Para detectar los obstáculos del Desafío Cerrado de manera confiable usamos un detector YOLO entrenado por nosotros y compilado para el AI HAT+. Esta sección documenta el modelo completo: qué es, con qué datos se entrenó, cómo lo medimos y qué decisiones tomamos a partir de esas mediciones.

### El modelo y su pipeline

| Aspecto | Valor |
|---------|-------|
| Arquitectura | YOLOv11n (nano), 3 clases: prisma verde, magenta y rojo |
| Entrada | 640 × 640 × 3, UINT8 |
| Formato desplegado | ONNX compilado a HEF (Hailo-8) con Hailo Model Zoo |
| NMS | Embebido en el HEF, score 0.20, IoU 0.70 |
| Umbral de despliegue | 0.45 en el detector (las detecciones por debajo no llegan al navegador); 0.25 en el router de señales, para confirmación tardía |
| Rendimiento (throughput) | 101.5 FPS el HEF solo (`hailortcli run`); la cadena completa (captura → escala tipo letterbox → NPU → decodificación → publicación) corre a **15 Hz**, limitada por el temporizador de captura, no por el modelo |

Los primeros prototipos ejecutaban detección solo con CPU sobre la Raspberry Pi 5, a ~1-2 imágenes por segundo (~700 ms por imagen), demasiado lento para reaccionar a obstáculos a velocidad de carrera. El AI HAT+ movió la inferencia al NPU, y con ella reorganizamos el pipeline: el nodo de visión abre la cámara directamente y alimenta los fotogramas al NPU sin pasar por un intermedio de ROS para las imágenes, eliminando ese salto de la latencia.

### Datos de entrenamiento

El modelo actual se entrenó sobre **1,340 imágenes propias** de los prismas de la pista (verde, magenta y rojo), anotadas **manualmente con Label Studio** en formato YOLO. Es un conjunto de datos heredado de Klevor, que sigue siendo la base del detector actual.

En paralelo construimos el **apps/auto-annotator**, una herramienta de anotación asistida con SAM2 (orquestación en Go, servicio de ML en Python, frontend (interfaz) propio). No la usamos para el modelo actual: las anotaciones de este fueron a mano. La construimos pensando en la siguiente iteración del conjunto de datos, porque anotar 1,340 imágenes a mano fue la parte más lenta del entrenamiento y un modelo nuevo empieza por ahí. Las imágenes del conjunto de datos viven en el repositorio del apps/auto-annotator y sirven también como datos de calibración para la cuantización del HEF.

### Cómo lo medimos (y qué cambió por eso)

Evaluamos el modelo sobre 600 imágenes con IoU ≥ 0.5, comparando el resultado en punto flotante, tomado como referencia, contra dos variantes cuantizadas del compilador de Hailo:

| Variante | mAP@0.5 | mAP@0.5:0.95 | Clasificaciones erróneas | Omitidas |
|---|---|---|---|---|
| Punto flotante (referencia) | 0.9955 | 0.8885 | 0 | - |
| Nivel 0, la desplegada | 0.9954 | 0.8808 | 0 | 2 |
| Nivel 2 + QAT | 0.9689 | 0.8096 | 2 (magenta↔rojo) | 23 |

La decisión de desplegar la variante de nivel 0 salió directamente de esta tabla: la heurística "más optimización del compilador es mejor" era falsa para nuestro caso, y la variante de nivel 2, pese a llevar QAT, perdía mAP y, lo peor, introducía las únicas 2 confusiones entre clases del estudio.

Dos hallazgos de esta evaluación nos parecieron los más valiosos:

- **El orden de canales RGB/BGR casi pasa inadvertido.** Con el orden de canales equivocado, el mAP de la clase roja caía de 0.99 a **0.17**, y el sistema no falla de forma evidente: detecta "algo" con confianza razonable, solo que peor. Lo detectamos comparando mAP por clase entre variantes, no mirando imágenes.
- **Errar el color es peor que omitir la señal.** Clasificar un prisma rojo como verde invierte el lado de paso reglamentario; omitir la detección no lo hace, porque la red de seguridad en colisiones es el LIDAR, no la visión. Sobre 600 imágenes, el modelo desplegado jamás confundió rojo con verde y omitió 2 señales; los falsos positivos a umbral 0.25 fueron 119 (muchos atribuibles a etiquetado incompleto del conjunto de prueba), y el umbral de despliegue de 0.45 los suprime antes de que lleguen al navegador.

### Qué pasa cuando la visión falla

La visión no es la red de seguridad contra colisiones y la diseñamos como tal. Una detección falsa dentro del radio de activación (1.40 m) fuerza el lado de esquiva según su color, con el riesgo de una esquiva innecesaria; una detección omitida deja la esquiva sin invocar, pero el controlador de colisión por LIDAR sigue activo y los escapes escalan (retroceso y reintento) si el contacto ocurre igualmente. La máquina de estados, además, marca la visión como caída si deja de recibir detecciones dentro de su ventana de tiempo, de modo que una cámara o NPU averiada no pasa inadvertido en el autodiagnóstico de arranque.

## Algoritmo PID

El control de V-Titan tiene dos lazos con exigencias distintas, y solo uno de ellos es propiamente un PID. El de **velocidad** sí es un PI clásico sobre las RPM medidas por el encoder; el de **dirección** dejó de serlo: la ganancia proporcional pura resultó ser un lazo inestable a velocidad de carrera y fue reemplazada por *pure pursuit* basado en curvatura. Contar esa sustitución es, de hecho, la parte más instructiva de esta sección.

### Control de velocidad: PI sobre RPM

El lazo corre en la Raspberry Pi Zero 2 W con la señal del encoder. La clase `PIDController` implementa un PI con saturación de salida (límite de ciclo de trabajo en 50%) y anti-windup por integración condicional: el término integral solo acumula cuando la salida no está saturada, de modo que el windup no puede crecer contra el límite.

Sobre el PI va una **prealimentación (feedforward) afín** medida en banco, `duty = 0.20 + 0.8 · rpm/max_rpm`, con la zona muerta (deadband) medida con carga (`rpm = 434.6·duty − 86.7`). El lazo integral solo corrige lo que la prealimentación no modela; una consigna de cero devuelve ciclo de trabajo cero, así que el robot no sufre avance residual al detenerse.

Las ganancias son perfiles por motor y su historia ilustra por qué las constantes sin justificación dentro del código eran un problema. Al cambiar al HD Hex motor, el `counts_per_rev` correcto resultó ser 60 y no 676, lo que multiplicó la sensibilidad de la medición de RPM por ~8 y las ganancias viejas produjeron una oscilación visible: la velocidad oscilaba entre 2 y 21.5 RPM alrededor de una consigna de 13.6, con el duty oscilando de 0.15 a 0.31. Se reescalaron las ganancias en el mismo factor inverso (0.010→0.00125, 0.020→0.0025) para mantener constante la ganancia de lazo abierto, y se añadió un log por paso del PID (consigna, medida, ciclo de trabajo) para poder *ver* la oscilación en vez de inferirla de síntomas. Tras corregir además el feedforward (el `max_rpm` viejo dejaba el lazo apoyado contra su límite: la respuesta se estabilizaba a 1.33× la consigna con desviación estándar cero, la firma inequívoca de una saturación), el lazo sigue la consigna a ~2% en pista: tres vueltas limpias con 132.5 s frente a los 142.3 s previos al ajuste.

### Dirección: de PID a pure pursuit

La dirección de V-Titan no es un lazo P sobre error angular, aunque lo fue. Con `steering = kp · angle_error`, el sistema era estable solo por debajo de ~0.07 m/s: a velocidad de carrera, el lazo se volvía un oscilador no amortiguado que saturaba el servo entre −70.2° y +70.2° durante carreras completas. La causa tenía un detalle fino: la ganancia se había ajustado contra un modelo de simulación con dirección delantera, mientras el chasis real es de 4 ruedas direccionales en contrafase, que gira aproximadamente al doble de rápido para el mismo ángulo de servo.

La solución no fue ajustar la ganancia, sino cambiar la ley de control: **pure pursuit** sobre el punto de mira del camino planificado, con la distancia efectiva `L = wheelbase/2` para compensar el doble de tasa de guiñada del chasis en contrafase. La curvatura se convierte en ángulo de servo con saturación en ±70.2° y un limitador de tasa de 1.2 rad/s (bajado de 2.0 tras ver en un bag real que el controlador alcanzaba el límite de tasa en cada esquina, lo que en pista se percibía como una conducción demasiado brusca).

Dos refinamientos más, ambos dictados por evidencia de hardware:

- **Mezcla de la anticipación (lookahead) en vez de conmutación.** Los dos valores de anticipación (0.16 m corto, 0.32 m largo) conmutaban a ~2.5 Hz, y cada conmutación multiplicaba la curvatura por cuatro, produciendo un zigzag visible (pico medio de |steer| de 0.306 a 0.398 sin ganancia lateral real). Se reemplazó la conmutación por una rampa de mezcla continua.
- **Vista previa de esquina.** Con la señal de error lateral (una señal rezagada), el robot sostenía 0.9 rad de error de rumbo durante 3 s antes de reaccionar en las esquinas. Se añadió una vista previa geométrica de la pista a 0.80 m adelante para activar la anticipación corta antes, sin alargarla más porque otra prueba midió un tejido lateral de ±0.18 m con vista previa excesiva.

### El modo ciego: P de centrado eliminada por medición

En la fase inicial, antes de que la inferencia de dirección se estabilice, el robot sigue el pasillo solo con LIDAR. Ahí probamos un controlador P de dos términos (centrado + amortiguación de rumbo) y la ganancia de centrado resultó ser el peor error de ajuste del proyecto: con el centrado en 2.0, una barra de 128 escenarios perdió 12 casos su dirección y provocó 9 choques contra una pared; en hardware se midieron **112 inversiones de signo del steering en 177 s, con el 45% de los ciclos saturados en el límite**. La corrección fue eliminar el término de centrado (ganancia en 0) y quedarse solo con la amortiguación de rumbo: la misma barra pasó a 0 fallos y el avance lento inicial bajó de 6.7 s a 3.4 s. La lección registrada: corregir posición sin tener en cuenta el rumbo siempre sobrecorrige y el error reaparece, porque el steering fija la tasa de guiñada, no la posición.

### El rol del giroscopio

El BNO085 no alimenta un PID de rumbo: alimenta la **pose**. Su yaw relativo (ajustado por el desplazamiento de referencia al inicio de la ronda) se fusiona con odometría del encoder y con el LIDAR para producir la posición y rumbo que consume el pure pursuit; en el modo ciego entra solo por el término de amortiguación. En los cruces, el alineamiento con el eje del pasillo (medido contra el yaw del IMU) es lo que autoriza la velocidad normal, y un desalineamiento mayor a ~57° obliga a avance lento, que es donde vive la protección contra el sobrepaso que antes se le atribuía al PID.

## Estrategia en pista

El robot arranca **sin mapa y sin saber hacia qué lado se corre la pista**. Todo lo que sigue lo deduce de sus propios sensores durante los primeros metros. Los diagramas de flujo completos de esta lógica están en [`schemes/flowcharts/`](schemes/flowcharts/), separados en `common/` (lo compartido por ambos desafíos), `open/` y `obstacles/`.

### Inferencia del sentido de la vuelta

Es la primera decisión de cada ronda y condiciona todas las demás. El robot avanza despacio y centrado, y compara cuánto espacio libre mide el LIDAR a izquierda y derecha: el lado que **deja de ser pared** indica dónde está el bloque interior, y el bloque interior fija el sentido de giro.

<p align="center">
    <img src="schemes/flowcharts/common/webp/inferencia-direccion.webp" alt="Inferencia del sentido de la vuelta" width="700">
    <br>
    <i>Inferencia del sentido de la vuelta - fuente Mermaid: <a href="schemes/flowcharts/common/mermaid/inferencia-direccion.mmd"><code>inferencia-direccion.mmd</code></a></i>
</p>

Lo interesante no es la comparación, sino todo lo que hay que descartar antes de creerla. Una lectura solo cuenta como voto si supera cuatro filtros ([`inferencia-direccion.mmd`](schemes/flowcharts/common/mermaid/inferencia-direccion.mmd)):

1. **El chasis está alineado con el pasillo** (error menor a 25°). De lado, los rayos laterales cortan en diagonal y miden de más.
2. **Ningún rayo supera los 4.5 m.** En una pista de 3 m eso no puede ser una pared. Importa porque **un fallo de lectura del LIDAR se sustituye por el rango máximo**, que es exactamente la señal de «este lado está despejado» que el módulo busca: sin este filtro, un sensor sin respuesta parece un pasillo abierto.
3. **La suma de ambos lados supera 1.25 m.** La decisión se toma sobre la *suma*, no sobre cada rayo por separado, y este es el punto fino: dos paredes suman el ancho del pasillo sin importar dónde esté el robot entre ellas, así que la suma solo salta cuando un lado deja de ser pared. Comparar los rayos directamente no funciona: un robot desviado hacia el bloque interior lee 0.27 m a su izquierda y 0.72 m a su derecha, y «el lado más lejano está abierto» elige la pared exterior y devuelve exactamente la respuesta contraria.
4. **La diferencia entre lados supera 0.20 m**, para que el ruido no cuente como evidencia.

Y aun así una sola lectura no decide: hacen falta **5 votos coincidentes**. Un rayo que entra por la esquina de un bloque produce errores breves y agrupados, y uno de esos llegando primero no puede decidir la ronda.

### Seguimiento de pasillo, vueltas y escapes

Con el sentido resuelto, el robot sigue el pasillo manteniéndose centrado, cuenta las vueltas por el paso acumulado alrededor del circuito, y vigila permanentemente dos condiciones de fallo: **colisión** y **atasco**. Ambas comparten una misma rutina de escape, documentada una sola vez en `common/` y referenciada desde los dos desafíos en vez de redibujarse.

<p align="center">
    <img src="schemes/flowcharts/common/webp/conteo-vueltas.webp" alt="Conteo de vueltas" width="700">
    <br>
    <i>Conteo de vueltas por paso acumulado alrededor del circuito</i>
</p>

<p align="center">
    <img src="schemes/flowcharts/common/webp/escape-colision.webp" alt="Escape de colision y atasco" width="700">
    <br>
    <i>Rutina de escape compartida ante colisión y atasco</i>
</p>

<p align="center">
    <img src="schemes/flowcharts/common/webp/esquiva-generica.webp" alt="Esquiva generica" width="700">
    <br>
    <i>Esquiva genérica de obstáculo</i>
</p>

En el Desafío de Obstáculos se añade la regla de color: el robot debe pasar por un lado determinado de cada señal según sea roja o verde. La consecuencia de equivocarse no es perder puntos, es **terminar la ronda**, así que el criterio de paso es una de las partes más conservadoras del sistema.

<p align="center">
    <img src="schemes/flowcharts/obstacles/webp/regla-senales.webp" alt="Regla de paso por senales de color" width="700">
    <br>
    <i>Regla de paso por señales de color (Desafío de Obstáculos)</i>
</p>


### Vista completa de cada desafío

Los diagramas anteriores describen piezas sueltas de la lógica. Estos son los flujos completos y las máquinas de estado de cada desafío, renderizados desde las mismas fuentes Mermaid de [`schemes/flowcharts/`](schemes/flowcharts/).

<p align="center">
    <img src="schemes/flowcharts/open/webp/flujo-completo.webp" alt="Flujo completo del Open Challenge" width="800">
    <br>
    <i>Open Challenge - flujo completo</i>
</p>

<p align="center">
    <img src="schemes/flowcharts/open/webp/maquina-estados.webp" alt="Maquina de estados del Open Challenge" width="800">
    <br>
    <i>Open Challenge - máquina de estados</i>
</p>

<p align="center">
    <img src="schemes/flowcharts/obstacles/webp/flujo-parte1-conduccion.webp" alt="Obstacle Challenge, parte 1: conduccion" width="800">
    <br>
    <i>Obstacle Challenge - parte 1: conducción y señales</i>
</p>

<p align="center">
    <img src="schemes/flowcharts/obstacles/webp/flujo-parte2-estacionamiento.webp" alt="Obstacle Challenge, parte 2: estacionamiento" width="800">
    <br>
    <i>Obstacle Challenge - parte 2: estacionamiento</i>
</p>

<p align="center">
    <img src="schemes/flowcharts/obstacles/webp/maquina-estados.webp" alt="Maquina de estados del Obstacle Challenge" width="800">
    <br>
    <i>Obstacle Challenge - máquina de estados</i>
</p>
## Grabación y análisis de carreras

Una ronda dura como máximo **180 segundos** y no se puede pausar. Si algo sale mal, observar el robot no revela la causa. Por eso todo lo que ocurre a bordo queda grabado.

Cada ejecución escribe un *bag* en formato **MCAP** con todos los tópicos: barridos del LIDAR, pose estimada, comandos de dirección y velocidad, estado de la máquina de estados y detecciones de visión. Los bags se descargan del robot a `data/` y se analizan en frío, fuera de la pista.

Sobre esos bags corren **68 scripts de diagnóstico** especializados: uno reconstruye el conteo de vueltas, otro mide la sobrecorrección en las esquinas, otro compara la dirección inferida contra lo que realmente ocurrió, otro revisa la robustez de los rayos laterales. Para inspección visual, los bags se abren en **Foxglove**.

La diferencia práctica es grande: un fallo no se resuelve repitiendo la ronda y esperando que se manifieste de nuevo, sino **reproduciendo el instante exacto tantas veces como haga falta**, con los mismos datos, hasta encontrar la causa. Varios de los hallazgos listados más abajo salieron de un bag, no de la pista.

## Simulador y corpus de escenarios

Probar solo en la pista física tiene un límite duro: cada intento cuesta minutos, cada montaje es ligeramente distinto y no se puede repetir una situación difícil a voluntad. Por eso construimos un simulador y, sobre él, un **corpus de escenarios**.

Un escenario es una combinación concreta de posición de arranque, anchos de pasillo y sentido de la pista. El corpus los enumera de forma exhaustiva:

| Desafío | Escenarios | Qué varía |
|---------|-----------|-----------|
| Open Challenge | **640** | Posición de arranque, configuración de anchos, sentido de giro |
| Obstacle Challenge | **256** | Lo anterior, más la disposición y el color de las señales |

Esto cambia el significado de «funciona». Una mejora ya no se juzga por una carrera afortunada, sino por **cuántos de los 640 escenarios completa**. La referencia actual del Open Challenge es de **638 de 640**, y los dos casos restantes están identificados uno por uno, no agrupados como «ruido».

También cambia la forma de fallar. Cuando un cambio parece mejorar el resultado, el corpus permite comprobar *cuáles* casos concretos cambiaron de estado. Más de una vez, una mejora aparente resultó ser un caso que empezó a pasar y otro distinto que empezó a fallar, con el total sin moverse.

# Pensamiento sistémico y decisiones de ingeniería

Esta sección no describe qué hace el robot, sino **cómo tomamos las decisiones** que lo llevaron a ser así. Es la parte del proyecto que más nos cambió la forma de trabajar.

## Diseño gobernado por configuración

La regla que más impacto tuvo en la calidad del sistema es simple de enunciar: **ninguna constante de comportamiento vive dentro del código**. Todas están en archivos de configuración, y cada una tiene escrito al lado por qué vale lo que vale.

Hoy son **203 constantes repartidas en 21 archivos TOML**, acompañadas de **1093 líneas de comentario**: algo más de **cinco líneas de explicación por cada valor**.

No es documentación decorativa. Un número suelto en el código es imposible de auditar: nadie recuerda, tres meses después, si `0.20` se midió, se calculó o simplemente se estimó sin medir. Al obligarnos a escribir la justificación junto al valor, cada constante lleva su propia historia (qué se midió, con qué método, qué pasó cuando valía otra cosa). Un ejemplo real, del archivo que gobierna la inferencia de dirección:

```toml
# Diferencia mínima entre izquierda y derecha para que un barrido cuente como
# evidencia y no como ruido. Antes era 0.30, lo que descartaba la asimetría que
# realmente ve un robot que arranca cerca del centro de un pasillo ancho.
min_asymmetry_m = 0.20
```

Y otro, del perfil del motor actual, que muestra el caso contrario, una constante marcada explícitamente como *todavía no medida*:

```toml
# TODAVÍA NO MEDIDA EN BANCO. 0.234 es la velocidad medida del motor anterior
# (0.156 m/s) escalada por el "~50% más rápido" con que se describió este motor,
# y es la suposición que más peso carga en este archivo.
```

Escribir «esto todavía no está medido» dentro del propio sistema evita que una estimación se convierta silenciosamente en un hecho.

## Perfiles de hardware intercambiables

El robot cambió de servo y de motor durante el desarrollo. Para que eso no obligara a tocar el código, el hardware está descrito en **perfiles componibles**: uno por pieza física.

| Perfil | Pieza |
|--------|-------|
| `180deg-injora-14kg` | Servo de dirección de 180°, 14 kg |
| `270deg-hiwonder-35kg` | Servo de dirección de 270°, 35 kg (actual) |
| `generic-motor-1500rpm` | Motor de tracción de 1500 rpm |
| `rev-hd-hex-motor-6000rpm` | Motor de tracción HD Hex de 6000 rpm (actual) |

Se combinan al arrancar. Cambiar de servo es seleccionar otro perfil, no editar código, y, sobre todo, significa que **los dos servos siguen siendo probables** después del cambio: si el de 35 kg falla en competencia, volver al de 14 kg es una línea de configuración, no una tarde de reescritura.

## Ciclo de trabajo: idea → simulación → pista

Nuestro método de trabajo se estabilizó en tres pasos, y el orden importa:

1. **Formular la hipótesis antes de medir.** Escribir qué esperamos que cambie y en qué dirección, *antes* de ejecutar nada. Sin esto es demasiado fácil ejecutar, mirar el resultado y construir después una explicación que lo justifique.
2. **Contrastar contra el corpus completo**, no contra un caso. Un cambio se evalúa sobre los 640 escenarios, y se compara siempre contra la versión inmediatamente anterior, no contra una medición vieja tomada en otras condiciones.
3. **Verificar en la pista.** El simulador orienta; no decide. Solo la pista confirma.

Dos disciplinas que aprendimos a costa de errores:

**El simulador se calibra contra la realidad, no al revés.** Al comparar grabaciones reales con simuladas descubrimos que el simulador giraba más de lo que gira el robot y alcanzaba su velocidad al instante, cosa que el robot no hace. Era **optimista**: aprobaba comportamientos que en pista fallaban. Lo corregimos contra datos medidos y **descartamos las conclusiones anteriores a esa calibración**, porque estaban tomadas contra un robot que no existe.

**Un resultado solo es comparable dentro de sus propias condiciones.** Una misma configuración medida en dos momentos distintos puede dar resultados diferentes si algo del entorno cambió. Por eso las comparaciones se hacen en una sola ejecución y contra la versión inmediatamente anterior.

## Hallazgos de ingeniería

Los errores más costosos del proyecto no fueron de programación, sino **suposiciones que nadie había verificado**. Estos son los que más nos enseñaron:

| Hallazgo | Consecuencia |
|----------|--------------|
| El encoder daba **60 pulsos por vuelta, no 86** | Toda medición de distancia y velocidad estaba mal por ese factor. Se descubrió midiendo con cinta métrica una distancia conocida y comparándola con lo que el robot creía haber recorrido. |
| El «techo de 0.45 m/s» **no era un límite físico** | Era un artefacto del error anterior. Con el valor correcto, el techo real resultó ser **~0.58 m/s**. Estuvimos limitando el robot por un error de cuentas, no por el motor. |
| Un LIDAR montado invertido necesita **espejar las lecturas, no rotarlas 180°** | Rotar deja los ángulos invertidos en un sentido que parece plausible: el robot no falla de golpe, sino que interpreta mal la pista de forma sutil. Fue de los fallos que más costó localizar. |
| Un fallo de lectura del LIDAR **se sustituye por el rango máximo** | Es decir, un sensor sin respuesta se lee como «lado completamente despejado», justo la señal que usamos para decidir el sentido de la vuelta. Sin filtrarlo, el robot podía salir a dar vueltas al revés con total confianza. |
| El puente H **operaba diez veces por encima de su especificación** | Medir el consumo real del tren motriz (~10 A, con picos de ~20 A) contra los 2 A por canal del L298N explicó de golpe los cortes y el calentamiento. |
| Sobredimensionar una pieza **no elimina el cuello de botella** | Al pasar a un puente de 43 A, el elemento más débil de la ruta de potencia pasó a ser el interruptor de encendido. El límite se movió de sitio; no desapareció. |

El patrón es siempre el mismo: **el sistema se comportaba de forma coherente con una suposición equivocada**, y por eso los síntomas nunca apuntaban a la causa. La conclusión que sacamos, y que ahora aplicamos por defecto, es medir antes de optimizar.

## Gestión de riesgos

Riesgos identificados del robot, con su mitigación o su estado. Incluimos también los abiertos sin solución completa: declararlos es parte de gestionarlos.

| Riesgo | Impacto | Mitigación | Estado |
|--------|---------|------------|--------|
| Caída de tensión (undervoltage) en la Pi 5 (consumo conjunto placa + AI HAT+ cerca del margen) | Reinicios o reducción de frecuencia (throttling) en plena ronda | Presupuesto de potencia por riel; monitoreo con `vcgencmd get_throttled` | Vigilado |
| Fallo de la cámara o la NPU durante la ronda | Ciegas ante señales y obstáculos visuales | La visión está marcada como caída si no hay detecciones en su ventana; la colisión la cubre el LIDAR, no la visión | Mitigado |
| Lectura fallida del LIDAR que se reporta como rango máximo | El robot interpreta un lado despejado que no lo está | Cuatro filtros de voto + 5 votos coincidentes antes de inferir dirección | Mitigado |
| Lecturas fantasma del LIDAR (rangos alternando sin causa clara) | Navegación con datos esporádicamente erróneos | Los mismos filtros de voto absorben lecturas aisladas | **Abierto** - causa raíz sin identificar |
| Watchdog DDS: nodo vivo pero silencioso (matcheados sin datos) | Robot sin comandos con todo "conectado" | Watchdog de BOOT_CHECK (3 fallos antes de actuar) y reinicio coordinado; watchdog de motores auto-frena a 1 s sin comandos | Mitigado |
| Fallo del puente H o de la ruta de potencia | Pérdida de tracción | BTS7960 sobredimensionado (43 A); el eslabón débil actual es el interruptor de encendido | Mitigado - punto débil documentado |
| Pull-down físico ausente en `LPWM` del BTS7960 | Pulso de motor espurio al arrancar la Pi | `LPWM` solo se usa en reversa (estacionamiento/recuperación, jitter tolerado por diseño); la ruta de carrera usa `RPWM` por PWM de hardware | **Mitigado parcialmente** - pull-down físico en cola |
| Sobrepeso cerca del límite de 1.5 kg | Descalificación | Pieza por pieza contra carga medida | Vigilado |
| El simulador es optimista respecto a la pista real | Fallos en pista que la simulación no muestra | Calibración del simulador contra mediciones reales; ninguna conclusión se da por válida solo en sim | Mitigado parcialmente |
| Modo ciego con FOV limitado (~2.3 m) | 78 % de los fallos blind ocurren en la primera vuelta | Velocidad reducida, prioridad de paso estrecho por seguridad | Conocido - aceptado |
| Fallback de ronda equivocada (la ronda corrió como el challenge incorrecto, 2026-09-06) | Puntaje nulo en la ronda real | Re-muestreo del jumper en SYSTEM_RESET; timeout de 180 s | Mitigado tras el fallo |
| Peso del stack: arranque lento y servicios caídos al boot | Robot no listo al llamar a pista | Unidades systemd con `Restart`/`on-failure`; boot reducido de ~3 min | Mitigado |

## Tecnologías utilizadas

| Tecnología | Uso | Por qué |
|------------|-----|---------|
| **ROS2 Kilted** | Middleware de todo el robot | Comunicación entre nodos, herramientas de grabación y ecosistema ya maduro |
| **Python** | Navegación, visión, máquina de estados | Velocidad de iteración durante el desarrollo |
| **Go** | Backend de telemetría (en producción) y segunda implementación de la pila de navegación (en migración) | Reemplazo a largo plazo de ROS2: arranque más rápido y menor consumo de recursos en el robot; el corte a producción se hace cuando el stack de Go alcance paridad completa |
| **Pixi / RoboStack** | Entorno de desarrollo | Permite trabajar el mismo proyecto en Windows, Linux y en la Raspberry sin divergencias |
| **Gazebo** | Simulación física | Ejecutar el corpus de escenarios sin pista |
| **Hailo + YOLO** | Detección de señales | Inferencia en NPU: de ~700 ms por imagen en CPU a un pipeline de 15 Hz de punta a punta |
| **MCAP + Foxglove** | Grabación y análisis | Formato de bags y visualización posterior de cada ronda |
| **Task** | Automatización | Un único punto de entrada para compilar, probar, desplegar y simular |
| **tscircuit** | Esquemático de conexiones | El arnés se define en código y se versiona igual que el software |

# Videos de V-Titan

Finalmente, quisieramos invitarlos a revisar nuestro canal de Youtube, en el que subiremos contenido relacionado a V-Titan y los desafíos de la WRO.

Esta misma lista está disponible como [`video/video.md`](video/video.md), la carpeta que pide la categoría.

## Open Challenge

| [![Open Challenge #1](https://img.youtube.com/vi/xrTShkQfnUk/0.jpg)](https://youtube.com/shorts/xrTShkQfnUk) | [![Open Challenge #2](https://img.youtube.com/vi/28cxIb5Uug4/0.jpg)](https://youtu.be/28cxIb5Uug4) |
|:---:|:---:|
| **Open Challenge #1** | **Open Challenge #2** |
| [![Open Challenge #3](https://img.youtube.com/vi/JDZCLhUOZ_Q/0.jpg)](https://youtube.com/shorts/JDZCLhUOZ_Q) | [![Open Challenge #4](https://img.youtube.com/vi/wWfganqnq8A/0.jpg)](https://youtu.be/wWfganqnq8A) |
| **Open Challenge #3** | **Open Challenge #4** |
| [![Open Challenge #5](https://img.youtube.com/vi/0JTcstQ5lVM/0.jpg)](https://youtube.com/shorts/0JTcstQ5lVM) | [![Open Challenge #6](https://img.youtube.com/vi/tpZ2MUb4gyc/0.jpg)](https://youtu.be/tpZ2MUb4gyc) |
| **Open Challenge #5** | **Open Challenge #6** |

## Open Challenge Simulation

| [![Open Challenge Simulation Nº1](https://img.youtube.com/vi/S0tjWiyK1bM/0.jpg)](https://youtu.be/S0tjWiyK1bM) | |
|:---:|:---:|
| **Open Challenge Simulation Nº1** | |

## Obstacles Challenge Simulation

| [![Obstacles Challenge Simulation Nº1](https://img.youtube.com/vi/fb5zcayUf0A/0.jpg)](https://youtu.be/fb5zcayUf0A) | |
|:---:|:---:|
| **Obstacles Challenge Simulation Nº1** | |

## Parking Challenge

| [![Parking Challenge #1](https://img.youtube.com/vi/dpk2NokeFFs/0.jpg)](https://youtube.com/shorts/dpk2NokeFFs) | |
|:---:|:---:|
| **Parking Challenge #1** | |

## Otros

| [![Counter Phase Steering](https://img.youtube.com/vi/K51M7iB6rWM/0.jpg)](https://youtube.com/shorts/K51M7iB6rWM) | [![Previous Prototypes #1](https://img.youtube.com/vi/MQwCzlizyTI/0.jpg)](https://youtube.com/shorts/MQwCzlizyTI) |
|:---:|:---:|
| **Counter Phase Steering** | **Previous Prototypes #1** |
| [![Robot POV Nº1](https://img.youtube.com/vi/u9PNsfgKNgM/0.jpg)](https://youtu.be/u9PNsfgKNgM) | [![Foxglove Studio Replay](https://img.youtube.com/vi/c7y4DL4ijQ8/0.jpg)](https://youtu.be/c7y4DL4ijQ8) |
| **Robot POV Nº1** | **Foxglove Studio Replay** |