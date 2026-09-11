# Diagrama de lógica - Obstacle Challenge

Diagramas Mermaid que documentan el flujo de control del **Obstacle Challenge**, en lenguaje natural. Comparte toda la máquina del [Open Challenge](open-challenge-diagrama.md) - arranque en **modo ciego**, inferencia obligatoria de dirección, conducción normal, escape/atasco y conteo de vueltas - y agrega dos piezas exclusivas: la **salida de la bahía de estacionamiento** al arrancar y la esquiva de señales de tránsito. La maniobra de estacionamiento al final existe en el código pero **está apagada**; ver las notas.

La lógica compartida entre ambos desafíos (inferencia de dirección, vigilancia de colisión/atasco, conteo de vueltas) vive una sola vez en [`schemes/flowcharts/common/`](../../../schemes/flowcharts/common/mermaid/) y se referencia desde aquí, en vez de redibujarse.

El flujo completo se dividió en dos diagramas porque, de una sola pieza, resultaba demasiado largo para leerse cómodo: **Parte 1** cubre la salida de la bahía + conducción + señales + vueltas, **Parte 2** cubre el final de carrera (la parada en la sección de meta, que es lo que embarca, y el estacionamiento deshabilitado con sus subestados STAGE/ENTER/DONE - se eliminó un diagrama de estados aparte porque duplicaba exactamente esa misma información y quedaba saturado).

Cada archivo `.mmd` trae al inicio un bloque de comentarios `%%` (no se renderiza en la imagen) con las referencias exactas al código y a las constantes usadas, para poder ubicarlas rápido si algún valor cambia.

## Diagramas

Fuente Mermaid en `schemes/flowcharts/obstacles/mermaid/`, renders WebP en `schemes/flowcharts/obstacles/webp/` (generados con `task docs:diagrams`).

| Diagrama | Fuente `.mmd` | WebP |
|---|---|---|
| Máquina de estados de alto nivel (4 estados; no hay estado PARKING) | [`schemes/flowcharts/obstacles/mermaid/maquina-estados.mmd`](../../../schemes/flowcharts/obstacles/mermaid/maquina-estados.mmd) | `schemes/flowcharts/obstacles/webp/maquina-estados.webp` |
| Flujo - Parte 1: salida de la bahía, conducción, señales y vueltas | [`schemes/flowcharts/obstacles/mermaid/flujo-parte1-conduccion.mmd`](../../../schemes/flowcharts/obstacles/mermaid/flujo-parte1-conduccion.mmd) | `schemes/flowcharts/obstacles/webp/flujo-parte1-conduccion.webp` |
| Flujo - Parte 2: final de carrera (estacionamiento deshabilitado) | [`schemes/flowcharts/obstacles/mermaid/flujo-parte2-estacionamiento.mmd`](../../../schemes/flowcharts/obstacles/mermaid/flujo-parte2-estacionamiento.mmd) | `schemes/flowcharts/obstacles/webp/flujo-parte2-estacionamiento.webp` |
| Detalle - regla de paso por señal (relativa al sentido de marcha) | [`schemes/flowcharts/obstacles/mermaid/regla-senales.mmd`](../../../schemes/flowcharts/obstacles/mermaid/regla-senales.mmd) | `schemes/flowcharts/obstacles/webp/regla-senales.webp` |

### Detalle común (compartido con Open Challenge)

| Diagrama | Fuente `.mmd` | WebP |
|---|---|---|
| Inferencia de dirección (horario vs. antihorario) | [`schemes/flowcharts/common/mermaid/inferencia-direccion.mmd`](../../../schemes/flowcharts/common/mermaid/inferencia-direccion.mmd) | `schemes/flowcharts/common/webp/inferencia-direccion.webp` |
| Vigilancia de colisión y atasco | [`schemes/flowcharts/common/mermaid/escape-colision.mmd`](../../../schemes/flowcharts/common/mermaid/escape-colision.mmd) | `schemes/flowcharts/common/webp/escape-colision.webp` |
| Esquiva genérica ante un obstáculo | [`schemes/flowcharts/common/mermaid/esquiva-generica.mmd`](../../../schemes/flowcharts/common/mermaid/esquiva-generica.mmd) | `schemes/flowcharts/common/webp/esquiva-generica.webp` |
| Conteo de vueltas | [`schemes/flowcharts/common/mermaid/conteo-vueltas.mmd`](../../../schemes/flowcharts/common/mermaid/conteo-vueltas.mmd) | `schemes/flowcharts/common/webp/conteo-vueltas.webp` |

## Notas

### Correcciones de fondo (2026-09-10)

Tres afirmaciones centrales de la versión anterior de esta página eran falsas. Se dejan escritas porque el error costó dos meses y conviene que no se reintroduzca.

- **La regla de paso SÍ depende del sentido de la vuelta.** Esta página afirmaba lo contrario ("rojo siempre por afuera, verde siempre por adentro") citando el código como prueba. La regla WRO 2026 9.19 es **relativa al sentido de marcha**: el vehículo pasa a su **propia derecha** de un pilar rojo y a su **propia izquierda** de uno verde. Como "su derecha" es la pared exterior en antihorario y el cuadro interior en horario, en coordenadas de pista la regla se invierte entre sentidos - y por eso las 4 filas `CLOCKWISE` de `routing.py::ROUTING_TABLE` son la **negación** de sus pares `COUNTERCLOCKWISE`. La tabla fue absoluta entre 2026-07-05 y 2026-09-03: correcta en antihorario y al revés en **toda** ronda horaria. Sobrevivió dos meses porque el scorer del simulador juzgaba con la misma convención que el router conducía, así que el simulador se calificaba contra su propio error.
- **Y por lo tanto la regla NO se cumple desde el primer momento.** `pass_side_lateral_axis()` exige la dirección y devuelve `None` sin ella; su docstring obliga al llamador a tratar ese `None` como "la regla no está disponible en este tick" y caer a esquiva genérica. Durante la inferencia de dirección una señal se esquiva **como un obstáculo cualquiera**, sin mirar el color. Es una pérdida de capacidad real, no un detalle de redacción.
- **El estacionamiento no se ejecuta.** `parking.toml` fija `attempt_after_final_lap = false`: tras la última vuelta el robot se detiene en la sección de meta y nunca entra al hueco. La bahía es geométricamente inalcanzable - 0.194 m de chasis contra 0.20 m de hueco, 3 mm por lado, menos que el error de pose. Sobre 256 escenarios, perseguirla no ganó **ni una vuelta** (`laps>=3` 159 en ambos brazos) y costó 96 rondas en tiempo y 47 colisiones. El `ParkController` sigue en el árbol y el diagrama de la Parte 2 lo dibuja como bloque deshabilitado.

### Cómo funciona hoy

- El robot arranca **dentro de la bahía de estacionamiento** (`assume_bay_start = true`, desde 2026-09-05): es el arranque que vale 7 puntos. Antes de poder inferir nada tiene que salir del hueco, y esa maniobra es la Fase 0 de la Parte 1. Asumir la bahía es seguro aunque sea falso: un arranque en paralelo tiene espacio libre adelante y la creencia se descarta en el tick siguiente.
- **La salida es un trinquete contra la pared exterior**: piernas alternas adelante/atrás a tope de volante, con el volante **espejado** en el retroceso. Sin espejar, el retroceso rehace el arco de la ida y la maniobra es un péndulo que no rota - ese era el fallo, y corregirlo llevó la fase de 62.2 s a 24.1 s, y a 10.4 s con el presupuesto de frenado medido. Cada pierna se corta por **clearance predicha** contra las aletas, nunca por contacto: terminar una pierna tocando una aleta es la violación 9.24.7 y anula todos los puntos de estacionamiento. El criterio de salida es la **rotación acumulada** (70°), no el espacio libre adelante, porque dentro del hueco la pared cae por debajo del rango mínimo del LIDAR.
- Salvo por eso, el arranque es el mismo que el del Open Challenge: modo ciego, sin dirección ni mapa conocidos (ver [open-challenge-diagrama.md](open-challenge-diagrama.md)).
- El reconocimiento de señales nace ciego: una vez fijada la dirección, empieza vacío y se va llenando en vivo conforme la cámara detecta señales - no hay mapa de señales previo.
- **Lo que mueve la trayectoria es el planificador de carril, no la deformación por waypoint.** `sign_lane_planner = true` y `sign_lane_suppress_deform = true`, así que quien manda es `apply_sign_lanes()`: un carril lateral con meseta plana de 0.25 m a la altura de la señal y rampas de 0.9 m a cada lado. El diagrama `regla-senales.mmd` dibujaba el mecanismo suprimido.
- El carril se activa progresivamente entre 1.40 m y 1.60 m de distancia. **Pero nunca recibe esa anticipación**: medido sobre bags de hardware, la cámara ve la señal a 0.66 m y el router se compromete a 0.38 m, así que la maniobra se ejecuta con una fracción del recorrido para el que fue diseñada. No es un fallo de la regla ni del carril: es falta de distancia.
- Una vez elegida una señal, se mantiene **esa** hasta superarla (`commit_hysteresis = true`) en vez de recalcular la más cercana cada tick. Con dos señales a 0.50 m en un corredor de 1.0 m, la carrera por tick cambiaba de ganador a mitad de la aproximación y saltaba la línea lateral comandada sin recorrido para seguirla.
- El anclaje a la altura de la señal (nodo de borde punteado en el diagrama) es un paso opcional, no obligatorio.
- Se ignora temporalmente el eco del LIDAR sobre una señal que ya se está esquivando, para que esa señal no dispare además el escape reactivo por colisión.
- Las señales se rearman cada vuelta: hay que esquivarlas en las 3 vueltas, no solo en la primera.
- **No existe un estado `PARKING`.** El enum `RobotState` tiene exactamente 4 valores y `_VALID_TRANSITIONS` no contiene ninguna arista hacia él; el estacionamiento siempre fue una fase interna del navegador dentro de `RACING`. El diagrama de estados lo dibujaba como estado propio.
