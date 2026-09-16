# Esquemas y diagramas

Diagramas de flujo, máquinas de estado y el esquemático del arnés. **La
explicación vive en el [README principal](../README.md)**; aquí solo está el
mapa de qué archivo es cada cosa y cómo se regenera.

```text
schemes/
├── ackermann-steering-system.webp      geometría Ackermann (la alternativa que NO usamos)
├── counter-phase-steering-system.webp  dirección en contrafase (la que sí usamos)
├── flowcharts/
│   ├── common/      lógica compartida y arquitectura de software
│   ├── open/        Open Challenge
│   ├── obstacles/   Obstacle Challenge
│   └── _legacy/     diagramas de versiones anteriores, conservados
└── wiring/
    ├── harness.schematic.svg  esquemático del arnés (y su versión .png)
    └── tscircuit/             el proyecto que lo genera
```

## Los diagramas de flujo

Cada diagrama existe **tres veces**, y las tres salen del mismo `.mmd`:

| Dónde | Para qué |
|---|---|
| `flowcharts/<grupo>/mermaid/*.mmd` | **La fuente única.** GitHub la renderiza al abrir el archivo |
| `flowcharts/<grupo>/webp/*.webp` | Render estático, para PDF o para leer sin conexión |
| Bloques ` ```mermaid ` en el README principal | Para que el diagrama se vea sin salir del documento |

`common/` contiene lo que comparten los dos desafíos (inferencia del sentido de
la vuelta, escape ante colisión y atasco, conteo de vueltas, esquiva genérica,
interacciones entre subsistemas). `open/` y `obstacles/` lo **referencian en vez
de redibujarlo**, que es la razón de que exista la separación.

Seis de los diagramas de `common/` no describen comportamiento sino
**estructura**, y conviene leerlos en este orden:

| Archivo | Qué dibuja |
|---|---|
| `subsistemas.mmd` | Hardware: energía y datos entre batería, cómputo, sensores y actuación |
| `paquetes-ros2.mmd` | Los cinco paquetes ROS2 y en qué placa corre cada uno |
| `nodos-ros2.mmd` | Plano de carrera: los 8 nodos que deciden y actúan, tópico a tópico |
| `nodos-ros2-telemetria.mmd` | Plano de observación: bags, OLED y el puente, que solo escuchan |
| `oled-paginas.mmd` | Dentro de `oled_display_node`: la cadena de prioridad que elige la página |
| `oled-fuentes.mmd` | Qué tópico alimenta cada línea de cada página del OLED |

Las aristas de `nodos-ros2*.mmd` salen de un `create_publisher` o
`create_subscription` real, y los nombres de tópico de `src/config/ros_topics.toml`
(la fuente única, ADR 0017). Si se añade o renombra un tópico, esos dos
diagramas son lo que hay que actualizar junto al código. Los dos de `oled-*`
siguen a `vtitan_drivers/oled_display_node.py`: `_update_display` para el árbol
de decisión y los métodos `_render_*` para las líneas de cada página.

Los cinco **solo dibujan lo que corre en competencia**. El backend de telemetría
en Go y su panel quedan fuera a propósito: en pista el robot no tiene red, así
que ese enlace no existe durante una ronda y dibujarlo sugeriría una dependencia
que no hay. Cada `.mmd` lo declara en sus notas `%%`, que no se renderizan.

Las líneas que empiezan por `%%` dentro de un `.mmd` son notas de mantenimiento
(referencias a código, cifras de barridos). No se renderizan nunca y se
descartan al inlinear en el README.

### Regenerar

```bash
task docs:diagrams     # .mmd -> WebP (requiere mermaid-cli vía npx, y cwebp)
task docs:mermaid      # .mmd -> bloques inlineados del README principal
task docs:mermaid -- --check   # falla si algún bloque quedó desincronizado
```

**Si editas un `.mmd`, corre las dos.** La primera actualiza el render, la
segunda la copia del README. `--check` es lo que debería llamar CI para que la
copia no derive nunca de su fuente.

## El arnés

El esquemático de conexiones no está dibujado a mano: se define en código con
[tscircuit](https://tscircuit.com/) en `wiring/tscircuit/circuit.tsx`, así que
un cambio de pin queda en el historial de git como cualquier otro cambio.

```bash
cd schemes/wiring/tscircuit
npm install          # solo la primera vez
npm run artifacts
```

Los exportados (`harness.schematic.svg` y `harness.schematic.png`) se versionan
porque son lo que se lee en la documentación y reconstruirlos exige toda la
cadena de herramientas. El `.png` se conserva como respaldo universal del `.svg`;
esa es la única razón por la que no se convirtió a WebP como el resto.
