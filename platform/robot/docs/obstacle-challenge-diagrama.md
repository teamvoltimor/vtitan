# Diagrama de lógica — Obstacle Challenge

Diagramas Mermaid que documentan el flujo de control del **Obstacle Challenge**, en lenguaje natural. Comparte toda la máquina del [Open Challenge](open-challenge-diagrama.md) — arranque en **modo ciego**, inferencia obligatoria de dirección, conducción normal, escape/atasco y conteo de vueltas — y agrega dos piezas exclusivas: la esquiva de señales de tránsito y la maniobra de estacionamiento.

La lógica compartida entre ambos desafíos (inferencia de dirección, vigilancia de colisión/atasco, conteo de vueltas) vive una sola vez en [`docs/schemes/flowcharts/common/`](../../../docs/schemes/flowcharts/common/mermaid/) y se referencia desde aquí, en vez de redibujarse.

El flujo completo se dividió en dos diagramas porque, de una sola pieza, resultaba demasiado largo para leerse cómodo: **Parte 1** cubre conducción + señales + vueltas, **Parte 2** cubre el estacionamiento (subestados STAGE/ENTER/DONE incluidos ahí mismo — se eliminó un diagrama de estados aparte porque duplicaba exactamente esa misma información y quedaba saturado).

Cada archivo `.mmd` trae al inicio un bloque de comentarios `%%` (no se renderiza en la imagen) con las referencias exactas al código y a las constantes usadas, para poder ubicarlas rápido si algún valor cambia.

## Diagramas

Fuente Mermaid en `docs/schemes/flowcharts/obstacles/mermaid/`, renders WebP en `docs/schemes/flowcharts/obstacles/webp/` (generados con `task docs:diagrams`).

| Diagrama | Fuente `.mmd` | WebP |
|---|---|---|
| Máquina de estados de alto nivel (incluye estacionamiento) | [`docs/schemes/flowcharts/obstacles/mermaid/maquina-estados.mmd`](../../../docs/schemes/flowcharts/obstacles/mermaid/maquina-estados.mmd) | `docs/schemes/flowcharts/obstacles/webp/maquina-estados.webp` |
| Flujo — Parte 1: conducción, señales y vueltas | [`docs/schemes/flowcharts/obstacles/mermaid/flujo-parte1-conduccion.mmd`](../../../docs/schemes/flowcharts/obstacles/mermaid/flujo-parte1-conduccion.mmd) | `docs/schemes/flowcharts/obstacles/webp/flujo-parte1-conduccion.webp` |
| Flujo — Parte 2: estacionamiento | [`docs/schemes/flowcharts/obstacles/mermaid/flujo-parte2-estacionamiento.mmd`](../../../docs/schemes/flowcharts/obstacles/mermaid/flujo-parte2-estacionamiento.mmd) | `docs/schemes/flowcharts/obstacles/webp/flujo-parte2-estacionamiento.webp` |
| Detalle — regla de paso por señal (rojo vs. verde) | [`docs/schemes/flowcharts/obstacles/mermaid/regla-senales.mmd`](../../../docs/schemes/flowcharts/obstacles/mermaid/regla-senales.mmd) | `docs/schemes/flowcharts/obstacles/webp/regla-senales.webp` |

### Detalle común (compartido con Open Challenge)

| Diagrama | Fuente `.mmd` | WebP |
|---|---|---|
| Inferencia de dirección (horario vs. antihorario) | [`docs/schemes/flowcharts/common/mermaid/inferencia-direccion.mmd`](../../../docs/schemes/flowcharts/common/mermaid/inferencia-direccion.mmd) | `docs/schemes/flowcharts/common/webp/inferencia-direccion.webp` |
| Vigilancia de colisión y atasco | [`docs/schemes/flowcharts/common/mermaid/escape-colision.mmd`](../../../docs/schemes/flowcharts/common/mermaid/escape-colision.mmd) | `docs/schemes/flowcharts/common/webp/escape-colision.webp` |
| Esquiva genérica ante un obstáculo | [`docs/schemes/flowcharts/common/mermaid/esquiva-generica.mmd`](../../../docs/schemes/flowcharts/common/mermaid/esquiva-generica.mmd) | `docs/schemes/flowcharts/common/webp/esquiva-generica.webp` |
| Conteo de vueltas | [`docs/schemes/flowcharts/common/mermaid/conteo-vueltas.mmd`](../../../docs/schemes/flowcharts/common/mermaid/conteo-vueltas.mmd) | `docs/schemes/flowcharts/common/webp/conteo-vueltas.webp` |

## Notas

- El robot arranca igual que en el Open Challenge: modo ciego, sin dirección ni mapa conocidos, resolviendo el sentido de giro antes de poder navegar (ver [open-challenge-diagrama.md](open-challenge-diagrama.md)).
- La maniobra de "retroceder y girar hacia el lado más despejado" se repite tanto al inicio (esquiva de un obstáculo cualquiera durante la inferencia de dirección) como en pleno trayecto (vigilancia de colisión durante la conducción normal), así que vive una sola vez en `esquiva-generica.mmd` en vez de redibujarse en cada fase.
- **La regla roja/verde se cumple desde el primer momento, incluso antes de fijar la dirección**: el robot igual tiene que avanzar y, si se topa con una señal, la esquiva por su lado correcto. Esto es posible porque la regla WRO roja=afuera/verde=adentro **no depende del sentido de la vuelta** — confirmado en el código: `sign_router.py::_ROUTING_TABLE` da el mismo eje y multiplicador para horario y antihorario en las 4 secciones. Así lo muestra el diagrama `inferencia-direccion.mmd` (rama "¿Es una señal roja o verde?").
- El reconocimiento de señales nace ciego: una vez fijada la dirección, empieza vacío y se va llenando en vivo conforme la cámara detecta señales — no hay mapa de señales previo.
- **La regla de paso es fija, no depende del sentido de la vuelta**: rojo siempre se pasa por afuera del cuadro central, verde siempre por adentro. El desplazamiento se activa progresivamente entre 1.40 m y 1.60 m de distancia a la señal. El anclaje a la altura de la señal (nodo de borde punteado en el diagrama) es un paso opcional, no obligatorio.
- Se ignora temporalmente el eco del LIDAR sobre una señal que ya se está esquivando, para que esa señal no dispare además el escape reactivo por colisión.
- Las señales se rearman cada vuelta: hay que esquivarlas en las 3 vueltas, no solo en la primera.
- El estacionamiento solo se activa una vez, tras la última vuelta, cuando el robot está en el corredor correspondiente y a 0.45 m del punto de espera; antes de eso sigue conduciendo con normalidad.
- Prioridad explícita al entrar al hueco: **no chocar por encima de estacionar**. Si el robot rompería la pared o un poste de la bahía, la maniobra se detiene ahí en vez de forzar la entrada.
- Durante todo el estacionamiento se vigila el espacio libre alrededor (360°) en cada instante y se frena por precaución si baja del margen de seguridad, porque la geometría de la maniobra puede rozar una pared o bloque desde cualquier ángulo.
