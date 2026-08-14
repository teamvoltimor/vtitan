# Diagrama de lógica — Open Challenge

Diagramas Mermaid que documentan el flujo de control del **Open Challenge**, en lenguaje natural. El robot corre siempre en **modo ciego**: al iniciar no conoce la dirección de la pista (horario/antihorario) ni tiene mapa, así que la primera fase obligatoria es un avance reactivo que infiere la dirección observando la asimetría del pasillo con el LIDAR. No hay señales de tránsito ni maniobra de estacionamiento — eso corresponde al Obstacle Challenge, documentado en [`obstacle-challenge-diagrama.md`](obstacle-challenge-diagrama.md).

La lógica compartida entre ambos desafíos (inferencia de dirección, vigilancia de colisión/atasco, conteo de vueltas) vive una sola vez en [`diagrams/common/`](diagrams/common/mermaid/) y se referencia desde aquí, en vez de redibujarse.

Cada archivo `.mmd` trae al inicio un bloque de comentarios `%%` (no se renderiza en el PNG) con las referencias exactas al código y a las constantes usadas, para poder ubicarlas rápido si algún valor cambia.

## Diagramas

Fuente Mermaid en `diagrams/open/mermaid/`, renders PNG en `diagrams/open/png/` (generados con `task robot:diagrams`).

| Diagrama | Fuente `.mmd` | PNG |
|---|---|---|
| Máquina de estados de alto nivel | [`diagrams/open/mermaid/maquina-estados.mmd`](diagrams/open/mermaid/maquina-estados.mmd) | `diagrams/open/png/maquina-estados.png` |
| Flujo completo por tick de control | [`diagrams/open/mermaid/flujo-completo.mmd`](diagrams/open/mermaid/flujo-completo.mmd) | `diagrams/open/png/flujo-completo.png` |

### Detalle común (compartido con Obstacle Challenge)

| Diagrama | Fuente `.mmd` | PNG |
|---|---|---|
| Inferencia de dirección (horario vs. antihorario) | [`diagrams/common/mermaid/inferencia-direccion.mmd`](diagrams/common/mermaid/inferencia-direccion.mmd) | `diagrams/common/png/inferencia-direccion.png` |
| Vigilancia de colisión y atasco | [`diagrams/common/mermaid/escape-colision.mmd`](diagrams/common/mermaid/escape-colision.mmd) | `diagrams/common/png/escape-colision.png` |
| Esquiva genérica ante un obstáculo | [`diagrams/common/mermaid/esquiva-generica.mmd`](diagrams/common/mermaid/esquiva-generica.mmd) | `diagrams/common/png/esquiva-generica.png` |
| Conteo de vueltas | [`diagrams/common/mermaid/conteo-vueltas.mmd`](diagrams/common/mermaid/conteo-vueltas.mmd) | `diagrams/common/png/conteo-vueltas.png` |

## Notas

- El robot **siempre** empieza sin conocer la dirección de la pista: la Fase 1 es obligatoria en todo Open Challenge, no opcional. Avanza centrado entre paredes mientras acumula votos por asimetría izquierda/derecha hasta que el mismo lado gana 5 lecturas seguidas.
- El Open Challenge **nunca** construye un enrutador de señales ni un controlador de estacionamiento, por lo que la conducción normal simplemente omite esas ramas.
- Al completar las 3 vueltas, el robot no tiene ninguna maniobra de estacionamiento que ejecutar: se detiene por completo y de forma indefinida.
- La selección de sentido de giro es puramente reactiva (asimetría del espacio libre lateral) y no depende del tipo de desafío — por eso vive en `common/` en vez de duplicarse.
