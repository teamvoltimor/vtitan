# Diagrama de lógica - Open Challenge

Diagramas Mermaid que documentan el flujo de control del **Open Challenge**, en lenguaje natural. El robot corre siempre en **modo ciego**: al iniciar no conoce la dirección de la pista (horario/antihorario) ni tiene mapa, así que la primera fase obligatoria es un avance reactivo que infiere la dirección observando la asimetría del pasillo con el LIDAR. No hay señales de tránsito ni maniobra de estacionamiento - eso corresponde al Obstacle Challenge, documentado en [`obstacle-challenge-diagrama.md`](obstacle-challenge-diagrama.md).

La lógica compartida entre ambos desafíos (inferencia de dirección, vigilancia de colisión/atasco, conteo de vueltas) vive una sola vez en [`schemes/flowcharts/common/`](../../../schemes/flowcharts/common/mermaid/) y se referencia desde aquí, en vez de redibujarse.

Cada archivo `.mmd` trae al inicio un bloque de comentarios `%%` (no se renderiza en la imagen) con las referencias exactas al código y a las constantes usadas, para poder ubicarlas rápido si algún valor cambia.

## Diagramas

Fuente Mermaid en `schemes/flowcharts/open/mermaid/`, renders WebP en `schemes/flowcharts/open/webp/` (generados con `task docs:diagrams`).

| Diagrama | Fuente `.mmd` | WebP |
|---|---|---|
| Máquina de estados de alto nivel | [`schemes/flowcharts/open/mermaid/maquina-estados.mmd`](../../../schemes/flowcharts/open/mermaid/maquina-estados.mmd) | `schemes/flowcharts/open/webp/maquina-estados.webp` |
| Flujo completo por tick de control | [`schemes/flowcharts/open/mermaid/flujo-completo.mmd`](../../../schemes/flowcharts/open/mermaid/flujo-completo.mmd) | `schemes/flowcharts/open/webp/flujo-completo.webp` |

### Detalle común (compartido con Obstacle Challenge)

| Diagrama | Fuente `.mmd` | WebP |
|---|---|---|
| Inferencia de dirección (horario vs. antihorario) | [`schemes/flowcharts/common/mermaid/inferencia-direccion.mmd`](../../../schemes/flowcharts/common/mermaid/inferencia-direccion.mmd) | `schemes/flowcharts/common/webp/inferencia-direccion.webp` |
| Vigilancia de colisión y atasco | [`schemes/flowcharts/common/mermaid/escape-colision.mmd`](../../../schemes/flowcharts/common/mermaid/escape-colision.mmd) | `schemes/flowcharts/common/webp/escape-colision.webp` |
| Esquiva genérica ante un obstáculo | [`schemes/flowcharts/common/mermaid/esquiva-generica.mmd`](../../../schemes/flowcharts/common/mermaid/esquiva-generica.mmd) | `schemes/flowcharts/common/webp/esquiva-generica.webp` |
| Conteo de vueltas | [`schemes/flowcharts/common/mermaid/conteo-vueltas.mmd`](../../../schemes/flowcharts/common/mermaid/conteo-vueltas.mmd) | `schemes/flowcharts/common/webp/conteo-vueltas.webp` |

## Notas

- El robot **siempre** empieza sin conocer la dirección de la pista: la Fase 1 es obligatoria en todo Open Challenge, no opcional. Avanza centrado entre paredes mientras acumula votos por asimetría izquierda/derecha hasta que el mismo lado gana 5 lecturas seguidas.
- El Open Challenge **nunca** construye un enrutador de señales ni un controlador de estacionamiento, por lo que la conducción normal simplemente omite esas ramas.
- Al completar las 3 vueltas, el robot no tiene ninguna maniobra de estacionamiento que ejecutar: se detiene por completo y de forma indefinida.
- La selección de sentido de giro es puramente reactiva (asimetría del espacio libre lateral) y no depende del tipo de desafío - por eso vive en `common/` en vez de duplicarse.
