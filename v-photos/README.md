# Fotos y renders de vTitan

Carpeta obligatoria de la categoría Futuros Ingenieros. **La descripción del
robot está en el [README principal](../README.md)**; aquí solo está el índice.

```text
v-photos/
├── vtitan/                  vTitan (WRO 2026)
└── klevor/                  Klevor (WRO 2025), cuatro iteraciones
```

## vTitan

| Archivo | Qué es |
|---|---|
| `vtitan-front-view.webp`, `vtitan-rear-view.webp`, `vtitan-left-view.webp`, `vtitan-right-view.webp`, `vtitan-top-view.webp`, `vtitan-bottom-view.webp` | Las seis vistas que pide la categoría |
| `vtitan-full.webp` | Render del conjunto ensamblado desde el CAD |
| `vtitan-breakdown.webp` | Vista despiezada, usada en [Montaje](../README.md#montaje) |
| `vtitan-breakdown-enumerated.webp` | La misma vista despiezada, con cada pieza numerada |

Los renders del conjunto y las dos vistas despiezadas salen del mismo modelo de
SolidWorks del que se exportan [`models/vtitan/`](../models/README.md), así que lo
que se ve en el render es exactamente lo que hay en los `.step` y los `.stl`.

## Prototipos anteriores

`klevor/` guarda las cuatro iteraciones de Klevor, el robot de la
temporada 2025, con sus vistas y (en `klevor-v0.1/`) las tres capas por
separado. Cada iteración tiene su documento con qué cambió y por qué:

- [Klevor v0.1](../other/docs/development/previous-prototypes/klevor-v0.1.md)
- [Klevor v0.1.1](../other/docs/development/previous-prototypes/klevor-v0.1.1.md)
- [Klevor v0.2](../other/docs/development/previous-prototypes/klevor-v0.2.md)
- [Klevor v1.0](../other/docs/development/previous-prototypes/klevor-v1.0.md)

> [!WARNING]
> **Pendiente declarado.** De vTitan solo hay fotos del estado final. La
> historia de iteración documentada con imágenes es la de Klevor, no la de
> vTitan; las fases de diseño de esta temporada están contadas en texto en
> [Evolución y justificación del diseño](../README.md#evolución-y-justificación-del-diseño).

Todas las imágenes están en WebP. `vtitan-full.webp` es la única sin pérdida,
porque es un render del CAD con zonas planas que comprimen bien así; las 45
fotografías usan WebP con pérdida a calidad alta, que en fotos pesa una fracción
de lo que costaría conservarlas sin pérdida.
