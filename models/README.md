# Modelos 3D

Piezas de vTitan y de los prototipos anteriores. **El razonamiento de diseño no
vive aquí**: está en el [README principal](../README.md), en
[Movilidad y Diseño Mecánico](../README.md#movilidad-y-diseño-mecánico). Este
documento es el manifiesto: qué hay, en qué formato y para qué sirve cada uno.

```text
models/
├── current-models/     vTitan (WRO 2026)
│   ├── blueprints/     planos acotados (PNG y WebP)
│   ├── step-files/     29 archivos .step  -> para fabricar y editar
│   └── stl-files/      48 archivos .stl   -> para imprimir y para ver en GitHub
└── old-models/         Klevor (WRO 2025)
    ├── blueprints/     27 planos
    └── stl-files/      43 archivos .stl
```

## Por qué dos formatos

| Formato | Para qué | Nota |
|---|---|---|
| **`.step`** | Fabricar y **editar**. Conserva la geometría exacta (B-rep), así que se puede reabrir y modificar en cualquier CAD | GitHub **no** lo previsualiza |
| **`.stl`** | **Imprimir** y **mirar**. Es una malla triangular, no se edita bien, pero es lo que come una impresora 3D | GitHub lo renderiza en un **visor 3D interactivo**: pincha cualquier `.stl` y podrás girarlo, desplazarlo y acercarlo en el navegador |

Por eso publicamos ambos. Si solo quieres ver una pieza, abre el `.stl` desde
GitHub. Si quieres rehacerla, usa el `.step`.

De las 29 piezas en `.step`, **17 tienen su `.stl` con el mismo nombre**. Las 12
restantes son componentes comerciales que modelamos solo para el ensamblaje
virtual y nunca se imprimen: Raspberry Pi 5, cámara, RPLiDAR C1, batería
Ovonic, rodamientos y rolineras.

## Convención de nombres

Todo en kebab-case ASCII y extensión en minúscula: `pinon-40-dientes-servo-cajera.stl`.
Los nombres originales de SolidWorks (mayúsculas, espacios, `ñ`) se normalizaron
porque rompen URLs, globs de shell y clones en sistemas de archivos que no son
UTF-8. Una pieza que existe en los dos formatos **usa el mismo nombre** en
ambos, que es lo que permite emparejarlas de un vistazo.

## Piezas de vTitan, por subsistema

**Transmisión**

`engranaje-unificado-motor-rev-36-dientes`, `engranaje-unificado-motor-rev-40-dientes`,
`pinon-15-dientes-correa-dentada`, `pinon-20-dientes-correa-dentada`,
`pinon-22-dientes-correa-dentada`, `pinon-33-dientes-correa-dentada`,
`pinon-33-dientes-correa-dentada-eje-motor-pequeno`,
`pinon-n-dientes-correa-dentada-eje-motor-pequeno`, `pinon-conico-15x8-dientes`,
`pinon-conico-20-dientes`, `pinon-90-cruceta-10-dientes`,
`pinon-90-cruceta-10-dientes-con-eje`, `rueda-dentada`, `rueda-vtitan`,
`ring-mv-max`, `ring-mv-ultimate`, `ring-mv-pinon-grande`, `cubierta-de-ring`,
`buje-guia-de-cruceta`, `aro-de-fijacion-axial-de-caucho`

**Dirección**

`brazo-de-direccion-max`, `pinon-8-dientes-direccion`,
`pinon-20-dientes-direccion`, `pinon-20-dientes-direccion-editado`,
`pinon-20-dientes-rueda-dentada`, `pinon-20-dientes-rueda-dentada-nuevo`,
`pinon-40-dientes-direccion`, `pinon-40-dientes-direccion-arrastre`,
`pinon-40-dientes-servo`, `pinon-40-dientes-servo-cajera`

**Tensor de correa**

`brazo-de-tensor-1`, `brazo-de-tensor-2`, `brazo-de-tensor-2-largo`,
`brazo-de-tensor-3`, `rodillo-tensor`, `rodillo-tensor-nuevo`

**Chasis y bancadas**

`monochasis`, `monochasis-max`, `monochasis-max-ligero`, `monochasis-ultimate`,
`base-de-sistema-de-transmision-corto`, `soporte-superior`,
`tapa-de-bancada-motor-pequeno`, `tapa-de-bancada-motor-rev`,
`suplemento-de-bancada-motor-pequeno`

**Soportes de cámara**

`soporte-camara-inferior`, `soporte-camara-brazo-intermedio`,
`soporte-camara-superior`

**Componentes comerciales** (modelados solo para el ensamblaje, no se imprimen)

`raspberry-pi-5`, `camera-module-3-v8`, `rplidar-c1`, `ovonic-air-lipo-battery`,
`rodamiento-14-20-12-hk1412`, `rolinera-3-7-2`, `rolinera-6.35-9.525-3.175`

**No son del robot**

`pista-bloque-senales` y `pista-pared-magenta` son piezas de **pista**, no de
vTitan: las señales de color y un tramo de pared que imprimimos para practicar.
`prueba-rodamiento` es una impresión de prueba de ajuste.

## Trazabilidad

Varias piezas llevan sufijos de versión del diseñador (`-max`, `-ultimate`,
`-nuevo`, `-editado`, `-largo`) porque son iteraciones reales que convivieron.
Se conservan a propósito: son la evidencia de cómo evolucionó el diseño, y esa
historia está contada en
[Evolución y justificación del diseño](../README.md#evolución-y-justificación-del-diseño).

> [!NOTE]
> **Pendiente declarado.** Falta el manifiesto de impresión: material, altura de
> capa, relleno, soportes y orientación de cama por pieza. Sin esos parámetros,
> otra persona puede abrir los `.stl` pero no reproducir exactamente nuestras
> piezas. También falta `pinon-n-dientes-correa-dentada-eje-motor-pequeno`, cuyo
> número de dientes nunca quedó en el nombre.
