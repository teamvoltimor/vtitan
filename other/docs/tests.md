# Flujo de pruebas de vTitan

Este documento describe **cómo probamos**, no qué pruebas existen. La lista de
pruebas se lee en el código; lo que no se lee en el código es el criterio: qué
nivel usamos para cada pregunta, qué medimos, y qué trampas nos han costado
conclusiones falsas.

El principio que ordena todo lo demás: **medir antes de optimizar**. Los errores
más caros del proyecto no fueron de programación sino de suposiciones sin
verificar (ver [Hallazgos de ingeniería](README.md#hallazgos-de-ingeniería)), y
cada nivel de esta escalera existe para atrapar una clase distinta de esas
suposiciones.

## Los cuatro niveles

| Nivel | Qué atrapa | Coste por intento | Cuándo se usa |
|---|---|---|---|
| 1. Pruebas unitarias | Lógica rota, contratos de módulo, regresiones | segundos | En cada cambio |
| 2. Corpus de simulación | Comportamiento de navegación sobre cientos de pistas | minutos | Antes de aceptar cualquier ajuste |
| 3. Banco de hardware | Lo que el simulador no modela: servo, motor, LIDAR real | minutos, con el robot delante | Antes de llevar algo a pista |
| 4. Rondas en pista | La verdad | ~3 minutos por ronda, y no se repite a voluntad | Validación final |

La regla que los une: **una conclusión que solo existe en el nivel 2 no es una
conclusión**. El simulador sirve para descartar rápido y para comparar, no para
aprobar.

## Nivel 1: pruebas unitarias

95 archivos de prueba en Python y 162 en Go.

```bash
task test                      # Python + Go, todos los módulos
task robot:test                # solo el robot (entorno pixi)
task robot:test SCOPE=unit     # unit | fast | navigation | hardware | all
task go:test                   # módulo Go del robot (Go puro, sin la ruta CGo de cámara)
task sim:test                  # pruebas de la simulación
task lint                      # ruff, golangci-lint, ESLint, buf lint
```

**Configuración que importa** (`src/python/pytest.ini`):

- `-n 6 --dist loadscope`. Es el máximo confirmado que completa una corrida
  entera. Por encima, los trabajadores nativos de xdist se caen en Windows al
  destruir el contexto de rclpy. No es un número arbitrario y no conviene
  subirlo sin volver a medirlo.
- `-p no:launch_testing -p no:launch_ros`. Los paquetes ROS2 de RoboStack traen
  plugins de pytest cuyos *hookspecs* son incompatibles con pytest >= 8 y
  abortan la recolección. Este proyecto usa pytest plano con simulacros, así que
  se desactivan.
- Marcadores: `hardware` (exige una Raspberry con periféricos) y `slow`
  (simulación de lazo cerrado, varios segundos por caso).

**Trampa conocida:** el conjunto necesita `VTITAN_HARDWARE_PROFILE` **sin
definir**, mientras que los diagnósticos la necesitan **definida**. Una variable
de entorno heredada de una sesión de diagnóstico hace fallar pruebas que no
tienen nada roto.

## Nivel 2: el corpus de simulación

Probar solo en la pista física tiene un límite duro: cada intento cuesta
minutos, cada montaje es distinto, y una situación difícil no se repite a
voluntad. Por eso generamos un corpus con semilla fija.

```bash
task gen:corpus:all            # 640 escenarios Open + 256 Obstacles, semilla 2026
task sim:navigate              # lazo cerrado con el navegador real
task sim:navigate:visualize:all -- --challenge open --interactive   # en vivo, con RViz
```

Semilla fija significa **resultados repetibles**: la misma revisión sobre el
mismo corpus da el mismo número, así que una diferencia entre dos corridas es
una diferencia de código y no de suerte.

### Métricas que decidimos

No usamos "se ve mejor". Cada comparación se resuelve con las mismas columnas:

- **vueltas >= 3**: escenarios donde el robot completó las tres vueltas.
- **en tiempo**: de los anteriores, cuántos dentro del límite de 180 s.
- **tiempos agotados**: escenarios que agotaron el reloj.
- **colisiones**: contactos con pared o con señal.
- **paso por el lado incorrecto** (solo Obstacles): se puntúa contra verdad de
  campo, no contra lo que el robot creía. Un paso por el lado incorrecto
  **termina la ronda**, así que esta columna no se promedia con las demás.

### Protocolo de comparación A/B, y sus cinco trampas

Este protocolo nació de conclusiones que resultaron falsas. Cada punto es una
que nos costó tiempo:

1. **Criba con 128, decide con 640.** 128 escenarios sirven para descartar; no
   sirven para aceptar. Varias recomendaciones sobrevivieron a 128 y murieron al
   volver a medirlas sobre el corpus grande.
2. **La referencia es el commit padre**, no una medición anterior. Medir contra
   un número apuntado la semana pasada compara dos cosas que ya no comparten
   código.
3. **Los resultados no son comparables en el tiempo.** Si el corpus, el modelo
   del simulador o cualquier constante cambiaron entre dos corridas, los dos
   números no se restan.
4. **Las dos ramas deben correr módulos de prueba idénticos.** Un conjunto
   distinto convierte la comparación en ruido.
5. **Diferenciar las dos ramas antes de creer el resultado.** Un eje de barrido
   cableado a la nada produce dos brazos byte a byte idénticos y un veredicto
   perfectamente convincente.

Regla derivada: `--tuning` **reemplaza** el árbol de configuración por el
archivo indicado, no lo superpone. Un A/B que pasa solo la clave que cambia deja
todas las demás en su valor por defecto, y entonces no está midiendo lo que cree.

## Nivel 3: banco de hardware

El simulador no modela todo, y lo que no modela es justamente lo que rompe en
pista. Dos ejemplos medidos: el modelo de contacto del simulador **nunca desliza
a lo largo de una pared**, y el LIDAR simulado es mucho más limpio que el C1
real. Por eso hay pruebas que solo existen contra el robot.

```bash
task go:hw:build                # compila cruzado las pruebas //go:build hw para linux/arm64
task go:hw:build:interactive    # las que mueven el robot: giro de motor, barrido de servo
task go:hw:run                 # compila, envía a la Pi y ejecuta un paquete completo
task robot:test-motors         # prueba de humo del nodo de motores, se corre EN la Pi 5
task robot:calibrate-encoder   # calibración de counts_per_rev contra distancia medida con cinta
```

Las interactivas están **cerradas por operador**: no arrancan solas, porque
mueven el robot. Antes de correrlas hay que parar los servicios de producción
con `task go:hw:stop`, o dos procesos se pelean por el mismo puerto serie.

`robot:calibrate-encoder` merece mención aparte: es la prueba que descubrió que
el encoder daba **60 pulsos por vuelta y no 86**, un error que falseaba toda
medición de distancia y velocidad del proyecto.

## Nivel 4: rondas en pista, y la grabación

Cada ronda queda grabada entera en un bag MCAP: pose, LIDAR, detecciones,
comandos y el estado interno del navegador, tick a tick. Una ronda dura 180 s y
no se repite, así que lo que no quedó grabado se perdió.

```bash
task robot:deploy              # código + detector HEF a la Pi, reconstruye y reinicia servicios
task robot:pull-runs           # baja los bags MCAP
task robot:watch-vision        # detecciones en vivo, una línea por fotograma
```

El análisis posterior vive en los diagnósticos `diag_bag_*.py`. El flujo normal
es empezar por el inventario (una fila por bag) y solo entonces abrir el bag
concreto que interesa.

**Trampas de análisis, todas medidas:**

- **Un diagnóstico que se cae devuelve código de salida 0.** Hay que leer la
  salida, nunca el código. Y para afirmar que algo *no aparece*, hace falta un
  control con algo que sí esté presente; si no, no se distingue un hecho de una
  ruta mal escrita.
- `travelled_m` es **con signo** y se cancela solo cuando el robot oscila
  adelante y atrás. Un robot atascado en un vaivén reporta cero recorrido.
- `/motor/drive_speed` está en **grados por segundo**, no en metros por segundo.
- La longitud del camino calculada desde la pose **sobreestima ~10%**.
- La Pi poda los bags a 20 corridas o 4 GB. El archivo histórico vive en el
  computador de desarrollo, y no debe copiarse de vuelta a la placa.

## Configuración: pruebas que no son de código

El robot está gobernado por un árbol de TOML, y esa configuración se valida como
si fuera código:

```bash
task config:validate    # el árbol src/config/ contra su estructura
task config:lint        # los TOML contra su esquema JSON, con Taplo
task config:check       # cada clave descrita, y cada referencia x-journal resuelve
```

`config:check` es el que cierra el círculo documental: **falla si un valor del
robot cita un ADR que no existe**. Una constante sin justificación rastreable no
pasa la verificación del repositorio.

## Integración continua

`.github/workflows/ci.yml` corre diez trabajos en cada push y en cada pull
request: lint más pruebas del robot, backend Go, la API y el servicio de
aprendizaje del auto-anotador, Hailo, el frontend, y construcción de imágenes
Docker con escaneo de vulnerabilidades.

**Estado real, declarado:** CI está en rojo, y por dos causas concretas que
conviene no confundir con pruebas que fallan:

1. Cuatro trabajos de Docker no arrancan porque la acción
   `aquasecurity/trivy-action` está fijada a una versión `0.28.0` que ya no
   existe. Es un problema de fijado de versión, no del proyecto.
2. Tres trabajos mueren en el paso de Ruff (40, 14 y 2 errores de estilo) **antes
   de llegar a ejecutar una sola prueba**.

Es decir: hoy CI no nos dice nada sobre si las pruebas pasan, porque en la mitad
de los trabajos nunca llegan a correr. Lo anotamos aquí en vez de esconderlo,
porque un CI rojo que nadie mira es peor que no tenerlo: da una sensación de
cobertura que no existe. La verificación real de esta temporada se ha hecho en
local con los niveles 1 a 4 descritos arriba.

## Cómo usamos todo esto en un cambio típico

1. Escribir la prueba unitaria que falla (nivel 1).
2. Implementar hasta que pase, y `task lint`.
3. Correr el corpus de criba, 128 escenarios, contra el **commit padre**.
4. Si sobrevive, repetir con el corpus completo de 640.
5. Si toca hardware, banco (nivel 3) antes de pista.
6. Pista, y el bag se guarda.
7. Si el valor cambiado es una constante de configuración, escribir su ADR en
   `other/docs/adr/` y enlazarlo desde el esquema con `x-journal`, o
   `config:check` lo rechaza.
