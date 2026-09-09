# Guía para construir la Documentación de Ingeniería (WRO Future Engineers 2026)

Este documento es una **guía de trabajo interna**, no la documentación final para los jueces.
Su objetivo es servir de mapa para ir construyendo el **Engineering Journal** y el
**repositorio de GitHub** a medida que el proyecto avanza, de forma que al llegar la fecha
límite ya exista material suficiente y bien organizado en vez de tener que reconstruir la
historia del proyecto de memoria en las últimas semanas.

Está basada en el **Apéndice C** del reglamento WRO Future Engineers 2026 ("Engineering
Journal and Documentation Requirements"). Cuando el reglamento y esta guía difieran, prevalece
el reglamento; en tal caso, esta guía debe actualizarse.

---

## 1. Qué se evalúa y por qué esto nos importa desde ya

Los jueces **no** califican por belleza visual ni por longitud. Califican:

1. El **proceso** de ingeniería, no solo el robot terminado.
2. La **calidad de las decisiones** de diseño y el pensamiento sistémico.
3. Que **otro equipo pudiera reproducir** el robot con la documentación.
4. Que sirva de **diferenciador** cuando el desempeño en pista es similar entre equipos.

Esto significa que la documentación **no se escribe al final**: se construye en paralelo al
desarrollo, capturando decisiones, pruebas y fallos *en el momento en que ocurren*. Un commit
con buen mensaje, un experimento fallido registrado y un `docs/internal/algorithms/*.md`
actualizado valen más para el puntaje que una redacción pulida escrita de memoria tres
semanas después.

**Regla práctica:** cada vez que cerremos una decisión de diseño, un ajuste de calibración
importante, o detectemos (y resolvamos) un fallo de fondo, esa nota va a una de dos partes:
el Engineering Journal (narrativa) o el repositorio (README, documentación técnica, CAD,
cableado). Si no queda registrado en algún lugar, a efectos de evaluación **no existió**.

---

## 2. Los cinco criterios (30 puntos, escala 0/2/4/6)

| # | Criterio | Qué mide en una frase |
|---|----------|------------------------|
| 1 | Mobility & Mechanical Design | Chasis, dirección, tracción, torque/velocidad, iteración mecánica |
| 2 | Power & Sensor Architecture | Presupuesto de potencia, selección y ubicación de sensores, calibración, cableado |
| 3 | Software Architecture & Obstacle Strategy | Arquitectura de software, máquina de estados, algoritmos, gestión de obstáculos, ajuste de parámetros |
| 4 | Systems Thinking & Engineering Decisions | Restricciones, trade-offs, iteración, riesgos y mitigación, "elegimos X en vez de Y porque…" |
| 5 | Reproducibility & GitHub Quality | Estructura del repositorio, commits, README, CAD, código y cableado reproducibles |

Escala por criterio:

- **0 - Sin evidencia:** falta, es irrelevante, o no se puede evaluar.
- **2 - Evidencia limitada:** hay información, pero incompleta o sin justificar.
- **4 - Ingeniería competente:** claro, estructurado, reproducible.
- **6 - Ingeniería avanzada:** decisiones, pruebas, trade-offs y pensamiento sistémico
  completamente justificados.

Máximo total: **30 puntos**. Aplica igual a todas las edades (14–22); el reconocimiento por
edad es aparte y no cambia el puntaje.

---

## 3. Qué necesita cada criterio (versión operativa)

### 3.1 Mobility and Mechanical Design

Para llegar a nivel 6 necesitamos, además de la descripción del chasis/dirección/tracción:

- Razonamiento de **torque y velocidad** (por qué este motor, esta relación de reducción,
  este diámetro de rueda).
- **Trade-offs** explícitos de diseño (ej. velocidad vs. precisión de frenado).
- Evidencia de **pruebas/iteraciones** que cambiaron el diseño mecánico y mejoraron el
  desempeño (versión 1 → versión 2, con datos de por qué).
- **Diagramas** (aunque sean dibujos simples o el CAD exportado) que permitan a otro equipo
  reproducir el diseño.

Fuentes internas a explotar: mediciones de chasis/ruedas en memoria
(`robot_wheel_lidar_camera_measurements`), `robot.toml` como fuente de verdad, y cualquier
sesión de banco de motor/servo (`motor_servo_bench_session`).

### 3.2 Power and Sensor Architecture

Para nivel 6:

- **Presupuesto de potencia**: consumo estimado/medido por subsistema (motores, Pi, LIDAR,
  cámara, IMU) y por qué el regulador y la batería elegidos resultan suficientes.
- **Trade-offs de sensores** y **justificación de ubicación** en función de la geometría del
  campo (ej. altura y ángulo de la cámara para evitar deslumbramiento, zona ciega del LIDAR).
- **Método de calibración** de cada sensor.
- Consideración de **puntos de falla** (interferencia, ruido, sombras, vibración) y cómo se
  mitigan.
- Al menos un **diagrama de cableado**.

Fuentes internas: perfiles de hardware, `lidar_offset_clearance_recalibration`,
`encoder_counts_per_rev`, `drive_feedforward_is_affine`, cualquier nota de calibración de
IMU/cámara/LIDAR.

### 3.3 Software Architecture and Obstacle Strategy

Para nivel 6:

- **Máquina de estados** con justificación de cada transición.
- Algoritmos **justificados** (por qué pure pursuit y no otro controlador, por qué ese
  método de visión, cómo se fusiona IMU).
- **Casos límite** manejados explícitamente (señal ambigua, oclusión, pérdida de carril).
- Proceso de **pruebas y de ajuste descrito**, con las **métricas** empleadas (tasa de
  colisión, error de trayectoria o *cross-track*, vueltas completadas, etc.), no solo
  "funciona".

Este es el criterio donde más material ya existe en el proyecto (barridos, comparaciones A/B,
métricas de `SimResult`, scripts de diagnóstico). El trabajo aquí es más de **curaduría y
narrativa** que de generación: convertir hallazgos técnicos (memoria, commits,
`docs/internal/algorithms/`) en una explicación legible para un jurado que no conoce el
código.

Para la parte de ajuste de parámetros, la referencia versionada es
[`configuracion_toml_navegacion.md`](configuracion_toml_navegacion.md): documenta qué variable
gobierna cada comportamiento en pista, en qué fichero reside y con qué orden de precedencia se
resuelve. Es material directamente citable en este criterio y en el 4.

### 3.4 Systems Thinking and Engineering Decisions

Para nivel 6:

- **Restricciones explícitas** identificadas (peso, potencia, procesamiento, tiempo,
  presupuesto).
- **Trade-offs e iteraciones** descritos con datos.
- **Riesgos y modos de falla** discutidos, con mitigación.
- Frases del tipo *"elegimos X en vez de Y porque…"* respaldadas por pruebas o datos, no
  solo intuición.

Este criterio conecta los otros cuatro: requiere mostrar cómo mecánica, potencia, sensores y
software interactúan como sistema, no como partes aisladas.

### 3.5 Reproducibility and GitHub Quality

Para nivel 6:

- Robot **totalmente reproducible** desde la documentación.
- Estructura de repositorio **clara**, mensajes de commit **significativos**.
- Flujo de **pruebas documentado**.
- **Versionado o notas de versión** (ej. v1.0 = regional, v2.0 = internacional).

Mínimos de nivel 4 (no negociables, son criterios cuantitativos):

- README con **al menos 5000 caracteres**.
- Al menos **tres commits significativos** con mensajes claros.
- CAD, código y cableado **incluidos** en el repositorio, no solo mencionados.

---

## 4. Flujo de trabajo sugerido (construir sobre la evolución del proyecto)

En vez de escribir la documentación al final, usamos el propio historial del proyecto como
insumo continuo:

1. **Cada decisión de diseño o hallazgo relevante** (mecánico, eléctrico, de software o de
   pruebas) se registra apenas se cierra, en el lugar que corresponda:
   - Decisión de arquitectura/algoritmo → `docs/internal/algorithms/` (ya existe como fuente
     canónica interna) o el journal.
   - Cambio mecánico/eléctrico con antes/después → journal, con foto o diagrama.
   - Resultado de un experimento (A/B, barrido, calibración) → journal, citando el commit o
     la rama donde reside el código y, si procede, el archivo de resultados.
2. **Los commits sirven de bitácora cruda.** Un buen mensaje de commit (ver
   `feedback_commit_style`: sin `Co-Authored-By:`, logging estructurado, nunca `print()`) es
   la primera fuente para reconstruir *cuándo* y *por qué* cambió algo. Revisar
   `git log --oneline` periódicamente para no perder decisiones que solo quedaron en el
   código.
3. **Cada 2–3 semanas, sesión de "traducción"**: tomar las notas técnicas acumuladas
   (memoria de sesiones de desarrollo, `docs/internal/`, commits) y convertir las que tengan
   peso de **decisión de ingeniería** (no fallos internos triviales) en párrafos del journal,
   usando el lenguaje de "constraint → trade-off → decisión → resultado medido".
4. **Antes de la fecha límite**, aplicar la lista de verificación de la sección 6 y realizar
   una auditoría cruzada: ¿cada criterio cuenta con al menos un ejemplo de nivel 4, con
   evidencia concreta (dato, diagrama, gráfico) y no con una mera afirmación?

Este enfoque evita el problema típico: llegar al final con un robot funcional pero sin
memoria de *por qué* se tomó cada decisión, que es justo lo que más puntúa (criterios 1, 2,
4).

---

## 5. Plantilla de entrada de journal (una por decisión/iteración)

Usar esta plantilla mínima cada vez que se registre un hallazgo o decisión, para que el
journal tenga el formato que buscan los jueces sin esfuerzo extra al final:

```markdown
### [Fecha] Título corto de la decisión o iteración

**Contexto / restricción:** ¿qué límite o problema disparó esto? (peso, potencia, tiempo,
un fallo observado en pista o en simulación...)

**Opciones consideradas:** A vs B (aunque sea breve).

**Qué hicimos:** la decisión tomada.

**Por qué:** el razonamiento, apoyado en datos si existen (medición, barrido, prueba A/B).

**Resultado medido:** número, gráfico o comparación antes/después.

**Referencia:** commit(s), rama, archivo de resultados o memoria interna relacionada.
```

Ejemplo real ya disponible en el proyecto (criterio 3, nivel 6):

> Cambiamos el modelo de contacto de "ratchet" a "slide" en la salida de la bahía porque el
> primero producía una tasa de éxito perfecta (254/256) pero dependía de un artefacto de
> físicas de simulación no representativo del chasis real; verificamos con el modelo
> `slide` (`ae15ee3e`) que también resuelve la salida y es más conservador respecto al
> comportamiento físico esperado.

Este tipo de entrada ya casi escribe sola el párrafo de journal - solo falta trasladarla del
formato técnico interno al lenguaje narrativo para jueces.

---

## 6. Lista de verificación final antes de la entrega

### General

- [ ] El Engineering Journal cuenta la **historia del proceso**, no solo pasos de ensamblaje.
- [ ] El repositorio de GitHub tiene **estructura clara** y todos los archivos importantes.
- [ ] La documentación explica **por qué**, no solo **qué**.

### Mobility and Mechanical Design

- [ ] ¿Explicamos por qué elegimos este chasis y este sistema de dirección/tracción?
- [ ] ¿Incluimos diagramas de la disposición mecánica?
- [ ] ¿Describimos pruebas o cambios que mejoraron el diseño?

### Power and Sensor Architecture

- [ ] ¿Mostramos cómo se distribuye y regula la potencia?
- [ ] ¿Justificamos elección y ubicación de sensores?
- [ ] ¿Hay al menos un diagrama de cableado y descripción de calibración?

### Software Architecture and Obstacle Strategy

- [ ] ¿Mostramos un diagrama de flujo o una máquina de estados del software?
- [ ] ¿Explicamos cómo seguimos el carril y evitamos los obstáculos?
- [ ] ¿Incluimos descripciones de las pruebas y del ajuste de parámetros?

### Systems Thinking and Engineering Decisions

- [ ] ¿Identificamos restricciones (potencia, peso, tiempo, procesamiento)?
- [ ] ¿Mostramos al menos un trade-off explícito y por qué lo resolvimos así?
- [ ] ¿Mostramos cómo evolucionó el diseño (v1, v2, v3)?

### Reproducibility and GitHub Quality

- [ ] ¿Otro equipo podría reconstruir el robot con nuestra documentación?
- [ ] ¿El README explica cómo funciona el sistema y cómo construirlo? (mínimo 5000
      caracteres)
- [ ] ¿Tenemos al menos tres commits significativos con mensajes claros?
- [ ] ¿CAD, cableado y código están todos dentro del repositorio?

---

## 7. Glosario rápido (para nivelar el equipo)

- **Constraint (restricción):** límite dentro del cual hay que trabajar - peso máximo,
  batería, presupuesto, tiempo.
- **Trade-off:** elegir entre dos cosas donde mejorar una empeora la otra (ej. más velocidad,
  menos precisión de frenado).
- **Torque:** fuerza de giro de un motor; más torque ayuda a mover cargas o subir pendientes.
- **Power budget (presupuesto de potencia):** estimación de cuánta corriente/potencia usa
  cada parte y si la batería/reguladores lo soportan.
- **State machine (máquina de estados):** describir el comportamiento del robot como
  "estados" (buscar, seguir carril, evitar obstáculo) con reglas de transición.
- **Calibration (calibración):** ajustar lecturas de sensores o parámetros de control para
  que el robot mida y se comporte correctamente.
- **Noise (ruido):** variación no deseada en lecturas o señales que puede causar
  comportamiento inestable.
- **Iteration (iteración):** repetir "planear, construir, probar, mejorar". v1, v2, v3 son
  iteraciones.
- **Failure mode (modo de falla):** forma en que el robot puede fallar (ruedas patinan,
  sensor encandilado por luz).
- **Reproducibility (reproducibilidad):** que alguien más pueda seguir la documentación y
  construir un robot con desempeño similar.

---

## 8. Cómo evalúan los jueces (para escribir pensando en ellos)

Flujo típico del jurado (15–20 min por equipo):

1. Abren el repositorio de GitHub y localizan el README y las carpetas principales.
2. Revisan el Engineering Journal buscando las secciones que correspondan a los cinco
   criterios.
3. Por cada criterio, buscan evidencia de nivel 0/2/4/6.
4. Asignan un puntaje por criterio basado **solo en evidencia**, no en impresión general.

Consecuencia práctica: cada sección del journal y del README debería dejar **explícito a qué
criterio corresponde** (aunque sea con un encabezado tipo "Mecánica y chasis" / "Potencia y
sensores" / "Software y estrategia de obstáculos" / "Decisiones de sistema"), para que el
jurado no tenga que inferirlo. La calidad del idioma no afecta el puntaje salvo que impida
entender el razonamiento - pero como el journal será en español y el jurado puede no serlo,
conviene evaluar si conviene una versión o resumen en inglés para el journal final (fuera del
alcance de esta guía interna).

---

## 9. Notas de mantenimiento de esta guía

- Este archivo vive en `docs/documentacion_ingenieria_wro.md` y es una guía de proceso, no
  la documentación final de competencia.
- Si el reglamento 2026 cambia (nueva versión del Apéndice C), actualizar este archivo
  primero antes de seguir usándolo como referencia.
- La fecha límite de documentación y el enlace exacto del repositorio que se entregará deben quedar
  registrados aparte (ej. en el journal o en `docs/internal/backlog.md`), no en esta guía.
