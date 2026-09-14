# Historial de versiones de vTitan

Cada etiqueta marca un estado del robot que existió de verdad: o compitió, o se
desplegó en las placas y corrió rondas completas. Entre etiquetas el historial es
continuo, con mensajes de commit convencionales (`fix(nav):`, `feat(sim):`,
`perf(robot):`), así que cualquier cifra de esta documentación se puede rastrear
hasta el commit exacto que la produjo.

Las tres etiquetas `archive/*` no son versiones: son ramas de trabajo congeladas
para no perder su historia.

---

## v1.2 (13 de septiembre de 2026)

**216 commits desde v1.1.** La versión más grande de la temporada, y la que
cambió más el método, no solo el código.

**Enrutado por señales.** Se construyó el mapa de posiciones restringido por
reglas, primero apagado y después activado para su A/B de hardware. El
compromiso del enrutador con una señal ya no se suelta porque aparezca otro
pilar, y el planificador de carril de señales pasó a centrar el hueco cuando el
carril viene estrecho.

**Estacionamiento.** Pasó de ser código inalcanzable a ejecutable: el
aparcamiento se deriva ahora del arranque dentro de la bahía, que es lo que
permite que el controlador llegue siquiera a construirse.

**Mediciones que corrigieron supuestos.** El servomotor resultó ser **al menos el
doble de rápido de lo asumido, 2.4 rad/s medidos**, lo que resolvió la salida de
bahía. La cuota de detecciones verdes no se había desplomado: es una función del
**sentido de marcha**, y creerlo al revés había enviado a buscar un fallo que no
existía.

**Diagnósticos que recomendaron no construir.** Varios commits de esta versión
son análisis que terminaron en "no lo construyas": la maniobra de retroceso para
comprar pista se evaluó y se descartó con números. Un resultado negativo bien
medido ahorra más que uno positivo mal medido.

**45 refactorizaciones**, sobre todo consolidar constantes en una sola fuente y
migrar cargadores de configuración a estructuras generadas desde los esquemas.

## v1.1 (8 de septiembre de 2026)

**26 commits desde v1.0.** Versión estrecha y correctiva, posterior al evento
regional.

**Localización.** El robot ahora publica **si su estimación de posición explica
de verdad el barrido del LIDAR**, y tiene un camino de vuelta cuando deja de
explicarlo. Antes, una pose divergida era irrecuperable a mitad de ronda y
costaba la carrera entera.

**Geometría real del chasis.** Se le dio su radio de giro medido y un escape lo
bastante largo para poder usarlo. Hasta aquí el planificador pedía curvas que el
chasis no puede trazar.

**Fidelidad del simulador.** La cámara simulada pasó a quedarse ciega con la
distancia, como la real, y las cifras de calibración del modelo de alcance se
corrigieron midiéndolas en lazo cerrado.

**Fusión LIDAR y visión**, detrás de `SIGN_LIDAR_PROPOSE` y apagada por defecto:
el LIDAR propone **dónde** está una señal mientras la cámara sigue decidiendo
**qué** es.

## v1.0 (7 de septiembre de 2026)

**El estado con el que vTitan compitió en el evento regional de la WRO 2026.**
1311 commits desde el inicio del proyecto.

Contiene el robot completo tal y como se describe en el
[README](README.md): la pila ROS2 repartida entre Raspberry Pi 5 y Pi Zero 2 W,
el detector YOLO compilado para el AI HAT+, la navegación por pure pursuit con
control PI de velocidad, la máquina de estados de los dos desafíos, el
simulador con su corpus de escenarios de semilla fija, y el árbol de
configuración TOML que gobierna ambas implementaciones.

---

## Después de v1.2 (sin etiquetar)

70 commits. El trabajo en curso se ha concentrado en **cerrar la brecha entre el
simulador y el hardware**, porque varias conclusiones de temporada resultaron
ser artefactos del modelo y no del robot:

- El corpus ya no corre con **pose perfecta**: usa el presupuesto de error de
  sensor medido.
- La cámara emulada tiene la **tasa de detección y la latencia** del hardware.
- El chasis bloqueado **desliza a lo largo de la superficie** en vez de detenerse
  en seco, que era el mayor artefacto conocido del modelo de contacto.
- El servo simulado se mueve a la **velocidad del hardware**, no a la de la
  política de curva.
- Se modeló la **oclusión del propio chasis y la tasa real de pérdida** del
  LIDAR C1.

Estos cambios están marcados con `!` porque **invalidan comparaciones anteriores**:
un número medido antes de ellos no se resta con uno medido después.

---

## Convenciones

- **Etiquetas**: `vMAYOR.MENOR`, creadas cuando un estado se despliega y corre.
- **Mensajes de commit**: convencionales, en inglés. La documentación va en
  español.
- **`!` en el tipo** (`feat(sim)!:`) marca un cambio que rompe la comparabilidad
  de resultados anteriores, no solo la compatibilidad de una interfaz.
- **Firma**: los commits van firmados con GPG y aparecen como verificados.
