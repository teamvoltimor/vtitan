# `src/` - Código de control de V-Titan

El código que corre en competencia **no vive en esta carpeta**: V-Titan es un monorepo y el
software está organizado por plataforma dentro de `platform/`. Esta carpeta existe para
señalar dónde encontrarlo.

## El código que corre en carrera

| Qué | Dónde |
|-----|-------|
| Pila ROS2 completa del robot (Python) | [`platform/robot/`](../platform/robot/) |
| Paquete de navegación: seguimiento de pasillo, planificación, escapes | [`platform/robot/ros2_ws/src/vtitan_navigation/`](../platform/robot/ros2_ws/src/vtitan_navigation/) |
| Paquete de visión: cámara e inferencia en el AI HAT+ | [`platform/robot/ros2_ws/src/vtitan_vision/`](../platform/robot/ros2_ws/src/vtitan_vision/) |
| Máquina de estados de carrera y grabación de bags | [`platform/robot/ros2_ws/src/vtitan_state_machine/`](../platform/robot/ros2_ws/src/vtitan_state_machine/) |
| Controladores de hardware (IMU, I2C, UART) | [`platform/robot/ros2_ws/src/vtitan_drivers/`](../platform/robot/ros2_ws/src/vtitan_drivers/) |
| Lanzamiento y composición de nodos | [`platform/robot/ros2_ws/src/vtitan_bringup/`](../platform/robot/ros2_ws/src/vtitan_bringup/) |
| Configuración que gobierna el comportamiento (203 constantes en TOML) | [`platform/shared/config/`](../platform/shared/config/) |

## Código que **no** corre en carrera

| Qué | Dónde |
|-----|-------|
| Segunda implementación de la pila en Go (en migración, no compite) | [`platform/robot-go/`](../platform/robot-go/) |
| Backend de telemetría en Go (sí corre en producción, fuera del lazo de carrera) | [`platform/backend/`](../platform/backend/) |
| Panel de telemetría (dashboard) | [`platform/frontend/`](../platform/frontend/) |
| Simulador y generador del corpus de escenarios | [`platform/gazebo/`](../platform/gazebo/), [`platform/robot/scripts/sim/`](../platform/robot/scripts/sim/) |

## Cómo compilarlo y desplegarlo

Todo pasa por [Task](https://taskfile.dev). Los comandos completos están en la sección
[Arranque rápido y reproducibilidad](../README.md#arranque-rápido-y-reproducibilidad) del
README principal. Los tres más usados:

```bash
task platform:install          # Instalar dependencias
task platform:test             # Ejecutar todas las pruebas
task platform:robot:deploy     # Desplegar al robot por SSH
```
