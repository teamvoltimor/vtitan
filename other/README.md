# `other/` - Todo lo demás

Herramientas, infraestructura y documentación de apoyo que no son material de
competencia. La raíz del repositorio queda con las carpetas obligatorias de la
WRO y con `src/` (el robot); todo lo demás vive aquí dentro.

| Qué | Dónde |
|-----|-------|
| Documentación de apoyo (bitácora, documento de ingeniería WRO, guía de las Raspberry Pi) | [`docs/`](docs/) |
| Hojas de datos de los componentes | [`docs/reference/datasheets/`](docs/reference/datasheets/) |
| Historial de prototipos anteriores (Klevor v0.1 a v1.0) | [`docs/development/previous-prototypes/`](docs/development/previous-prototypes/) |
| Entrenamiento, cuantización y compilación del detector YOLO a HEF | [`ml/hailo/`](ml/hailo/) |
| Pesos publicados del detector (ONNX, HEF, PyTorch) | [`ml/weights/`](ml/weights/) |
| Backend de telemetría (Go, HTTP + gRPC) | [`apps/backend/`](apps/backend/) |
| Panel de telemetría (dashboard React + Three.js) | [`apps/frontend/`](apps/frontend/) |
| Simulador Gazebo/ROS2 y generador de escenarios | [`apps/gazebo/`](apps/gazebo/) |
| Auto-annotator: anotación asistida con SAM2 | [`apps/auto-annotator/`](apps/auto-annotator/) |
| Contratos compartidos (protobuf + OpenAPI) consumidos por robot, backend y frontend | [`contracts/`](contracts/) |
| Provisionamiento de las Raspberry Pi con Ansible | [`deploy/ansible/`](deploy/ansible/) |
| Scripts de diagnóstico sobre bags MCAP | [`../src/python/scripts/bag/`](../src/python/scripts/bag/) |
| Salidas de ejecución (bags, fotos, videos); vacía en el repo | [`data/`](data/) |
| Logos e imágenes de la documentación | [`assets/`](assets/) |
| Tareas del Taskfile raíz (fleet, platform, infra) | [`tasks/`](tasks/) y [`../Taskfile.yml`](../Taskfile.yml) |
| Sitio de documentación | [`apps/hugo-docs/`](apps/hugo-docs/) |
| Página de presentación del proyecto | `apps/landing/` (ignorada por git por ahora; solo contiene un handoff de marca local) |

El código del robot no está bajo `other/`: vive en [`../src/`](../src/), porque
es material de competencia. Los contratos compartidos, en cambio, se movieron
aquí a [`contracts/`](contracts/) junto con sus consumidores (`apps/`), para que
la raíz quede solo con lo que pide la WRO más `src/`.
