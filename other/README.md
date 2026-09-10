# `other/` - Todo lo demás

Herramientas, infraestructura y documentación de apoyo que no encajan en las carpetas
obligatorias. Como el resto del repositorio, viven en su ubicación real dentro del
monorepo; esta carpeta indica dónde.

| Qué | Dónde |
|-----|-------|
| Documentación de apoyo (bitácora, documento de ingeniería WRO, guía de las Raspberry Pi) | [`docs/`](../docs/) |
| Hojas de datos de los componentes | [`docs/reference/datasheets/`](../docs/reference/datasheets/) |
| Historial de prototipos anteriores (Klevor v0.1 a v1.0) | [`docs/development/previous-prototypes/`](../docs/development/previous-prototypes/) |
| Entrenamiento, cuantización y compilación del detector YOLO a HEF | [`ml/hailo/`](../ml/hailo/) |
| Pesos publicados del detector (ONNX, HEF, PyTorch) | [`ml/weights/`](../ml/weights/) |
| Backend de telemetría (Go, HTTP + gRPC) | [`apps/backend/`](../apps/backend/) |
| Panel de telemetría (dashboard React + Three.js) | [`apps/frontend/`](../apps/frontend/) |
| Simulador Gazebo/ROS2 y generador de escenarios | [`apps/gazebo/`](../apps/gazebo/) |
| Auto-annotator: anotación asistida con SAM2 | [`apps/auto-annotator/`](../apps/auto-annotator/) |
| Contratos compartidos (protobuf + OpenAPI) consumidos por robot, backend, frontend y el generador Go | [`contracts/`](../contracts/) |
| Provisionamiento de las Raspberry Pi con Ansible | [`deploy/ansible/`](../deploy/ansible/) |
| Scripts de diagnóstico sobre bags MCAP | [`src/python/scripts/bag/`](../src/python/scripts/bag/) |
| Automatización completa del proyecto (un solo punto de entrada) | [`Taskfile.yml`](../Taskfile.yml) |
| Sitio de documentación | [`apps/hugo-docs/`](../apps/hugo-docs/) |
