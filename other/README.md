# `other/` - Todo lo demás

Herramientas, infraestructura y documentación de apoyo que no encajan en las carpetas
obligatorias. Como el resto del repositorio, viven en su ubicación real dentro del
monorepo; esta carpeta indica dónde.

| Qué | Dónde |
|-----|-------|
| Documentación de apoyo (bitácora, documento de ingeniería WRO, guía de las Raspberry Pi) | [`docs/`](../docs/) |
| Hojas de datos de los componentes | [`docs/reference/datasheets/`](../docs/reference/datasheets/) |
| Historial de prototipos anteriores (Klevor v0.1 a v1.0) | [`docs/development/previous-prototypes/`](../docs/development/previous-prototypes/) |
| Entrenamiento, cuantización y compilación del detector YOLO a HEF | [`hailo/`](../hailo/) |
| Pesos publicados del detector (ONNX, HEF, PyTorch) | [`ml-models/`](../ml-models/) |
| Auto-annotator: anotación asistida con SAM2 | [`auto-annotator/`](../auto-annotator/) |
| Provisionamiento de las Raspberry Pi con Ansible | [`ansible/`](../ansible/) |
| Scripts de diagnóstico sobre bags MCAP | [`platform/robot/scripts/bag/`](../platform/robot/scripts/bag/) |
| Automatización completa del proyecto (un solo punto de entrada) | [`Taskfile.yml`](../Taskfile.yml) |
| Sitio de documentación | [`hugo-docs/`](../hugo-docs/) |
