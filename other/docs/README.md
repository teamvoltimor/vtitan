# Docs

Mapa de `other/docs/`. Cada carpeta es un tipo de documento y el tipo manda:
un ADR es una decisión, un backlog es una cola, una guía es un procedimiento
vigente.

| Carpeta | Qué guarda | Se edita después |
|---|---|---|
| `adr/` | Una decisión por fichero, `NNNN-slug.md`: contexto, opciones, decisión, consecuencias. Es el registro normativo y lo único que un clon nuevo tiene garantizado | Solo `Status` y `Superseded by` |
| `backlog/` | La cola de trabajo: `inbox.md`, `vtitan.md` (doing, next, later, someday) y `done/YYYY-MM.md` | Sí, a diario |
| `reference/` | Hojas de datos y material de consulta de los componentes | No |
| `development/` | Material de prototipos anteriores | No |
| `internal/` | Notas de trabajo (sesiones, auditorías, diagnósticos). No se versiona y puede estar desactualizado; un fichero versionado no lo cita como su razón | Sí, sin contrato |

Las guías vigentes viven hoy en la raíz de esta carpeta (`pi-setup.md`,
`runbook-guardias-de-sentido-offline.md`, `tests.md`,
`configuracion-toml-navegacion.md`, `documentacion-ingenieria-wro.md`,
`bitacora-ingenieria.md`). La adopción completa de la convención (carpetas
`guides/` y `methodology/`, frontmatter en los ADR, linter en CI) está en el
backlog como `DOCS-ARCH-1`, no hecha.

Reglas de convivencia con `other/docs/adr/0097-doc-ownership-and-drift-checks.md`:
la prosa versionada describe solo el presente que se puede comprobar, y el
backlog es la única lista de trabajo abierto (ADR 0100). `task docs:check`
comprueba que las rutas citadas existan y estén versionadas.
