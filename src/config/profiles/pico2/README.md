# pico2

The actuation-board profile: a Pico 2 drives the steering servo and the drive
motor instead of the Pi Zero. It changes no robot fact, so this directory only
registers the name (the loaders reject a profile with no directory here). Its
one overlay is `src/config/hardware/profiles/pico2/board.toml`.

Stack it after the servo and motor profiles:
`VTITAN_HARDWARE_PROFILE=270deg-hiwonder-35kg,rev-hd-hex-motor-6000rpm,pico2`.

See `other/docs/adr/0098-pico-actuation-board-and-portable-cores.md`.
