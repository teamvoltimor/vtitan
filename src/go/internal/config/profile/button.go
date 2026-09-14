package profile

// DefaultButtonGPIOTOMLPath is
// src/config/hardware/button/gpio.toml, relative to the repo
// root. The file's shape is the generated button.HardwareButtonGpio DTO, and
// its [button] section is button.HardwareButtonGpioButton; button_gpio_pin is
// a top-level key (not sectioned).
const DefaultButtonGPIOTOMLPath = "src/config/hardware/button/gpio.toml"

// DefaultButtonNodeTOMLPath is
// src/config/hardware/button/button_node.toml, relative to the
// repo root -- the ROS2 node's own poll cadence, a separate file from
// gpio.toml's driver-level debounce/threshold config, with the generated
// button.HardwareButtonButtonNode shape.
const DefaultButtonNodeTOMLPath = "src/config/hardware/button/button_node.toml"
