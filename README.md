# DRV883x Driver Module for ZMK

This module exposes DRV883x inputs via Zephyr's `sensor_driver_api` and key press behavior.

## Installation

Include this project on ZMK's west manifest in `config/west.yml`:

```yml
manifest:
  remotes:
    #...
    - name: badjeff
      url-base: https://github.com/badjeff
    #...
  projects:
    #...
    - name: zmk-drv883x-driver
      remote: badjeff
      revision: main
    #...
  self:
    path: config
```

Update `board.overlay` adding the necessary bits (update the pins for your board accordingly):

```dts
/{
  /* setup the gpios to operate drv883x */
  drv883x_0: drv883x {
    compatible = "ti,drv883x";

    enable-gpios = <&zero_header 0 (GPIO_ACTIVE_HIGH | GPIO_PULL_DOWN)>;

    /* map PWM gpios to IN1/2 and IN3/4 pins on drv883x */
    /* IN1/2 shall be controlled via behavior param1 0 in 'zmk,behavior-drv883x' */
    /* IN3/4 shall be controlled via behavior param1 1 in 'zmk,behavior-drv883x' if defined */
    pwms = <&pwm 1 PWM_MSEC(20) PWM_POLARITY_NORMAL>
         , <&pwm 2 PWM_MSEC(20) PWM_POLARITY_NORMAL>
         //  , <&pwm 3 PWM_MSEC(20) PWM_POLARITY_NORMAL>
         //  , <&pwm 4 PWM_MSEC(20) PWM_POLARITY_NORMAL>
         ;
  };
};
```

Update `board.keymap` for key press behavior:

```keymap
/{
    /* setup key press behavior, bind the drv883x driver */
    ins883x: behavior_drv883x_0 {
        compatible = "zmk,behavior-drv883x";
        #binding-cells = <2>;

        /* assign a drv883x device */
        drv883x-dev = <&drv883x_0>;

        /* define velocity scope of param2, e.g. range of 265, 128 as neutral point */
        vel-neutral = <128>;
        vel-min-max = <128>;
        // param1: channel = < 0 = IN1/2, 1 = IN3/4 >
        // param2: velocity = < min ... neutral ... max >
    };

    keymap {
        compatible = "zmk,keymap";
        default_layer {
                bindings = <
                    &ins883x 0 160 // channel 0, speed +32 (forward)
                    &ins883x 0 128 // channel 0, speed 0   (neutral)
                    &ins883x 0 96  // channel 0, speed -32 (reverse)
                >;
        };
    };
};
```

Enable the driver config in `<shield>.config` file (read the Kconfig file to find out all possible options):

```conf
# Enable PWM
CONFIG_PWM=y
CONFIG_PWM_LOG_LEVEL_DBG=y

# Enable debug logging
CONFIG_DRV883X_LOG_LEVEL_DBG=y
```
