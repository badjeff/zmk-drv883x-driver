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

    /* map PWM gpios to IN1/2 pin on drv883x */
    /* set N in <&pwm N ...>, where N is refering to group<N> from above &pinctrl */
    /* PWM_DT_SPEC_GET_BY_IDX should assign pwm1/2 by the index of array */
    pwms = <&pwm 1 PWM_MSEC(20) PWM_POLARITY_NORMAL>
         , <&pwm 2 PWM_MSEC(20) PWM_POLARITY_NORMAL>;

    /* either use plain gpios if always run at full speed */
    // in1-gpios = <&zero_header 1 (GPIO_ACTIVE_HIGH | GPIO_PULL_DOWN)>;
    // in2-gpios = <&zero_header 2 (GPIO_ACTIVE_HIGH | GPIO_PULL_DOWN)>;
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
        // param1: channel = < 0 = PWM1/2 or IN1/2, 1 = PWM3/4 & IN3/4 >
        // param2: velocity = < min ... neutral ... max >

        //*** NOTE ***
        // Currently `param1` has no effect, defineing one more "ti,drv883x" device
        // for IN3/4 will workaround.
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
