STATUS: Software compiles and runs, Nov 21,2024. Prototype PCB hardware needs adjustments.

##  Contents

See the auto-generated menu in the github README display (above right).

## Summary

This crate has code for a hardware design as in repository
https://github.com/pdgilbert/SensorProject_t16-f401-pcb.
It measures temperature from sixteen 10K NTC 3950 sensors using an adc. 
The measurements are transmitted by LoRa and optionally displayed on an SSD1306.

The code here is based on code in https://github.com/pdgilbert/rust-integration-testing/examples/projects.
Relative to that repository, this crate is much simplified by targetting specific hardware
and a specific MCU (stm32f401). 
The intention is that code here should be stable and remain working for the hardware.
The code in repository `rust-integration-testing` is intended for testing new versions of 
crates and hals, and as a result is sometimes broken.

## Building

```
cargo build --no-default-features --target thumbv7em-none-eabihf --features stm32f401,stm32f4xx 
```

## Loading

If `openocd`, `gdb`, `.cargo/config` with needed runners, and an appropriate probe are 
in place then in one window run

```
openocd -f interface/stlink-v2.cfg  --target target/stm32f4x.cfg 
```
Adjust interface for your programming dongle.

In another window do
```
cargo  run --target thumbv7em-none-eabihf --features stm32f401,stm32f4xx  [ --release]
```
The `--release` will be needed if code is too big for memory.
cargo  run --target $TARGET --features $HAL,$MCU  [ --release]

## License

Licensed under either of

 * Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or
   http://www.apache.org/licenses/LICENSE-2.0)
 * MIT license ([LICENSE-MIT](LICENSE-MIT) or
   http://opensource.org/licenses/MIT)

at your option.

## Contributing

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you, as defined in the Apache-2.0 license, shall
be dual licensed as above, without any additional terms or conditions.
