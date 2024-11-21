
#![no_std]

#![feature(type_alias_impl_trait)]   //  for impl Trait` in type aliases is unstable

#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;

pub mod lora;  // has structures and constants
