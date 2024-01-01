# RP2040 Embassy Snippets

Some small programs for the RP2040 chip using the Embassy framework and the `embassy-rp` HAL.

## Requirements

- Rust nightly toolchain for the `thumbv6m-none-eabi` target tripple: `rustup target add thumbv6m-none-eabi`.
- `probe-rs`: see <https://probe.rs/>.

## Running the examples

```shell
cargo embed --release --bin <NAME>
```

where `<NAME>` is the name of the example in `src/bin`.
