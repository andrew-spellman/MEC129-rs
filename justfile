deploy:
    cargo build --release
    elf2uf2-rs target/thumbv6m-none-eabi/release/MEC129 target/thumbv6m-none-eabi/release/MEC129.ef2
    cp target/thumbv6m-none-eabi/release/MEC129.uf2 /run/media/$USER/RPI-RP2/
