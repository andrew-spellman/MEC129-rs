build: src/*
	cargo build --release

deploy: build
	elf2uf2-rs target/thumbv6m-none-eabi/release/MEC129 target/thumbv6m-none-eabi/release/MEC129.ef2
	cp target/thumbv6m-none-eabi/release/MEC129.uf2 /run/media/$(USER)/RPI-RP2/
	sync

term:
	picocom -b 115200 -f h /dev/ttyACM0

all: build
