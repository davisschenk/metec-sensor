cargo build --release --target aarch64-unknown-linux-gnu

scp target/aarch64-unknown-linux-gnu/release/sensor davis@raspberrypi.local:~/sensor
scp .env davis@raspberrypi.local:~/.env
