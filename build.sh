# HOST=metec-pi.local
# USER=metec

HOST=10.85.174.20
USER=metec

# HOST=raspberrypi.local
# USER=davis

cargo build --release --target aarch64-unknown-linux-gnu

scp target/aarch64-unknown-linux-gnu/release/sensor $USER@$HOST:~/sensor
# scp .env $USER@$HOST:~/.env
