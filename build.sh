HOST=metec-pi.local
USER=metec

# HOST=raspberrypi.local
# USER=davis

cargo build --release --target aarch64-unknown-linux-gnu

scp target/aarch64-unknown-linux-gnu/release/sensor $USER@$HOST:~/sensor
# scp .env $USER@$HOST:~/.env
