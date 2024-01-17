# README

## Compile

Change interface name in `Raw_Socket::init()` from `wlp4s0` to your network interface and recompile.

```bash
cargo build
```

## run

```bash
sudo ./target/debug/rawsocket-rs # raw sockets require super user privileges
```

