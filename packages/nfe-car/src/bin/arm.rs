use std::net::UdpSocket;
use std::time::Duration;

use anyhow::{bail, Context, Result};

#[derive(Clone, Copy, Debug)]
enum Command {
    Arm,
    Disarm,
}

impl Command {
    fn as_str(self) -> &'static str {
        match self {
            Command::Arm => "arm",
            Command::Disarm => "disarm",
        }
    }
}

struct Args {
    command: Command,
    host: String,
    port: u16,
    token: String,
}

impl Args {
    fn parse() -> Result<Self> {
        let raw: Vec<String> = std::env::args().collect();
        let get = |flag: &str| -> Option<String> {
            raw.windows(2).find(|w| w[0] == flag).map(|w| w[1].clone())
        };
        let has = |flag: &str| raw.iter().any(|a| a == flag);

        let arm = has("--arm");
        let disarm = has("--disarm");
        let command = match (arm, disarm) {
            (true, false) => Command::Arm,
            (false, true) => Command::Disarm,
            _ => bail!(
                "usage: nfe-arm (--arm|--disarm) [--host <ip>] [--port <port>] [--token <token>]"
            ),
        };

        Ok(Self {
            command,
            host: get("--host").unwrap_or_else(|| "127.0.0.1".to_string()),
            port: get("--port").and_then(|v| v.parse().ok()).unwrap_or(4578),
            token: get("--token").unwrap_or_else(|| "nfe".to_string()),
        })
    }
}

fn main() -> Result<()> {
    let args = Args::parse()?;
    let addr = format!("{}:{}", args.host, args.port);
    let payload = format!("NFE_ARM {} {}\n", args.token, args.command.as_str());
    let socket = UdpSocket::bind("0.0.0.0:0").context("bind UDP client socket")?;
    socket
        .set_write_timeout(Some(Duration::from_secs(1)))
        .context("configure UDP write timeout")?;
    socket
        .send_to(payload.as_bytes(), &addr)
        .with_context(|| format!("send arm command to {addr}"))?;
    println!("nfe-arm: sent {} to {addr}", args.command.as_str());
    Ok(())
}
