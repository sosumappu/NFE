use crate::{cli::Args, config::Config};

pub fn initialize(args: &Args) -> Config {
    lock_memory();
    Config::load(args.config.as_deref())
}

pub fn lock_memory() {
    #[cfg(target_os = "linux")]
    unsafe {
        let flags: libc::c_int = if std::env::var("JOURNAL_STREAM").is_ok() {
            3
        } else {
            1
        };
        extern "C" {
            fn mlockall(flags: libc::c_int) -> libc::c_int;
        }
        if mlockall(flags) != 0 {
            eprintln!("mlockall failed — check LimitMEMLOCK=infinity");
        } else {
            tracing::info!("memory: pages locked (flags={})", flags);
        }
    }
}
