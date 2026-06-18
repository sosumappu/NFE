use std::env;
use std::path::PathBuf;

fn main() {
    println!("cargo:rerun-if-changed=proto/messages.proto");

    let out_dir = PathBuf::from(env::var("OUT_DIR").unwrap());

    let mut config = prost_build::Config::new();

    config.file_descriptor_set_path(out_dir.join("messages_descriptor.bin"));

    config
        .compile_protos(
            &["proto/car_software.proto", "proto/foxglove.proto"],
            &["proto/"],
        )
        .unwrap();
}
