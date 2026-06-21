fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut config = prost_build::Config::new();
    let out_dir = std::path::PathBuf::from(std::env::var("OUT_DIR")?);
    config.out_dir(&out_dir);
    config.file_descriptor_set_path(out_dir.join("foxglove_descriptor.bin"));
    config.compile_protos(&["proto/foxglove.proto"], &["proto"])?;
    println!("cargo:rerun-if-changed=proto/foxglove.proto");
    Ok(())
}
