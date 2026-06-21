{
  lib,
  stdenv,
  rustPlatform,
  pkg-config,
  protobuf,
  systemd, # libsystemd (sd_notify, watchdog)
  fontconfig,
}:
rustPlatform.buildRustPackage {
  pname = "nfe-car";
  version = "0.1.1";

  # Build from the workspace root so path dependencies, the proc-macro crate,
  # and runtime build.rs protobuf generation are all visible to Cargo/Nix.
  src = ../..;
  cargoLock.lockFile = ../../Cargo.lock;

  # Build the full workspace. The lib-only crates are built as dependencies and
  # the wiring crate installs car, car-diag, car-tune, and nfe-arm.
  cargoBuildFlags = ["--workspace" "--bins"];
  cargoTestFlags = ["--workspace"];

  # prost-build in nfe-runtime/build.rs needs protoc at build time.
  nativeBuildInputs = [pkg-config protobuf];

  buildInputs = [systemd fontconfig];

  postInstall = ''
    ${lib.optionalString (stdenv.hostPlatform != stdenv.buildPlatform) ''
      $STRIP $out/bin/car
      $STRIP $out/bin/car-diag
      $STRIP $out/bin/car-tune
      $STRIP $out/bin/nfe-arm
    ''}
  '';

  meta = {
    description = "NeverFastEnough car control and optimization package";
    homepage = "https://github.com/sosumappu/nfe";
    license = lib.licenses.mit;
    platforms = ["aarch64-linux"];
    mainProgram = "car";
  };
}
