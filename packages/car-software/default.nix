{
  lib,
  stdenv,
  rustPlatform,
  pkg-config,
  systemd, # libsystemd (sd_notify, watchdog)
  libudev-zero, # udev shim — lighter than full systemd for cross builds
}:
rustPlatform.buildRustPackage {
  pname = "car-software";
  version = "0.1.0";

  # Source is the package directory itself
  src = ./.;

  # Cargo.lock must exist at the source root
  cargoLock.lockFile = ./Cargo.lock;

  # Build both binaries defined in Cargo.toml:
  #   [[bin]] name = "car"      → $out/bin/car
  #   [[bin]] name = "car-diag" → $out/bin/car-diag
  cargoBuildFlags = ["--bins"];

  # pkg-config resolves libsystemd headers during build
  nativeBuildInputs = [pkg-config];

  # Runtime link dependencies
  buildInputs = [systemd];

  # ── Release profile ────────────────────────────────────────────
  # Matches [profile.release] in Cargo.toml:
  #   opt-level = 3, lto = "thin", codegen-units = 1, panic = "abort"
  # buildRustPackage always passes --release, so no extra flag needed.

  # ── Post-install ───────────────────────────────────────────────
  # Strip debug info to keep the closure small on the Pi's SD card.
  # The release profile already sets panic=abort; stripping is safe.
  postInstall = ''
    ${lib.optionalString (stdenv.hostPlatform != stdenv.buildPlatform) ''
      # Cross-strip: use the target strip, not the build-host one
      $STRIP $out/bin/car
      $STRIP $out/bin/car-diag
    ''}
  '';

  meta = {
    description = "NFE autonomous RC car control loop (PREEMPT_RT / Tokio)";
    homepage = "https://github.com/sosumappu/nfe";
    license = lib.licenses.mit;
    platforms = ["aarch64-linux"];
    mainProgram = "car";
  };
}
