{
  description = ''NeverFastEnough'';

  nixConfig = {
    bash-prompt = "\\[nfe\\] ➜ ";
    extra-substituters = [
      "https://cache.nixos.org"
      "https://nixos-raspberrypi.cachix.org"
      "https://nix-community.cachix.org"
    ];
    extra-trusted-public-keys = [
      "cache.nixos.org-1:6NCHdD59X431o0gWypbMrAURkbJ16ZPMQFGspcDShjY="
      "nixos-raspberrypi.cachix.org-1:4iMO9LXa8BqhU+Rpg6LQKiGa2lsNh/j2oiYLNOQ5sPI="
      "nix-community.cachix.org-1:mB9FSh9qf2dCimDSUo8Zy7bkq5CX+/rkCWyvRCYg3Fs="
    ];
    connect-timeout = 5;
  };

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";

    nixos-raspberrypi = {
      url = "github:nvmd/nixos-raspberrypi/main";
      inputs.nixpkgs.follows = "nixpkgs";
    };

    deploy-rs = {
      url = "github:serokell/deploy-rs";
      inputs.nixpkgs.follows = "nixpkgs";
    };

    fenix = {
      url = "github:nix-community/fenix";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  };

  outputs = {
    self,
    nixpkgs,
    nixos-raspberrypi,
    deploy-rs,
    fenix,
    ...
  } @ inputs: let
    allSystems = nixpkgs.lib.systems.flakeExposed;
    forSystems = f: nixpkgs.lib.genAttrs allSystems f;
    targetSystem = "aarch64-linux";

    pkgsFor = system:
      import nixpkgs {
        inherit system;
        overlays = [fenix.overlays.default self.overlays.default];
      };
  in {
    # ── Overlay: nfe-car + RT Rust toolchain ─────────────────
    overlays.default = final: prev: {
      rustToolchain = fenix.packages.${prev.stdenv.hostPlatform.system}.stable.toolchain;

      nfe-car = prev.callPackage ./packages/nfe-car {
        inherit (prev) systemd protobuf;
        rustPlatform = prev.makeRustPlatform {
          cargo = final.rustToolchain;
          rustc = final.rustToolchain;
        };
      };
    };

    #
    packages = forSystems (system: {
      inherit ((pkgsFor "aarch64-linux")) nfe-car;
      default = (pkgsFor "aarch64-linux").nfe-car;
    });

    # ── NixOS configuration: nfe ──────────────────────────────────
    nixosConfigurations.nfe = nixos-raspberrypi.lib.nixosSystemFull {
      system = "aarch64-linux";
      specialArgs = inputs;
      modules = [
        ./hosts/nfe/configuration.nix
        {
          nixpkgs.buildPlatform = "aarch64-linux";
          nixpkgs.overlays = [self.overlays.default];
          fileSystems."/" = {
            device = "/dev/disk/by-label/NIXOS_SD";
            fsType = "ext4";
            options = ["noatime"];
          };

          fileSystems."/boot/firmware" = {
            device = "/dev/disk/by-label/FIRMWARE";
            fsType = "vfat";
            options = ["noatime"];
          };

          zramSwap = {
            enable = true;
            algorithm = "zstd";
          };
        }
      ];
    };

    # deploy .#nfe                       — deploy latest
    # deploy .#nfe -- --rollback         — roll back one generation
    # deploy .#nfe -- --dry-activate     — dry run
    #
    deploy.nodes.nfe = {
      hostname = "nfe.local";
      sshUser = "localhost";
      sshOpts = ["-i" "/Users/localhost/.ssh/nix_builder" "-o" "StrictHostKeyChecking=accept-new"];
      magicRollback = false;
      autoRollback = false;
      confirmTimeout = 60;

      profiles.system = {
        user = "root";
        path =
          deploy-rs.lib.${targetSystem}.activate.nixos
          self.nixosConfigurations.nfe;
      };
    };

    # deploy-rs schema checks
    checks.${targetSystem} =
      deploy-rs.lib.${targetSystem}.deployChecks self.deploy;

    # ── Dev shell ──────────────────────────────────────────────────
    devShells = forSystems (system: let
      pkgs = pkgsFor system;
      findRepoRoot = ''
        find_repo_root() {
          dir="$PWD"
          while [ "$dir" != "/" ]; do
            if [ -f "$dir/tools/pyproject.toml" ] && [ -f "$dir/packages/nfe-car/nfe.toml" ]; then
              printf '%s\n' "$dir"
              return 0
            fi
            dir="$(dirname "$dir")"
          done
          echo "nfe tuning: cannot find repository root" >&2
          return 1
        }
      '';
      tune = pkgs.writeShellApplication {
        name = "tune";
        runtimeInputs = [pkgs.coreutils pkgs.uv pkgs.rustToolchain];
        text = ''
          ${findRepoRoot}
          root="$(find_repo_root)"
          cd "$root"
          mkdir -p runs/tuning
          if [ ! -x target/debug/car-tune ]; then
            cargo build -p nfe-car --bin car-tune
          fi
          exec uv run --project tools nfe-tune-optuna \
            --car-tune target/debug/car-tune \
            --sim worlds/tracks/awake1.json \
            --config packages/nfe-car/nfe.toml \
            --trials 500 \
            --storage sqlite:///runs/tuning/nfe-optuna.db \
            --trial-dir runs/tuning/trials \
            --out runs/tuning/optuna-best-runtime-config.json \
            "$@"
        '';
      };
      tuner-plot = pkgs.writeShellApplication {
        name = "tuner-plot";
        runtimeInputs = [pkgs.coreutils pkgs.uv];
        text = ''
          ${findRepoRoot}
          root="$(find_repo_root)"
          cd "$root"
          mkdir -p runs/tuning/plots
          exec uv run --project tools nfe-plot-optuna \
            --storage sqlite:///runs/tuning/nfe-optuna.db \
            --study nfe-tpe \
            --out-dir runs/tuning/plots \
            "$@"
        '';
      };
    in {
      default = pkgs.mkShell {
        name = "nfe-car-dev";

        nativeBuildInputs = with pkgs;
          [
            nil
            nixpkgs-fmt
            nix-output-monitor
            deploy-rs.packages.${system}.default
            fenix.packages.${system}.stable.toolchain
            pkg-config
            protobuf
            mcap-cli
            cargo-audit
            uv
            tune
            tuner-plot
          ]
          ++ pkgs.lib.optionals pkgs.stdenv.hostPlatform.isLinux [
            rt-tests
            fontconfig
            stdenv.cc.cc.lib
            udev
          ];

        shellHook =
          ''
            echo ""
            echo "  NeverFastEnough dev shell"
            echo "  ─────────────────────────────────────────────────────────────────"
            echo "  Build"
            echo "    nom build .#packages.aarch64-linux.nfe-car  build all binaries"
            echo "    cross build --release --target aarch64-unknown-linux-gnu  fast binary build"
            echo ""
            echo "  Deploy"
            echo "    deploy .#nfe                                      deploy to Pi"
            echo "    deploy .#nfe -- --rollback                        roll back one generation"
            echo "    deploy .#nfe -- --dry-activate                    dry run"
            echo ""
            echo "  On the Pi (via SSH)"
            echo "    ssh localhost@nfe.local 'car-diag all'            verify all sensors"
            echo "    ssh localhost@nfe.local 'car-diag imu --once'     single IMU reading"
            echo "    ssh localhost@nfe.local 'car-diag lidar'          live LIDAR stats"
            echo "    ssh localhost@nfe.local 'car-diag sonar --once'   sonar pass/fail"
            echo "    ssh localhost@nfe.local 'car --record /tmp/s.bin' record a session"
            echo "    ssh localhost@nfe.local 'nfe-arm --arm --host nfe.local' arm StartGate"
            echo "    ssh localhost@nfe.local 'systemctl restart car'   restart service"
            echo "    ssh localhost@nfe.local 'journalctl -u car -f'    live service logs"
            echo ""
            echo "  On Dev Machine"
            echo "    cargo run --bin car-monitor -- --pi nfe.local     live dashboard"
            echo "    tune                                             run Optuna apex tuning from repo root"
            echo "    tuner-plot                                       plot Optuna tuning results"
            echo "    cargo run --release -- --replay sessions/s.bin    replay session"
            echo "    cargo run --release -- --replay sessions/s.bin --fast  fast replay"
            echo "    scp localhost@nfe.local:/tmp/s.bin sessions/      copy session file"
            echo ""
            echo "  RT verification"
            echo "    ssh localhost@nfe.local 'cyclictest -p80 -t1 -a3 -n -q -D60'  latency test"
            echo "    ssh localhost@nfe.local 'cat /sys/devices/system/cpu/isolated' check core isolation"
            echo "  ─────────────────────────────────────────────────────────────────"
            echo ""
          ''
          + pkgs.lib.optionalString pkgs.stdenv.isLinux ''
            export LD_LIBRARY_PATH="${pkgs.lib.makeLibraryPath [
              pkgs.stdenv.cc.cc.lib
              pkgs.fontconfig
            ]}:$LD_LIBRARY_PATH"
          '';
      };
    });
  };
}
