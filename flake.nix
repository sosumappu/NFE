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
    # ── Overlay: car-software + RT Rust toolchain ─────────────────
    overlays.default = final: prev: {
      rustToolchain = fenix.packages.${prev.stdenv.hostPlatform.system}.stable.toolchain;

      car-software = prev.callPackage ./packages/car-software {
        inherit (prev) systemd libudev-zero;
        rustPlatform = prev.makeRustPlatform {
          cargo = final.rustToolchain;
          rustc = final.rustToolchain;
        };
      };
    };

    #
    packages = forSystems (system: {
      inherit ((pkgsFor "aarch64-linux")) car-software;
      default = (pkgsFor "aarch64-linux").car-software;
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
          ]
          ++ pkgs.lib.optionals pkgs.stdenv.hostPlatform.isLinux [
            rt-tests
          ];

        shellHook = ''
          echo ""
          echo "  NeverFastEnough dev shell"
          echo "  ─────────────────────────────────────────────────────────────────"
          echo "  Build"
          echo "    nom build .#packages.aarch64-linux.car-software  build all binaries"
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
          echo "    ssh localhost@nfe.local 'car --stream'            broadcast sensors UDP:9200"
          echo "    ssh localhost@nfe.local 'systemctl restart car'   restart service"
          echo "    ssh localhost@nfe.local 'journalctl -u car -f'    live service logs"
          echo ""
          echo "  On Dev Machine"
          echo "    cargo run --bin car-monitor -- --pi nfe.local     live dashboard"
          echo "    cargo run --release -- --replay sessions/s.bin    replay session"
          echo "    cargo run --release -- --replay sessions/s.bin --fast  fast replay"
          echo "    scp localhost@nfe.local:/tmp/s.bin sessions/      copy session file"
          echo ""
          echo "  RT verification"
          echo "    ssh localhost@nfe.local 'cyclictest -p80 -t1 -a3 -n -q -D60'  latency test"
          echo "    ssh localhost@nfe.local 'cat /sys/devices/system/cpu/isolated' check core isolation"
          echo "  ─────────────────────────────────────────────────────────────────"
          echo ""
        '';
      };
    });
  };
}
