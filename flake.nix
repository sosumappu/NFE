{
  description = ''NeverFastEnough'';

  nixConfig = {
    bash-prompt = "\\[nfe\\] ➜ ";
    extra-substituters = [
      "https://nixos-raspberrypi.cachix.org"
      "https://nix-community.cachix.org"
    ];
    extra-trusted-public-keys = [
      "nixos-raspberrypi.cachix.org-1:4iMO9LXa8BqhU+Rpg6LQKiGa2lsNh/j2oiYLNOQ5sPI="
      "nix-community.cachix.org-1:mB9FSh9qf2dCimDSUo8Zy7bkq5CX+/rkCWyvRCUSeBc="
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
      inherit (pkgsFor system) car-software;
      default = (pkgsFor system).car-software;
    });

    # ── NixOS configuration: nfe ──────────────────────────────────
    nixosConfigurations.nfe = nixos-raspberrypi.lib.nixosSystemFull {
      specialArgs = inputs;
      modules = [
        ./hosts/nfe/configuration.nix
        {
          nixpkgs.overlays = [self.overlays.default];
          nixpkgs.crossSystem = nixpkgs.lib.systems.elaborate targetSystem;
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
      sshOpts = ["-o" "StrictHostKeyChecking=accept-new"];
      magicRollback = true;
      autoRollback = true;
      confirmTimeout = 60;

      profiles.system = {
        user = "root";
        path =
          deploy-rs.lib.${targetSystem}.activate.nixos
          self.nixosConfigurations.nfe;
      };
    };

    # deploy-rs schema checks
    checks = forSystems (
      system:
        deploy-rs.lib.${system}.deployChecks self.deploy
    );

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
          ]
          ++ pkgs.lib.optionals pkgs.stdenv.hostPlatform.isLinux [
            rt-tests
          ];

        shellHook = ''
          echo ""
          echo "  NeverFastEnough dev shell"
          echo "  ─────────────────────────────────────────────────────"
          echo "  nom build .#packages.aarch64-linux.car-software   build binary"
          echo "  deploy .#nfe                                      deploy to Pi"
          echo "  deploy .#nfe -- --rollback                        roll back"
          echo "  ssh localhost@nfe.local                           shell on Pi"
          echo "  ssh localhost@nfe.local 'journalctl -u car -f'"
          echo "  ssh localhost@nfe.local 'systemctl restart car'"
          echo "  ssh localhost@nfe.local 'cyclictest -p80 -t -n -q'"
          echo "  ─────────────────────────────────────────────────────"
          echo ""
        '';
      };
    });
  };
}
