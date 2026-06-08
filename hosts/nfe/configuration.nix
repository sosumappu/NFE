{
  config,
  pkgs,
  lib,
  nixos-raspberrypi,
  ...
}: {
  imports = [
    nixos-raspberrypi.nixosModules.raspberry-pi-5.base
    nixos-raspberrypi.nixosModules.raspberry-pi-5.page-size-16k

    ../modules/preempt-rt.nix # patch pour le real time
    ../modules/car-service.nix # car.service systemd unit
  ];

  networking.hostName = "nfe";
  time.timeZone = "UTC";

  users.users.localhost = {
    isNormalUser = true;
    extraGroups = ["wheel" "gpio" "i2c" "spi" "dialout" "tty" "video"];
    # Set a real password with: passwd localhost  — or use hashedPassword
    initialHashedPassword = ""; # passwordless for initial flash; harden post-deploy
    openssh.authorizedKeys.keys = [
      "ssh-ed25519
AAAAC3NzaC1lZDI1NTE5AAAAIOF1uj9DgHdyYxOezFk2GhrgdFR8DWoXXVr/O2g2CMfG
adelarab.works@gmail.com"
    ];
  };

  users.users.root = {
    initialHashedPassword = "";
    openssh.authorizedKeys.keys = [
      "ssh-ed25519
AAAAC3NzaC1lZDI1NTE5AAAAIOF1uj9DgHdyYxOezFk2GhrgdFR8DWoXXVr/O2g2CMfG
adelarab.works@gmail.com"
    ];
  };

  security.sudo = {
    enable = true; # sudo snas password pour wheels
    wheelNeedsPassword = false;
  };
  security.polkit.enable = true;

  services.getty.autologinUser = "localhost";

  services.openssh = {
    enable = true;
    settings = {
      PermitRootLogin = "prohibit-password"; # key-only root
      PasswordAuthentication = false;
      KbdInteractiveAuthentication = false;
    };
    extraConfig = ''
      AllowUsers localhost root
    '';
  };

  systemd.services.sshd.stopIfChanged = false;

  networking.useNetworkd = true;
  networking.firewall = {
    enable = true;
    allowedTCPPorts = [22];
    allowedUDPPorts = [5353]; # mDNS — reach as nfe.local
    logRefusedConnections = false;
  };
  systemd.network.networks = {
    "99-ethernet-default-dhcp".networkConfig.MulticastDNS = "yes";
    "99-wireless-client-dhcp".networkConfig.MulticastDNS = "yes";
  };
  networking = {
    wireless = {
      enable = true;
      networks = {
        "Freebox-0B1620" = {
          pskRaw = "caa3fbb62dfe3db0afd267a52470f178e574bf985145a1f51c0caaf0c54b13f9";
        };
      };
    };
    networkmanager.enable = lib.mkForce false;
  };

  hardware.i2c.enable = true; # IMU (6-DOF)
  hardware.raspberry-pi.config = {
    all = {
      # [all] conditional filter, https://www.raspberrypi.com/documentation/computers/config_txt.html#conditional-filters

      options = {
        # https://www.raspberrypi.com/documentation/computers/config_txt.html#enable_uart
        # in conjunction with `console=serial0,115200` in kernel command line (`cmdline.txt`)
        # creates a serial console, accessible using GPIOs 14 and 15 (pins
        #  8 and 10 on the 40-pin header)
        # enable_uart = {
        #   enable = true;
        #   value = true;
        # };
        # https://www.raspberrypi.com/documentation/computers/config_txt.html#uart_2ndstage
        # enable debug logging to the UART, also automatically enables
        # UART logging in `start.elf`
        # uart_2ndstage = {
        #   enable = true;
        #   value = true;
        # };
      };

      # Base DTB parameters
      # https://github.com/raspberrypi/linux/blob/a1d3defcca200077e1e382fe049ca613d16efd2b/arch/arm/boot/dts/overlays/README#L132
      base-dt-params = {
        i2c_arm = {
          enable = true;
          value = "on";
        };
      };

      dt-overlays = {
        # https://stackoverflow.com/questions/60066790/why-does-my-rpi-uart0-ttyama0-freezes-when-bluetooth-disabled
        # disable-bt = {
        #   enable = true; # Pas nécessaire car connecter via USB
        # };
        #https://github.com/raspberrypi/firmware/blob/master/boot/overlays/README
        pwm-2chan = {
          enable = true;
          params = {
            pin = {
              enable = true;
              value = 18;
            };
            pin2 = {
              enable = true;
              value = 19;
            };
            func = {
              enable = true;
              value = 2;
            };
          };
        };
      };
    };
  };

  # ── Boot / bootloader ─────────────────────────────────────────
  boot.tmp.useTmpfs = true;

  # ── car.service ───────────────────────────────────────────────
  # Enabled here; package wired in flake.nix once car-software builds
  services.car = {
    enable = true;
    package = pkgs.car-software;
  };

  environment.systemPackages = with pkgs; [
    tree
    htop
    i2c-tools
    usbutils
    pciutils
    strace
    # RT debugging
    rt-tests # cyclictest, hackbench
    ethtool
  ];

  # ── Nix settings (allow deploy-rs to push closures) ───────────
  nix.settings = {
    trusted-users = ["localhost" "root"];
    experimental-features = ["nix-command" "flakes"];
  };

  # ── State version ─────────────────────────────────────────────
  system.stateVersion = "24.11";

  system.nixos.tags = [
    "raspberry-pi-5"
    "preempt-rt"
    config.boot.kernelPackages.kernel.version
  ];
}
