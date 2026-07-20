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

    ../../modules/preempt-rt.nix # patch pour le real time
    ../../modules/car-service.nix # car.service systemd unit
  ];

  networking = {hostName = lib.mkForce "nfe";};

  time.timeZone = "UTC";

  users.users.localhost = {
    isNormalUser = true;
    extraGroups = ["wheel" "gpio" "i2c" "spi" "dialout" "tty" "video"];
    # Set a real password with: passwd localhost  — or use hashedPassword
    initialHashedPassword = ""; # passwordless for initial flash; harden post-deploy
    openssh.authorizedKeys.keys = [
      "ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAIOF1uj9DgHdyYxOezFk2GhrgdFR8DWoXXVr/O2g2CMfG adelarab.works@gmail.com"
      "ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAIPkUWf9LaO+xCEDLsUBTWiEZTWdfiWaj2jo3x6qhI1Ao nix-daemon-builder"
    ];
  };

  users.users.root = {
    initialHashedPassword = "";
    openssh.authorizedKeys.keys = [
      "ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAIOF1uj9DgHdyYxOezFk2GhrgdFR8DWoXXVr/O2g2CMfG adelarab.works@gmail.com"
      "ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAIPkUWf9LaO+xCEDLsUBTWiEZTWdfiWaj2jo3x6qhI1Ao nix-daemon-builder"
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
    allowedUDPPorts = [67 5353]; # DHCP + mDNS — reach as nfe.local
    logRefusedConnections = false;
  };
  systemd.network.networks = {
    "20-end0-debug" = {
      matchConfig.Name = "end0";
      address = ["192.168.50.3/24"];
      networkConfig = {
        LinkLocalAddressing = "yes";
        MulticastDNS = "yes";
      };
    };
    "30-wlan0-ap" = {
      matchConfig.Name = "wlan0";
      address = ["192.168.51.1/24"];
      networkConfig = {
        DHCPServer = "yes";
        LinkLocalAddressing = "no";
        MulticastDNS = "yes";
      };
      dhcpServerConfig = {
        PoolOffset = 10;
        PoolSize = 40;
        EmitDNS = false;
        EmitRouter = false;
        PersistLeases = true;
      };
    };
  };

  networking = {
    # The Pi exposes its own AP for field deploy/debug. Deploy-rs copies the
    # locally-built closure over SSH, so the car does not need upstream WiFi.
    wireless.enable = lib.mkForce false;
    networkmanager.enable = lib.mkForce false;
  };

  services.hostapd = {
    enable = true;
    radios.wlan0 = {
      band = "2g";
      channel = 6;
      networks.wlan0 = {
        ssid = "NFE";
        authentication = {
          mode = "wpa2-sha256";
          wpaPassword = "neverfastenough";
        };
      };
    };
  };

  services.avahi = {
    enable = true;
    nssmdns4 = true;
    openFirewall = true;
    publish = {
      enable = true;
      addresses = true;
      workstation = true;
    };
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
  boot.loader.raspberry-pi.bootloader = "kernel";

  # ── car.service ───────────────────────────────────────────────
  # Enabled here; package wired in flake.nix once nfe-car builds
  services.car = {
    enable = true;
    package = pkgs.nfe-car;
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
    nfe-car
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
