{
  config,
  pkgs,
  lib,
  ...
}: {
  options.services.car = {
    enable = lib.mkEnableOption "autonomous car control service";

    package = lib.mkOption {
      type = lib.types.package;
      description = "The car control binary package";
    };

    requireStartGate = lib.mkOption {
      type = lib.types.bool;
      default = true;
      description = "if true, services requires /run/car/START to exist
            before running";
    };

    logRingBufferSize = lib.mkOption {
      type = lib.types.str;
      default = "10M";
      description = "Size of the in-memory log ring buffer";
    };

    configFile = lib.mkOption {
      type = lib.types.str;
      default = "${config.services.car.package}/share/nfe-car/nfe.toml";
      description = "TOML configuration passed to the car binary";
    };
  };

  config = lib.mkIf config.services.car.enable {
    security.pam.loginLimits = [
      {
        domain = "car";
        type = "-";
        item = "memlock";
        value = "unlimited";
      }
      {
        domain = "car";
        type = "-";
        item = "rtprio";
        value = "99";
      }
    ];

    # Dedicated user with GPIO/I2C/SPI/UART access
    users.users.car = {
      isSystemUser = true;
      group = "car";
      extraGroups = ["gpio" "i2c" "spi" "dialout" "tty"];
      description = "Car control service user";
    };
    users.groups.car = {};
    users.groups.i2c = {};
    users.groups.gpio = {};
    users.groups.spi = {};

    systemd.services.car = {
      description = "Autonomous RC car control loop";
      documentation = ["https://github.com/sosumappu/nfe"];

      wantedBy =
        if config.services.car.requireStartGate
        then []
        else ["multi-user.target"];
      after = ["network.target" "systemd-udevd.service"];
      requires = ["systemd-udevd.service"];

      serviceConfig = {
        ExecStart = "${config.services.car.package}/bin/car --config ${config.services.car.configFile}";

        User = "car";
        Group = "car";

        # augmente la priorité du SHCED_FIFO à 50.
        CPUSchedulingPolicy = "fifo";
        CPUSchedulingPriority = 50;
        CPUAffinity = "3"; # pin le coeur

        # ── Memory locking ────────────────────────────────────────
        # Prevents page faults during the control loop
        LimitMEMLOCK = "infinity";

        # ── Restart policy ────────────────────────────────────────
        Restart = "always";
        RestartSec = "10s";

        # ── Watchdog integration ──────────────────────────────────
        # systemd kills the service if it doesn't notify within 5s
        # The Rust watchdog calls sd_notify("WATCHDOG=1") each tick
        WatchdogSec = "2s";
        NotifyAccess = "main";

        # ── Logging ───────────────────────────────────────────────
        # journald ring buffer — tail with: ssh nfe journalctl -u car -f
        LogRateLimitIntervalSec = 0;
        LogRateLimitBurst = 0;
        SyslogIdentifier = "car";

        # ── Capabilities ──────────────────────────────────────────
        # AmbientCapabilities = ["CAP_SYS_NICE" "CAP_IPC_LOCK"];
        # CapabilityBoundingSet = ["CAP_SYS_NICE" "CAP_IPC_LOCK"];
        #
        # # ── Security hardening (compatible with RT) ───────────────
        # NoNewPrivileges = true;
        # ProtectSystem = "strict";
        # ProtectHome = true;
        # PrivateTmp = true;
        # # Allow GPIO/I2C/UART device access
        # DeviceAllow = [
        #   "/dev/gpiochip0 rw"
        #   "/dev/i2c-1 rw"
        #   "/dev/ttyAMA0 rw"
        #   "/dev/ttyUSB0 rw"
        #   "/dev/pwmchip0 rw"
        #   "/dev/pwmchip1 rw"
        #   "/dev/pwmchip2 rw"
        #   "/dev/pwmchip3 rw"
        #   "/dev/gpiochip4 rw"
        #   "/dev/gpiomem4 rw"
        # ];
        # ReadWritePaths = ["/sys/class/pwm"];
        # DevicePolicy = "auto";
      };

      # Environment passed to the control binary
      environment = {
        RUST_LOG = "info";
        CAR_CONTROL_HZ = "100";
        CAR_WATCHDOG_TICKS = "3";
      };
    };

    # udev rules: expose GPIO/I2C/PWM to car group without root
    services.udev.extraRules = ''
      # GPIO
      SUBSYSTEM=="gpio", GROUP="gpio", MODE="0660"

      KERNEL=="gpiomem*", GROUP="gpio", MODE="0660"
      SUBSYSTEM=="bcm2835-gpiomem", GROUP="gpio", MODE="0660"
      # I2C (IMU)
      SUBSYSTEM=="i2c-dev", GROUP="i2c", MODE="0660"
      # LIDAR UART
      SUBSYSTEM=="tty", ATTRS{product}=="RPLidar*", GROUP="dialout", MODE="0660", SYMLINK+="lidar"
      SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", GROUP="dialout", MODE="0660", SYMLINK+="lidar"
      # PWM sysfs — the pwmchip itself is group-writable, but the per-channel
      # directories created by export are root:root 644 by default. This rule
      # runs after each channel is exported and fixes the permissions so the
      # car user (gpio group) can write period/duty_cycle/enable.
      SUBSYSTEM=="pwm", ACTION=="add", RUN+="/bin/sh -c 'chgrp -R gpio /sys%p && chmod -R g+rw /sys%p'"
    '';
  };
}
