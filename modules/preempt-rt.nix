# https://lore.kernel.org/linux-rt-users/d445e582-4e49-4135-9d94-f52d72ec5df6@gmx.net/
#
{
  config,
  pkgs,
  lib,
  ...
}: {
  nixpkgs.overlays = lib.mkAfter [
    (final: prev: {
      linuxPackages_rpi5_rt = let
        # Extend the rpi5 kernel packages with PREEMPT_RT config
        rtKernel = prev.linuxPackages_rpi5.kernel.override {
          structuredExtraConfig = with lib.kernel; {
            EXPERT = yes;
            PREEMPT_RT = yes;
            PREEMPT = lib.mkForce yes;
            PREEMPT_VOLUNTARY = lib.mkForce no;
            HZ_1000 = yes; # 1000Hz pour un meilleur control
            HZ = freeform "1000";
            HZ_250 = lib.mkForce no;
            HZ_300 = lib.mkForce no;
            HZ_500 = lib.mkForce no;
            HIGH_RES_TIMERS = yes;
            CPU_ISOLATION = yes;
            RT_GROUP_SCHED = lib.mkForce no; # enable FIFO
          };
          ignoreConfigErrors = true;
        };
      in
        prev.linuxPackagesFor rtKernel;
    })
  ];

  boot.kernelPackages = pkgs.linuxPackages_rpi5_rt;

  # Désactiver le real-time throttling
  # https://docs.kernel.org/scheduler/sched-rt-group.html
  boot.kernel.sysctl = {
    "kernel.sched_rt_runtime_us" = -1;
  };

  # Paramètres kernel
  boot.kernelParams = [
    # On isole le coeur n°3
    "isolcpus=3"
    "nohz_full=3"
    "rcu_nocbs=3"
    # On désactive le balancing sur le 4éme core
    "irqaffinity=0-2"
    # On désative le scaling des fréquences pour avoir une latence deterministe
    "cpufreq.default_governor=performance"
  ];

  # On désactive forcefully le partage équitable des taches
  services.irqbalance.enable = lib.mkForce false;

  # On persist le journald
  services.journald.extraConfig = ''
    Storage=persistent
    RateLimitIntervalSec=0
    RateLimitBurst=0
    SystemMaxUse=512M
  '';
}
