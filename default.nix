{ system ? builtins.currentSystem
, pkgs ? import <nixpkgs> { inherit system; }
, rustPlatform ? pkgs.rustPlatform
}:

pkgs.callPackage ./packages/nfe-car {
  inherit rustPlatform;
  inherit (pkgs) systemd protobuf;
}
