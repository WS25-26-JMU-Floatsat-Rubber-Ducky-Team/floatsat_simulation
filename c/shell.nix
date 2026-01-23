{ pkgs ? import <nixpkgs> {} }:

pkgs.mkShell {
  buildInputs = [
    pkgs.python310
    pkgs.python310Packages.numpy
    pkgs.python310Packages.matplotlib
    pkgs.gcc
  ];
}
