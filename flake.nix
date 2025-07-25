{
  description = "ESP32 Dev shell";

  inputs = {
    nixpkgs.url = "nixpkgs/nixos-25.05";
    esp-dev.url = "github:mirrexagon/nixpkgs-esp-dev";
  };

  outputs = {
    self,
    nixpkgs,
    ...
  } @ inputs: let
    system = "x86_64-linux";
    pkgs = import nixpkgs {
      inherit system;
      overlays = [
        (import "${inputs.esp-dev}/overlay.nix")
      ];
    };
  in {
    devShells.${system}.default = pkgs.mkShell {
      buildInputs = [
        # Deps and libs
        pkgs.esp-idf-full
      ];

      shellHook = ''
        echo "Welcome to Shell!"
      '';

      env = {
      };
    };
  };
}
