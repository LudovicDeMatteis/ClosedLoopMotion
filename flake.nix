{
  description = "dev environment for Closed Loop Motion generation";

  inputs = {
    flake-parts.url = "github:hercules-ci/flake-parts";
    # ndcurves is not yet upstream
    nixpkgs.url = "github:gepetto/nixpkgs/master";
    sobec = {
      url = "github:LudovicDeMatteis/sobec/icra-2025";
      inputs = {
        flake-parts.follows = "flake-parts";
        nixpkgs.follows = "nixpkgs";
      };
    };
    example-parallel-robots = {
      url = "github:gepetto/example-parallel-robots";
      inputs = {
        flake-parts.follows = "flake-parts";
        nixpkgs.follows = "nixpkgs";
      };
    };
    toolbox-parallel-robots = {
      url = "github:gepetto/toolbox-parallel-robots";
      inputs = {
        flake-parts.follows = "flake-parts";
        nixpkgs.follows = "nixpkgs";
      };
    };
  };

  outputs =
    inputs:
    inputs.flake-parts.lib.mkFlake { inherit inputs; } {
      systems = [ "x86_64-linux" ];
      perSystem =
        { pkgs, self', system, ... }:
        {
          devShells.default =
            with pkgs;
            mkShell {
              packages = [
                git
                (python3.withPackages (p: [
                  p.ipython
                  p.meshcat
                  p.ndcurves
                  p.matplotlib
                  p.tkinter
                  p.qpsolvers
                  p.proxsuite
                  p.tqdm
                  p.example-robot-data
                  self'.packages.example-parallel-robots
                  self'.packages.sobec
                  self'.packages.toolbox-parallel-robots
                ]))
              ];
            };
          packages = {
            inherit (inputs.sobec.packages.${system}) sobec;
            inherit (inputs.example-parallel-robots.packages.${system}) example-parallel-robots;
            inherit (inputs.toolbox-parallel-robots.packages.${system}) toolbox-parallel-robots;
            docker = pkgs.dockerTools.buildNixShellImage {
              name = "ghcr.io/ludovicdematteis/closedloopmotion";
              tag = "latest";
              drv = self'.devShells.default;
            };
          };
        };
    };
}
