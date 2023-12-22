
with import <nixpkgs> {};
let
 gurobi_fix = gurobi.override { python3 = python310; };

in

mkShell {
    buildInputs = [
	gcc
        llvmPackages.bintools
        pkg-config
        gurobi_fix
        rustup
        xdot

    ];
    allowUnfree = true; 
    shellHook = ''
      export GUROBI_HOME="${gurobi_fix}"
      export LD_LIBRARY_PATH="$GUROBI_HOME/lib"
    ''; 
}
