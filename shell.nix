with import <nixpkgs> {};
mkShell {
    buildInputs = [
	      gcc
        llvmPackages.bintools
        pkg-config
        gurobi
        cargo
    ];
    allowUnfree = true; 
    shellHook = ''
      export GUROBI_HOME="${gurobi}"
      export LD_LIBRARY_PATH="$GUROBI_HOME/lib"
    ''; 
}
