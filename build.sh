PREFIX_PATH="$HOME/git/fmtlib/fmt/build/install"
PREFIX_PATH="$HOME/git/mgradysaunders/Microcosm/build/install;$PREFIX_PATH"
cmake -S. -Bbuild -GNinja \
  -DCMAKE_PREFIX_PATH="$PREFIX_PATH" \
  -DCMAKE_INSTALL_PREFIX=build/install \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CXX_COMPILER=g++-11
