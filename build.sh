#!/bin/bash -e

NINJA="/Users/ricardo/Library/Application*Support/JetBrains/Toolbox/apps/CLion/ch-0/231.9225.21/CLion.app/Contents/bin/ninja/mac/ninja"

(
  cd cmake-build-debug
  $NINJA clean
  $NINJA
  ls -la *.uf2
)
