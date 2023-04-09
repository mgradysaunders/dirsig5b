add_executable(
  example
  PerspectiveCamera.cc
  LumberyardBistro.cc
  main.cc
  )
target_link_libraries(example PRIVATE dirsig5b Microcosm::stbi)
