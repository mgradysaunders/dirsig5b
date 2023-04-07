add_executable(
  example
  Sensor.cc
  World.cc
  main.cc
  )
target_link_libraries(example PRIVATE dirsig5b Microcosm::stbi)
