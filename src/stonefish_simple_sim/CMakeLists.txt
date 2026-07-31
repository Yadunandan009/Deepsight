cmake_minimum_required(VERSION 3.8)
project(stonefish_simple_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install the launch directory
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install the config directory (So your .scn file gets moved too!)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Install the data directory (So your .obj files get moved too!)
install(DIRECTORY data
  DESTINATION share/${PROJECT_NAME}
)


install(PROGRAMS
  ardusub_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
