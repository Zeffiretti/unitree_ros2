add_library(eigen INTERFACE)
target_include_directories(eigen INTERFACE
  ${UNITREE_INCLUDE_DIRS}
)

# Avoids using LGPL code.
target_compile_definitions(eigen INTERFACE EIGEN_MPL2_ONLY)

# Compatibility library for targets that require relative include path of Eigen, i.e.
# #include "Eigen/Dense" or #include <Eigen/Dense>.
add_library(eigen_compat INTERFACE)
target_include_directories(eigen_compat INTERFACE
  ${UNITREE_SIM_ROOT_DIR}/third_party/eigen
)
target_compile_definitions(eigen_compat INTERFACE EIGEN_MPL2_ONLY)
