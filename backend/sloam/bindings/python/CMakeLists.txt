find_package(pybind11 REQUIRED)

set(PYPKG_DIR "${CMAKE_CURRENT_BINARY_DIR}/slideslampy")

pybind11_add_module(py_slideslam py_slideslam.cpp)
target_link_libraries(py_slideslam PRIVATE sloam_core semantic_clipper triangulation)
# Same -march as semantic_clipper.cpp, so Eigen matrices passed to it get its (AVX, 32-byte) alignment
target_compile_options(py_slideslam PRIVATE -march=native)
set_target_properties(py_slideslam PROPERTIES OUTPUT_NAME "slideslampy")
set_target_properties(py_slideslam PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${PYPKG_DIR}")

# copy setup.py file binary dir for install with: pip install .
configure_file(setup.py.in ${CMAKE_CURRENT_BINARY_DIR}/setup.py)

# Create the Python package
file(WRITE "${PYPKG_DIR}/__init__.py" "from .slideslampy import *\n")

add_custom_target(pip-install
    DEPENDS py_slideslam
    COMMAND ${PYTHON_EXECUTABLE} -m pip install .
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
)
