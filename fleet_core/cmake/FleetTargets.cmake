# Compiler settings for every fleet_core target. Applied per target, so third-party code never
# sees -Werror.
function(fleet_configure_target target)
  set_target_properties(${target} PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(${target} PRIVATE
      -Wall -Wextra -Wpedantic -Wshadow -Wnon-virtual-dtor -Woverloaded-virtual
      -Wold-style-cast -Wcast-align -Wformat=2 -Wimplicit-fallthrough
      # Determinism: no fused multiply-add contraction, so x86 and ARM (Jetson) round alike.
      -ffp-contract=off)
    if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
      target_compile_options(${target} PRIVATE -Wduplicated-cond -Wlogical-op)
    endif()
    if(FLEET_WERROR)
      target_compile_options(${target} PRIVATE -Werror)
    endif()
    if(FLEET_SANITIZE)
      target_compile_options(${target} PRIVATE
        -fsanitize=address,undefined -fno-sanitize-recover=all -fno-omit-frame-pointer)
      target_link_options(${target} PRIVATE -fsanitize=address,undefined)
    endif()
  endif()
endfunction()

# Compiles every public header on its own, so a header that relies on an earlier include
# fails the build instead of the next person who includes it.
function(fleet_add_header_check library include_dir)
  file(GLOB_RECURSE headers CONFIGURE_DEPENDS "${include_dir}/*.hpp")
  set(sources "")
  foreach(header ${headers})
    file(RELATIVE_PATH rel "${include_dir}" "${header}")
    string(MAKE_C_IDENTIFIER "${rel}" id)
    set(tu "${CMAKE_CURRENT_BINARY_DIR}/header_check/${id}.cpp")
    file(WRITE "${tu}.in" "#include \"${rel}\"\n")
    configure_file("${tu}.in" "${tu}" COPYONLY)
    list(APPEND sources "${tu}")
  endforeach()
  add_library(${library}_header_check OBJECT ${sources})
  target_link_libraries(${library}_header_check PRIVATE ${library})
  fleet_configure_target(${library}_header_check)
endfunction()
