add_subdirectory(camera)
add_subdirectory(correspondence)
add_subdirectory(multiview)
add_subdirectory(numeric)
add_subdirectory(reconstruction)

if ( Ceres_FOUND )
    add_subdirectory(simple_pipeline)
endif ()