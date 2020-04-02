# Generated from orogen/lib/orogen/templates/tasks/CMakeLists.txt
find_package( Boost COMPONENTS filesystem)

find_package(lib_manager)
lib_defaults()
setup_qt()

include(marsTaskLib)
ADD_LIBRARY(${MARS_TASKLIB_NAME} SHARED 
    ${MARS_TASKLIB_SOURCES})
add_dependencies(${MARS_TASKLIB_NAME}
    regen-typekit)

if (${USE_QT5})
qt5_use_modules(${MARS_TASKLIB_NAME} Widgets)
endif (${USE_QT5})

pkg_check_modules(PKGCONFIG
        lib_manager
        mars_interfaces
        configmaps
        main_gui
        mars_app
        mars_graphics
        mars_sim
        scene_loader
)
include_directories(${PKGCONFIG_INCLUDE_DIRS})
link_directories(${PKGCONFIG_LIBRARY_DIRS})
add_definitions(${PKGCONFIG_CLFAGS_OTHER})  #flags excluding the ones with -I


TARGET_LINK_LIBRARIES(${MARS_TASKLIB_NAME}
    ${OrocosRTT_LIBRARIES}
    ${Boost_FILESYSTEM_LIBRARY}
    ${MARS_TASKLIB_DEPENDENT_LIBRARIES}
    ${QT_LIBRARIES}
    ${PKGCONFIG_LIBRARIES}
)

SET_TARGET_PROPERTIES(${MARS_TASKLIB_NAME}
    PROPERTIES LINK_INTERFACE_LIBRARIES "${MARS_TASKLIB_INTERFACE_LIBRARIES}")

INSTALL(TARGETS ${MARS_TASKLIB_NAME}
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib/orocos)

INSTALL(FILES ${MARS_TASKLIB_HEADERS}
    DESTINATION include/orocos/mars)

