# Compile the Icon Composer document, including a fallback icon for older macOS.
find_program(APEX_XCRUN xcrun REQUIRED)

set(apex_icon_source "${PROJECT_SOURCE_DIR}/assets/icons/Apex.icon")
set(apex_icon_output "${CMAKE_CURRENT_BINARY_DIR}/apex-icon")
set(apex_icon_resources
    "${apex_icon_output}/Assets.car"
    "${apex_icon_output}/Apex.icns"
)

set(apex_icon_minimum_macos "13.0")
if (CMAKE_OSX_DEPLOYMENT_TARGET)
    set(apex_icon_minimum_macos "${CMAKE_OSX_DEPLOYMENT_TARGET}")
endif ()

add_custom_command(
    OUTPUT ${apex_icon_resources}
    COMMAND "${CMAKE_COMMAND}" -E make_directory "${apex_icon_output}"
    COMMAND "${APEX_XCRUN}" actool "${apex_icon_source}"
        --compile "${apex_icon_output}"
        --platform macosx
        --minimum-deployment-target "${apex_icon_minimum_macos}"
        --app-icon Apex
        --output-partial-info-plist "${apex_icon_output}/partial-info.plist"
    DEPENDS
        "${apex_icon_source}/icon.json"
        "${apex_icon_source}/Assets/apex.png"
    COMMENT "Compiling Apex's native macOS icon"
    VERBATIM
)

target_sources(apex PRIVATE ${apex_icon_resources})
set_source_files_properties(${apex_icon_resources} PROPERTIES
    MACOSX_PACKAGE_LOCATION Resources
)

set_target_properties(apex PROPERTIES
    MACOSX_BUNDLE TRUE
    OUTPUT_NAME "Apex"
    MACOSX_BUNDLE_INFO_PLIST "${PROJECT_SOURCE_DIR}/cmake/Info.plist.in"
    MACOSX_BUNDLE_BUNDLE_NAME "Apex"
    MACOSX_BUNDLE_GUI_IDENTIFIER "com.steeve.apex"
    MACOSX_BUNDLE_ICON_FILE "Apex.icns"
    MACOSX_BUNDLE_BUNDLE_VERSION "${PROJECT_VERSION}"
    MACOSX_BUNDLE_SHORT_VERSION_STRING "${PROJECT_VERSION}"
)
