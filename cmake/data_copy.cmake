add_custom_target(
  mapra_data ALL
  COMMAND ${CMAKE_COMMAND} -E copy_directory ${mapra_SOURCE_DIR}/data/daten
          ${mapra_BINARY_DIR}/daten
)

add_custom_target(
  mapra_font ALL
  COMMAND ${CMAKE_COMMAND} -E copy_directory ${mapra_SOURCE_DIR}/data/font
          ${mapra_BINARY_DIR}/font
)
