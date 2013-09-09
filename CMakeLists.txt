cmake_minimum_required(VERSION 2.8.3)
project(essential-ros)

set(TEXTS
  title
  simple-test
  foo-other
)

set(EDITIONS
  hydro
)

set(PROCESSED_TEXTS)
file(MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/outputs)

foreach(EDITION ${EDITIONS})
  file(MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/${EDITION})

  foreach(TEXT ${TEXTS})
    set(OUTPUT "${EDITION}/${TEXT}.md") 
    add_custom_command(
      OUTPUT ${OUTPUT}
      DEPENDS text/${TEXT}.em.md
      COMMAND empy -D"rosdistro='hydro'" ${PROJECT_SOURCE_DIR}/text/${TEXT}.em.md > ${OUTPUT}
      )
    list(APPEND PROCESSED_TEXTS ${OUTPUT})
  endforeach()

  set(OUTPUT "outputs/${PROJECT_NAME}-${EDITION}.html") 
  add_custom_command(
    OUTPUT ${OUTPUT}
    DEPENDS ${PROCESSED_TEXTS}
    COMMAND pandoc ${PROCESSED_TEXTS} -o ${OUTPUT} --self-contained
    )
  add_custom_target(${EDITION}-html ALL DEPENDS ${OUTPUT})

  set(OUTPUT "outputs/${PROJECT_NAME}-${EDITION}.pdf") 
  add_custom_command(
    OUTPUT ${OUTPUT}
    DEPENDS ${PROCESSED_TEXTS}
    COMMAND pandoc ${PROCESSED_TEXTS} -o ${OUTPUT}
    )
  add_custom_target(${EDITION}-pdf ALL DEPENDS ${OUTPUT})
endforeach()
# TODO: target for multipage html-site target
# TODO: target for epub
