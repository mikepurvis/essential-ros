cmake_minimum_required(VERSION 2.8.3)
project(book)

set(TEXTS
  title
  simple-test
  foo-other
)

set(PROCESSED_TEXTS)

foreach(TEXT ${TEXTS})
  #  add_custom_target(${TEXT}.md
  #  DEPENDS text/${TEXT}.em.md
  #  COMMAND empy -D"rosdistro='hydro'" ${PROJECT_SOURCE_DIR}/text/${TEXT}.em.md > ${TEXT}.md
  #  )
  add_custom_command(
    OUTPUT ${TEXT}.md
    DEPENDS text/${TEXT}.em.md
    COMMAND empy -D"rosdistro='hydro'" ${PROJECT_SOURCE_DIR}/text/${TEXT}.em.md > ${TEXT}.md
    )
  list(APPEND PROCESSED_TEXTS ${TEXT}.md)
endforeach()

add_custom_command(
  OUTPUT ${PROJECT_NAME}.html
  DEPENDS ${PROCESSED_TEXTS}
  COMMAND pandoc ${PROCESSED_TEXTS} -o ${PROJECT_NAME}.html --self-contained
  )

add_custom_command(
  OUTPUT ${PROJECT_NAME}.pdf
  DEPENDS ${PROCESSED_TEXTS}
  COMMAND pandoc ${PROCESSED_TEXTS} -o ${PROJECT_NAME}.pdf
  )

add_custom_target(html ALL DEPENDS ${PROJECT_NAME}.html)
add_custom_target(pdf ALL DEPENDS ${PROJECT_NAME}.pdf)
