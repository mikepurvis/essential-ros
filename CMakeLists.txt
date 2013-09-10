cmake_minimum_required(VERSION 2.8.3)
project(essential-ros)

set(TEXTS
  title
  introduction
  installation
  workspaces-packages
  cpp-package
  python-package
  msg-package
  source-control
  hack-released-package
)

set(EDITIONS
  hydro-precise
)

set(PROCESSED_TEXTS)
file(MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/outputs)

foreach(EDITION ${EDITIONS})
  file(MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/${EDITION})
  set(EMPY_INVOCATION empy -F${PROJECT_SOURCE_DIR}/editions/${EDITION}.py)
  execute_process(
    COMMAND ${EMPY_INVOCATION} ${PROJECT_SOURCE_DIR}/text/version-brief.em.txt
    OUTPUT_VARIABLE VERSION_BRIEF
    OUTPUT_STRIP_TRAILING_WHITESPACE)
  
  foreach(TEXT ${TEXTS})
    set(OUTPUT "${EDITION}/${TEXT}.md") 
    set(INPUT "${PROJECT_SOURCE_DIR}/text/${TEXT}.em.md") 
    add_custom_command(
      DEPENDS ${INPUT}
      OUTPUT ${OUTPUT}
      COMMAND ${EMPY_INVOCATION} ${INPUT} > ${OUTPUT}
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
  set(TEMPLATE "${PROJECT_SOURCE_DIR}/tex/template.tex") 
  add_custom_command(
    OUTPUT ${OUTPUT}
    DEPENDS ${PROCESSED_TEXTS} ${TEMPLATE}
    COMMAND pandoc ${PROCESSED_TEXTS} -o ${OUTPUT} --template=${TEMPLATE} --variable version="${VERSION_BRIEF}" --latex-engine=xelatex
    )
  add_custom_target(${EDITION}-pdf ALL DEPENDS ${OUTPUT})
endforeach()
# TODO: target for multipage html-site target
# TODO: target for epub
