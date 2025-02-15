
target_sources(${PROJECT_NAME}
        PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/LasPlugin.h
        ${CMAKE_CURRENT_LIST_DIR}/LasIOFilter.h
        ${CMAKE_CURRENT_LIST_DIR}/LasDetails.h
        ${CMAKE_CURRENT_LIST_DIR}/LasOpenDialog.h
        ${CMAKE_CURRENT_LIST_DIR}/LasSaveDialog.h
        ${CMAKE_CURRENT_LIST_DIR}/LasScalarField.h
        ${CMAKE_CURRENT_LIST_DIR}/LasMetadata.h
        ${CMAKE_CURRENT_LIST_DIR}/LasExtraScalarField.h
        ${CMAKE_CURRENT_LIST_DIR}/LasExtraScalarFieldCard.h
        ${CMAKE_CURRENT_LIST_DIR}/LasScalarFieldLoader.h
        ${CMAKE_CURRENT_LIST_DIR}/LasScalarFieldSaver.h
        ${CMAKE_CURRENT_LIST_DIR}/LasWaveformLoader.h
        ${CMAKE_CURRENT_LIST_DIR}/LasTiler.h
        ${CMAKE_CURRENT_LIST_DIR}/LasVlr.h
        ${CMAKE_CURRENT_LIST_DIR}/LasSaver.h
        ${CMAKE_CURRENT_LIST_DIR}/LasWaveformSaver.h
        ${CMAKE_CURRENT_LIST_DIR}/CopcVlrs.h
        ${CMAKE_CURRENT_LIST_DIR}/CopcLoader.h
        )

target_include_directories(${PROJECT_NAME}
        PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        )
