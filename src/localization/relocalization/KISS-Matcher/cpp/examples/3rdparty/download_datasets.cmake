include(FetchContent)

set(DATA_DIR "${CMAKE_BINARY_DIR}/data")
set(ZIP_DIR "${CMAKE_BINARY_DIR}/zip_files")

set(DROPBOX_FILES
    "https://www.dropbox.com/scl/fi/1lsz6bxwan0sytj87ea9h/Vel16.zip?rlkey=g4jpze2j6iu6hk9ahq0m6t7o3&st=vrfpk4nv&dl=1"
    "https://www.dropbox.com/scl/fi/weqfi572gbi5z654d56ai/Vel64.zip?rlkey=l9upmgfjx7nhkbgl9du7igrfu&st=4q9gyous&dl=1"
    "https://www.dropbox.com/scl/fi/lnsbaqmbgz0qi1r8ocesd/HeLiPR-KAIST05.zip?rlkey=50jyhl180qpmf1j5jn9ru37ru&st=q0kay7o3&dl=1"
    "https://www.dropbox.com/scl/fi/c6c1jxrld17ywj7x2ok1q/VBR-Collosseo.zip?rlkey=b1sk0xvnntqy8kyw1apob37ob&st=5imrvvwo&dl=1"
    "https://www.dropbox.com/scl/fi/73l59cj5ypfvjrkrc23qx/KITTI00-to-KITTI360.zip?rlkey=pqukxxgpxaq1pugo6dhzq8xa4&st=yns7uolj&dl=1"
    "https://www.dropbox.com/scl/fi/lb49afp7x5ja3bfptbo68/KITTI00-to-07.zip?rlkey=qkwq99vwwlnxnxj3nhdptrusr&st=nzx8ts9j&dl=1"
    "https://www.dropbox.com/scl/fi/n6sypvt4rdssn172mn2jv/bun_zipper.ply?rlkey=hk2danjxt29a7ahq374s8m7ak&st=o5udnqjv&dl=1"
)

set(DOWNLOADED_FILES
    "${ZIP_DIR}/Vel16.zip"
    "${ZIP_DIR}/Vel64.zip"
    "${ZIP_DIR}/HeLiPR-KAIST05.zip"
    "${ZIP_DIR}/VBR-Collosseo.zip"
    "${ZIP_DIR}/KITTI00-to-KITTI360.zip"
    "${ZIP_DIR}/KITTI00-to-07.zip"
    "${ZIP_DIR}/bun_zipper.ply"
)

file(MAKE_DIRECTORY ${DATA_DIR})
file(MAKE_DIRECTORY ${ZIP_DIR})

list(LENGTH DROPBOX_FILES NUM_FILES)
math(EXPR LAST_INDEX "${NUM_FILES} - 1")

foreach(INDEX RANGE ${LAST_INDEX})
    list(GET DROPBOX_FILES ${INDEX} URL)
    list(GET DOWNLOADED_FILES ${INDEX} DEST_FILE)

    message(STATUS "Downloading ${URL} -> ${DEST_FILE}")
    file(DOWNLOAD ${URL} ${DEST_FILE} SHOW_PROGRESS)

    if (${DEST_FILE} MATCHES "\\.zip$")
        message(STATUS "Extracting ${DEST_FILE} to ${DATA_DIR}")
        execute_process(
            COMMAND ${CMAKE_COMMAND} -E tar xzf ${DEST_FILE}
            WORKING_DIRECTORY ${DATA_DIR}
        )
    else()
        file(COPY ${DEST_FILE} DESTINATION ${DATA_DIR})
    endif()
endforeach()

message(STATUS "All datasets downloaded and extracted to ${DATA_DIR}")
