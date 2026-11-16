# Replays a copy of the stored failing episode whose manifest has its variant label swapped (B0 -> B1).
# The replay must report the difference with a non-zero exit, on any toolchain: B1 takes a different
# decision at the junction, so the outcome and the route decisions change, not only the trace hash.
#   cmake -DROAD_DRIVE=<road_drive> -DEPISODE=<episode dir> -DWORK=<scratch dir> -P replay_detects_change.cmake
foreach(var ROAD_DRIVE EPISODE WORK)
  if(NOT DEFINED ${var})
    message(FATAL_ERROR "${var} is not set")
  endif()
endforeach()

file(REMOVE_RECURSE "${WORK}")
file(MAKE_DIRECTORY "${WORK}")
file(COPY "${EPISODE}/" DESTINATION "${WORK}")
file(READ "${WORK}/manifest.txt" manifest)
string(REGEX REPLACE "\nvariant=B0(\r?\n)" "\nvariant=B1\\1" changed "${manifest}")
if(changed STREQUAL manifest)
  message(FATAL_ERROR "the stored manifest has no variant=B0 line to change")
endif()
file(WRITE "${WORK}/manifest.txt" "${changed}")

execute_process(COMMAND "${ROAD_DRIVE}" replay "${WORK}" RESULT_VARIABLE rc OUTPUT_VARIABLE out ERROR_VARIABLE err)
message("${out}${err}")
if(rc EQUAL 0)
  message(FATAL_ERROR "replay accepted an episode whose recorded variant was changed")
endif()
message("replay rejected the changed episode (exit ${rc}), as required")
