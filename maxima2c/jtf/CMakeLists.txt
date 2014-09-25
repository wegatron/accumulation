aux_source_directory(. source)
file(GLOB head *.h)
enable_language (Fortran)

set(SRC_DIR "{${CMAKE_CURRENT_SOURCE_DIR},$ENV{HOME}/.maxima}")

message("hello: " ${SRC_DIR})

add_custom_command(OUTPUT test.f
   COMMAND "maxima" ARGS --userdir=${SRC_DIR} -b test.mac
   COMMAND "pwd"
   COMMAND "sed" ARGS -r "'s/ sqrt\\(([0-9]+)\\)/ sqrt\(\\1\\.0\)/g'" -i test.f
   COMMAND "sed" ARGS -r "'s/\\[/\(/g'" -i test.f
   COMMAND "sed" ARGS -r "'s/\\]/\)/g'" -i test.f
   DEPENDS test.mac)


add_executable(test main.cpp test.f)


