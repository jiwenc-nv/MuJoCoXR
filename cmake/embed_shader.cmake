# Turns a GLSL file into a C++ raw string literal for #include.
#
# A raw literal rather than escaped lines, so the generated file stays
# readable and a shader compile error's line numbers still match the source.
# The delimiter is chosen to be something GLSL cannot contain.
#
# There is deliberately NO newline after the opening delimiter: GLSL requires
# #version on the very first line, and a leading blank line makes every
# shader in the tree fail to compile with a message that blames #version.
#
# Usage: cmake -DIN=<glsl> -DOUT=<header> -P embed_shader.cmake

file(READ ${IN} MXR_SHADER_SRC)
file(WRITE ${OUT} "// Generated from ${IN} — do not edit.\nR\"MXRGLSL("
                  "${MXR_SHADER_SRC}"
                  ")MXRGLSL\"\n")
