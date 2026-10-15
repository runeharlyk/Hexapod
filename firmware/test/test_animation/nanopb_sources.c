// The native env compiles only the test directory, so nanopb and the generated animation schema
// are pulled in here as C rather than listed as sources.
#include "pb_common.c"
#include "pb_decode.c"
#include "pb_encode.c"
#include "animation.pb.c"
