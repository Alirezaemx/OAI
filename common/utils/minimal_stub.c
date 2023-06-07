#include <stdlib.h>
#ifndef T_TRACER
int T_stdout;
#endif
int oai_exit = 0;

void exit_function(const char *file, const char *function, const int line, const char *s, const int assert)
{
  if (assert) {
    abort();
  } else {
    exit(EXIT_SUCCESS);
  }
}
