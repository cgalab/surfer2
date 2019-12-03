#include "tools.h"

#include <stddef.h>
#include <time.h>

static uint32_t rnd_seed = 123456789;
static int seeded = false;

void
my_srand(uint32_t s) {
  rnd_seed = s;
  seeded = true;
}

uint32_t
my_rand() {
  if (!seeded) {
    int seed = time(NULL);
    //LOG(DEBUG) << "seeding RNG with " << seed;
    my_srand(seed);
  };
  rnd_seed = (1103515245 * rnd_seed + 12345); // % m;
  return rnd_seed;
}
