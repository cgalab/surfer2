/**  surfer2 -- a straight skeleton implementation
 *
 *  Copyright 2018, 2019 Peter Palfrader
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
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
