/**  surfer2 -- a straight skeleton implementation
 *
 *  Copyright 2015 -- 2019 Peter Palfraader
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
#include "gtest/gtest.h"

#include <filesystem>
#include <unistd.h>

/* defined in main.cpp when SURF_TEST_SUITE is defined */
int run_surfer(int argc, char *argv[]);

std::vector<std::string> input_files;

class SurferRunTest :
    public ::testing::TestWithParam<std::string> {};


static int
call_surfer(std::vector<std::string> args) {
  std::vector<const char*> cmdv;

  std::transform(args.begin(), args.end(), std::back_inserter(cmdv),
    [](std::string &s) { return s.c_str(); });
  char **cmd = const_cast<char**>(cmdv.data());

  std::cout << "Running ";
  for (auto a : cmdv) {
    std::cout << a << " ";
  };
  std::cout << std::endl;


  int ret = run_surfer(cmdv.size(), &cmd[0]);
  exit(ret);
}

TEST_P(SurferRunTest, Run) {
  std::vector<std::string> args;

  args.push_back( /* <dummy> */ "surfer");
  args.push_back(GetParam());
  args.push_back("/dev/null");

  EXPECT_EXIT(call_surfer(args), ::testing::ExitedWithCode(0), "All done.");
}

INSTANTIATE_TEST_CASE_P(SurferRunTests,
                        SurferRunTest,
                        ::testing::ValuesIn(input_files),
                        );

static bool
has_suffix(const std::string &path, const std::string &suffix) {
  if (suffix.size() > path.size()) return false;
  return path.compare(path.size() - suffix.size(), suffix.size(), suffix) == 0;
}

int main(int argc, char **argv) {

  for(auto& p : std::filesystem::recursive_directory_iterator(CMAKE_SOURCE_DIR "/test-data")) {
    if (has_suffix(p.path(), ".graphml")) {
      input_files.push_back(p.path());
    }
  }
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
