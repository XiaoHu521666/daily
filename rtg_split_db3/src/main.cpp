#include <exception>
#include <iostream>
#include <utility>

#include "rtg_split_db3/db3_splitter.hpp"

int main(int argc, char ** argv)
{
  try {
    auto options = rtg_split_db3::parse_options(argc, argv);
    rtg_split_db3::Db3Splitter splitter(std::move(options));
    splitter.run();
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "错误: " << error.what() << std::endl;
    return 1;
  }
}
