#include <boost/filesystem.hpp>
#include <iostream>

int main(int argc, char *argv[])
{
  boost::filesystem::path p("/media/wegatron/data/workspace/accumulation/test/tmp.txt");
  if(boost::filesystem::exists(p))
    std::cout << "exist" << std::endl;
  return 0;
}
