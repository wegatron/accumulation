#include <iostream>
#include "../data_cache.hpp"

int main(int argc, char* argv[])
{
	data_cache<int> dc(10, 20);
	for(int i=0; i<20; ++i)
	{
		dc[i].reset(new int(i));
	}
	for(int i=19; i>=0; --i)
	{
		std::cout << "dc[i] = " << *dc[i] << std::endl;
		std::cout << "::::";
		std::cout.flush();
		dc.print_lru();
		std::cout << std::endl;
		std::cout.flush();
	}
	return 0;
}
