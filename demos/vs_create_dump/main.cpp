#include <iostream>
#include "CreateDump.h"

int main(int argc, char* argv[])
{
	CreateDump::configure(".", "tdump");
	int i = 100;
	for (; i > 0; --i)
	{
		std::cout << "i = " << i  << std::endl;
	}
	std::cout << "i = " << i << std::endl;
	int tv = 100 / i;
	std::cout << "error" << std::endl;
	return 0;
}
