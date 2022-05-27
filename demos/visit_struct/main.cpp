#include "visit_struct.hpp"

#include <string>
#include <iostream>

struct my_type {
  int a;
  float b;
  std::string c;
};

VISITABLE_STRUCT(my_type, a, b, c);

template<typename T>
void debug_print(const T & my_struct) {
  visit_struct::for_each(my_struct,
    [](const char * name, const auto & value) {
      std::cerr << name << ": " << value << std::endl;
    });
}

int main(int argc, char const *argv[])
{
    my_type mt{1, 1.2f, "zsw"};
    debug_print(mt);
    return 0;
}