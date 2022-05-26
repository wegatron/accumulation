#include "visit_struct.hpp"

#include <string>
#include <iostream>

struct my_type {
  int a;
  float b;
  std::string c;
};

VISITABLE_STRUCT(my_type, a, b, c);



struct debug_printer {
  template <typename T>
  void operator()(const char * name, const T & value) {
    std::cerr << name << ": " << value << std::endl;
  }
};

void debug_print(const my_type & my_struct) {
  visit_struct::for_each(my_struct, debug_printer{});
}

int main(int argc, char const *argv[])
{
    my_type mt{1, 1.2f, "zsw"};
    debug_print(mt);
    return 0;
}
