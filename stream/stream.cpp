#include <iostream>
#include <fstream>

using namespace std;

/**
 * istream& seekg (streamoff off, ios_base::seekdir way)
 * seekdir should be: ios::begin, ios::end 
 */
void seekg_example(void)
{
  ifstream zsw_ifs("seek_example.txt");
  zsw_ifs.seekg(0, ios::end);
  int n = zsw_ifs.tellg();
  cout << "size of file is:" << n << endl;
}

/** 
 * get from ifstream
 * 
 */
void get_input_example(void)
{
  /// ifstream (const char* filename, ios_base::openmode mode = ios_base::in);
  /// ios_base::openmode should be:
  /**
   * binary(binary) Consider stream as binary rather than text.
   * in(input) Allow input operations on the stream.
   * out(output) Allow output operations on the stream.
   * trunc(truncate) Any current content is discarded, assuming a length of zero on opening.
   */
  ifstream zsw_ifs("", ios_base::openmode mode = ios_base::in);
}
int main(int argc, char *argv[])
{
  return 0;
}
