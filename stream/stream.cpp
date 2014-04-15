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
  zsw_ifs.seekg(0, ios::end); // seek get
  int n = zsw_ifs.tellg();
  cout << "size of file is:" << n << endl;

  ofstream zsw_ifs("seek_example.txt");
  zsw_ofs.seekp(0, ios::end); // seek end
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
   * ate at end	The output position starts at the end of the file.
   * app(append) All output operations happen at the end of the file, appending to its existing contents.
   * trunc(truncate) Any contents that existed in the file before it is open are discarded.
   */
  ifstream zsw_ifs("filename.txt", ios_base::openmode mode = ios_base::in);
  // zsw_ifs.get();
  // zsw_ifs.getline();
  // zsw_ifs.read()
  // getline(zsw_ifs, )
}

void output_example(void)
{}

void input_output_example(void)
{
  /**
   * binary(binary) Consider stream as binary rather than text.
   * in(input) Allow input operations on the stream.
   * out(output) Allow output operations on the stream.
   * ate at end	The output position starts at the end of the file.
   * app(append) All output operations happen at the end of the file, appending to its existing contents.
   * trunc(truncate) Any contents that existed in the file before it is open are discarded.
   */
  fstream zsw_fs("filename.txt", std::fstream::in|std::fstream::out);
  
  zsw_fs << "zsw_output" << endl;
  string teststr;
  zsw_fs >> teststr;
  cout << "test str is:" << teststr << endl;
}

int main(int argc, char *argv[])
{
  return 0;
}
