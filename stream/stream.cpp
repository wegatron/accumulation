#include <iostream>
#include <fstream>
#include <string.h>
#include "../assert/assure.h"

using namespace std;



/**
 * istream& seekg (streamoff off, ios_base::seekdir way)
 * seekdir should be: ios::begin, ios::end
 * if the file is opened for read and write seekg and seekp is the same
 */
void seekg_example(void)
{
  ifstream zsw_ifs("seek_example.txt");
  zsw_ifs.seekg(0, ios::end); // seek get 
  int n = zsw_ifs.tellg();
  cout << "size of file is:" << n << endl;

  ofstream zsw_ofs("seek_example.txt");
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
  ifstream zsw_ifs("filename.txt", ifstream::in);

  // istream& get (char& c);
  // istream& get (char* s, streamsize n);
  // istream& get (char* s, streamsize n, char delim);

  // istream& get (streambuf& sb);
  // istream& get (streambuf& sb, char delim);
  
  // istream& getline (char* s, streamsize n );
  // istream& getline (char* s, streamsize n, char delim );

  // istream& getline (istream& is, string& str, char delim);	
  // istream& getline (istream& is, string& str);

  // zsw_ifs.read()

}


class student{
 public:
  void show_info()
  {
    cout << "name:" << name << "\n"
         << "num:" << num << "\n"
         << "age:" << age << "\n"
         << "sex:" << sex << endl;
  }
  char name[31];
  char sex;
  int num;
  int age;

};

// struct student{
//   char name[31];
//   char sex;
//   int num;
//   int age;
// };

void binary_example(void)
{
  ofstream zsw_ofs("student", ofstream::binary|ofstream::trunc);
  assure_stream(zsw_ofs,"student");
  student stud[3]={"Li",'f',1001,18,"Fun",'m',1002,19,"Wang",'f',1004,17};
  zsw_ofs.write((char*)&stud[0], sizeof(stud)*sizeof(stud[0]));
  zsw_ofs.close();

  ifstream zsw_ifs("student", ifstream::binary);
  assure_stream(zsw_ifs, "student");
  student stud_read[3];
  zsw_ifs.read((char*)&stud_read[0], 3*sizeof(stud_read[0]));

  for (int i=0; i<3; ++i)
  {
    stud_read[i].show_info();
    // cout << "name:" << stud_read[i].name << "\n"
    //      << "num:" << stud_read[i].num << "\n"
    //      << "age:" << stud_read[i].age << "\n"
    //      << "sex:" << stud_read[i].sex << endl;
  }
}


int main(int argc, char *argv[])
{
  // input_output_example();
  binary_example();
  return 0;
}
