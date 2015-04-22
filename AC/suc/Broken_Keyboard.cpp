#include <iostream>
#include <algorithm>

#include "string.h"

using namespace std;

void report_wornout(char ch, bool* is_wornout, char *output, int &output_len)
{
  if(ch>= 'a' && ch <= 'z') {
    ch += 'A'-'a';
  }

  int index = 50;
  if(ch>='A' && ch<='Z') {
    index = ch - 'A';
  } else if(ch>='0' && ch<='9') {
    index = ch -'0'+36;
  } else if(ch == '_') {
    index = 46;
  }
  if(!is_wornout[index]) {
    is_wornout[index] = true;
    output[output_len] = ch;
    ++output_len;
    output[output_len] = '\0';
  }
}

int main(int argc, char *argv[])
{

  bool is_wornout[60];
  fill(is_wornout, is_wornout+46, false);
  is_wornout[50] = true;
  char ori[100], types[100];
  char output[100];
  int output_len = 0;

  scanf("%s%s", ori, types);

  int ori_len = strlen(ori);
  for(int ind_ori=0, ind_types=0; ind_ori<ori_len;) {
    if(ori[ind_ori] == types[ind_types]) { ++ind_types;}
    else {
      report_wornout(ori[ind_ori], is_wornout, output, output_len);
    }
    ++ind_ori;
  }

  printf("%s\n", output);
  return 0;
}
