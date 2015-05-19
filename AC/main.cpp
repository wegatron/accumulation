#include <iostream>
#include <algorithm>
#include <string.h>
#include <vector>

using namespace std;

class Solution {
public:
    int numIslands(vector<vector<char>>& grid) {
      const size_t row = grid.size();
      if(row == 0) return 0;
      const size_t col = grid[0].size();
      if(col == 0) return 0;
      vector<vector<bool>> visited(row, vector<bool>(col, false));
      size_t cnt = 0;
      for(size_t i=0; i<row; ++i) {
        for(size_t j=0; j<col; ++j) {
          if(grid[i][j]=='1' && !visited[i][j]) {
            bfs(grid, i, j, visited, cnt);
          }
        }
      }
      return 0;
    }
private:
  void bfs(const vector<vector<char>>& grid, size_t pos_r, size_t pos_j, vector<vector<bool>>& visited, size_t &cnt)
  {

  }
};

int main(int argc, char *argv[])
{
  return 0;
}
