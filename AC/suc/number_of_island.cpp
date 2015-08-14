#include <iostream>
#include <algorithm>
#include <string.h>
#include <vector>
#include <queue>

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
            bfs(grid, i, j, visited);
            ++cnt;
          }
        }
      }
      return cnt;
    }
private:
  void bfs(const vector<vector<char>>& grid, size_t pos_r, size_t pos_c, vector<vector<bool>>& visited)
  {
    queue<pair<size_t, size_t>> qu;
    visited[pos_r][pos_c] = true;
    qu.push(pair<size_t, size_t>(pos_r, pos_c));
    const size_t row = visited.size();
    const size_t col = visited[0].size();
    while(!qu.empty()) {
      pair<size_t, size_t> cur_pos = qu.front(); qu.pop();
      // up
      if(cur_pos.first > 0)  {
        pair<size_t, size_t> tmp_pos(cur_pos.first-1, cur_pos.second);
        if(grid[tmp_pos.first][tmp_pos.second]=='1' && !visited[tmp_pos.first][tmp_pos.second]) {
          visited[tmp_pos.first][tmp_pos.second] = true;
          qu.push(tmp_pos);
        }
      }
      // down
      if(cur_pos.first < row-1)  {
        pair<size_t, size_t> tmp_pos(cur_pos.first+1, cur_pos.second);
        if(grid[tmp_pos.first][tmp_pos.second]=='1' && !visited[tmp_pos.first][tmp_pos.second]) {
          visited[tmp_pos.first][tmp_pos.second] = true;
          qu.push(tmp_pos);
        }
      }
      // left
      if(cur_pos.second > 0)  {
        pair<size_t, size_t> tmp_pos(cur_pos.first, cur_pos.second-1);
        if(grid[tmp_pos.first][tmp_pos.second]=='1' && !visited[tmp_pos.first][tmp_pos.second]) {
          visited[tmp_pos.first][tmp_pos.second] = true;
          qu.push(tmp_pos);
        }
      }
      // right
      if(cur_pos.second < col-1)  {
        pair<size_t, size_t> tmp_pos(cur_pos.first, cur_pos.second+1);
        if(grid[tmp_pos.first][tmp_pos.second]=='1' && !visited[tmp_pos.first][tmp_pos.second]) {
          visited[tmp_pos.first][tmp_pos.second] = true;
          qu.push(tmp_pos);
        }
      }
    }
  }
};


void testCase0()
{
  vector<vector<char> > grid;

  {
    vector<char> g(5, '0');
    char gg[] = "11110";
    std::copy(gg, gg+5, g.begin());
    grid.push_back(g);
  }

  {
    vector<char> g(5, '0');
    char gg[] = "11010";
    std::copy(gg, gg+5, g.begin());
    grid.push_back(g);
  }

  {
    vector<char> g(5, '0');
    char gg[] = "11000";
    std::copy(gg, gg+5, g.begin());
    grid.push_back(g);
  }

  {
    vector<char> g(5, '0');
    grid.push_back(g);
  }

  Solution slu;
  slu.numIslands(grid);
  int res = slu.numIslands(grid);
  if( res == 1) { std::cout << "[INFO]" << __FUNCTION__ << "passed!" << std::endl; }
  else {
    std::cerr << "[ERROR] " << __FUNCTION__ << " line " << __LINE__ << std::endl;
    std::cerr << "error result is:" << res << std::endl;
  }
}

void testCase1()
{
  vector<vector<char> > grid;

  {
    vector<char> g(5, '0');
    char gg[] = "11000";
    std::copy(gg, gg+5, g.begin());
    grid.push_back(g);
  }

  {
    vector<char> g(5, '0');
    char gg[] = "11000";
    std::copy(gg, gg+5, g.begin());
    grid.push_back(g);
  }

  {
    vector<char> g(5, '0');
    char gg[] = "00100";
    std::copy(gg, gg+5, g.begin());
    grid.push_back(g);
  }

  {
    vector<char> g(5, '0');
    char gg[] = "00011";
    std::copy(gg, gg+5, g.begin());
    grid.push_back(g);
  }

  Solution slu;
  slu.numIslands(grid);
  int res = slu.numIslands(grid);
  if( res == 3) { std::cout << "[INFO]" << __FUNCTION__ << "passed!" << std::endl; }
  else {
    std::cerr << "[ERROR] " << __FUNCTION__ << " line " << __LINE__ << std::endl;
    std::cerr << "error result is:" << res << std::endl;
  }
}

int main(int argc, char *argv[])
{
  testCase0();
  testCase1();
  return 0;
}
