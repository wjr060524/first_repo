#include <iostream>
#include <vector>
#include <queue>
#include <limits>
#include <cmath>
using namespace std;

// 赛道一：智能车路径规划与避障建模
// 以二维平面网格为环境，0为可通行，1为障碍物，A*算法实现最短路径规划

struct Node {
    int x, y;
    int g, h;
    Node* parent;
    Node(int x, int y, int g, int h, Node* parent = nullptr)
        : x(x), y(y), g(g), h(h), parent(parent) {}
    int f() const { return g + h; }
};

struct cmp {
    bool operator()(const Node* a, const Node* b) const {
        return a->f() > b->f();
    }
};

int heuristic(int x1, int y1, int x2, int y2) {
    // 曼哈顿距离
    return abs(x1 - x2) + abs(y1 - y2);
}

vector<pair<int, int>> directions = {{0,1},{1,0},{0,-1},{-1,0}};

bool inBounds(int x, int y, int n, int m) {
    return x >= 0 && x < n && y >= 0 && y < m;
}

vector<pair<int, int>> reconstructPath(Node* endNode) {
    vector<pair<int, int>> path;
    Node* curr = endNode;
    while (curr) {
        path.push_back({curr->x, curr->y});
        curr = curr->parent;
    }
    reverse(path.begin(), path.end());
    return path;
}

vector<pair<int, int>> aStar(const vector<vector<int>>& grid, pair<int,int> start, pair<int,int> goal) {
    int n = grid.size(), m = grid[0].size();
    vector<vector<bool>> visited(n, vector<bool>(m, false));
    priority_queue<Node*, vector<Node*>, cmp> pq;
    pq.push(new Node(start.first, start.second, 0, heuristic(start.first, start.second, goal.first, goal.second)));
    while (!pq.empty()) {
        Node* curr = pq.top(); pq.pop();
        if (visited[curr->x][curr->y]) {
            delete curr;
            continue;
        }
        visited[curr->x][curr->y] = true;
        if (curr->x == goal.first && curr->y == goal.second) {
            auto path = reconstructPath(curr);
            // 释放内存
            delete curr;
            while (!pq.empty()) { delete pq.top(); pq.pop(); }
            return path;
        }
        for (auto& d : directions) {
            int nx = curr->x + d.first, ny = curr->y + d.second;
            if (inBounds(nx, ny, n, m) && !visited[nx][ny] && grid[nx][ny] == 0) {
                pq.push(new Node(nx, ny, curr->g + 1, heuristic(nx, ny, goal.first, goal.second), curr));
            }
        }
        // 注意：这里只释放已访问节点，未访问节点在最终释放
    }
    // 无法到达
    return {};
}

int main() {
    int n, m;
    cout << "请输入地图行数和列数:" << endl;
    cin >> n >> m;
    vector<vector<int>> grid(n, vector<int>(m));
    cout << "请输入地图(0为可通行，1为障碍):" << endl;
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < m; ++j)
            cin >> grid[i][j];
    int sx, sy, gx, gy;
    cout << "请输入起点坐标(x y):" << endl;
    cin >> sx >> sy;
    cout << "请输入终点坐标(x y):" << endl;
    cin >> gx >> gy;
    auto path = aStar(grid, {sx, sy}, {gx, gy});
    if (path.empty()) {
        cout << "无法到达终点" << endl;
    } else {
        cout << "最短路径为:" << endl;
        for (auto& p : path) {
            cout << "(" << p.first << "," << p.second << ") ";
        }
        cout << endl << "路径长度: " << path.size() - 1 << endl;
    }
    return 0;
}

