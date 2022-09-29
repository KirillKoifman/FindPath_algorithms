#ifndef LAB1V2_STRUCTS_H
#define LAB1V2_STRUCTS_H
#include <vector>
#include <iostream>
#include <list>
#include <cmath>
#define OBSTACLE 1
#define TRAVERSABLE 0
#define PATH 2
#define START 3
#define GOAL 4

struct Node
{
    int i;
    int j;
    double g;
    double h;
    double f;
    const Node* parent;
    Node(int _i=-1, int _j=-1, double _g=0, double _h=0, Node* _parent=nullptr):i(_i), j(_j), g(_g), h(_h), f(_h+_g), parent(_parent){}
    
    bool operator ==(const Node& other) const
    {
        return this->i == other.i && this->j == other.j;
    }
    
    bool operator <(const Node& other) const
    {
        return this->i < other.i || (this->i == other.i && this->j < other.j);
    }
};

struct Result
{
    std::list<Node> path; //путь 
    double cost; //стоимость пути
    double runtime; //время работы алгоритма
    int steps; //число итераций работы алгоритма
    int nodes_created; //число созданных вершин
};

struct Map
{
    int width;
    int height;
    std::vector<std::vector<int>> grid;

    Map(int _width=0, int _height=0):width(_width), height(_height)
    {
        grid = std::vector<std::vector<int>>(height, std::vector<int>(width, TRAVERSABLE));//?
    }

    void add_obstacle(int i, int j)
    {
        grid[i][j] = OBSTACLE;
    }

    bool cell_on_grid(int i, int j)
    {
        return (i >= 0 && j >=0 && i < height && j < width);
    }

    bool cell_is_obstacle(int i, int j)
    {
        return grid[i][j] == OBSTACLE;
    }

    std::vector<Node> get_neighbors(Node s, int connections=4)
    {
        std::vector<std::pair<int, int>> deltas_var1 = { {0,1}, {1,0}, {-1,0}, {0,-1} };

        std::vector<std::pair<int, int>> deltas_var2 = { {0,1}, {1,0}, {-1,0}, {0,-1}, {1,1}, {-1,1}, {-1,-1}, {1,-1} };

        std::vector<Node> neighbors;

        if (connections == 4)
            for (auto d : deltas_var1)
            {
                Node n(s.i + d.first, s.j + d.second);
                if (cell_on_grid(n.i, n.j) && !cell_is_obstacle(n.i, n.j))
                    neighbors.push_back(n);
            }
        else if (connections == 8)
            for (auto d : deltas_var2)
            {
                Node n(s.i + d.first, s.j + d.second);
                if (cell_on_grid(n.i, n.j) && !cell_is_obstacle(n.i, n.j) && d.first != 0 && d.second != 0)
                {
                    if (d.first == 1 && d.second == 1)
                    {
                        Node n_copy1(s.i, s.j + d.second);
                        Node n_copy2(s.i + d.first, s.j);
                        if (cell_on_grid(n_copy1.i, n_copy1.j) && !cell_is_obstacle(n_copy1.i, n_copy1.j) && cell_on_grid(n_copy2.i, n_copy2.j) && !cell_is_obstacle(n_copy2.i, n_copy2.j))
                            neighbors.push_back(n);
                        else continue;
                    }
                    if (d.first == 1 && d.second == -1)
                    {
                        Node n_copy1(s.i + d.first, s.j);
                        Node n_copy2(s.i, s.j + d.second);
                        if (cell_on_grid(n_copy1.i, n_copy1.j) && !cell_is_obstacle(n_copy1.i, n_copy1.j) && cell_on_grid(n_copy2.i, n_copy2.j) && !cell_is_obstacle(n_copy2.i, n_copy2.j))
                            neighbors.push_back(n);
                        else continue;
                    }
                    if (d.first == -1 && d.second == -1)
                    {
                        Node n_copy1(s.i + d.first, s.j);
                        Node n_copy2(s.i, s.j + d.second);
                        if (cell_on_grid(n_copy1.i, n_copy1.j) && !cell_is_obstacle(n_copy1.i, n_copy1.j) && cell_on_grid(n_copy2.i, n_copy2.j) && !cell_is_obstacle(n_copy2.i, n_copy2.j))
                            neighbors.push_back(n);
                        else continue;
                    }
                    if (d.first == -1 && d.second == 1)
                    {
                        Node n_copy1(s.i, s.j + d.second);
                        Node n_copy2(s.i + d.first, s.j);
                        if (cell_on_grid(n_copy1.i, n_copy1.j) && !cell_is_obstacle(n_copy1.i, n_copy1.j) && cell_on_grid(n_copy2.i, n_copy2.j) && !cell_is_obstacle(n_copy2.i, n_copy2.j))
                            neighbors.push_back(n);
                        else continue;
                    }
                }
                else if (cell_on_grid(n.i, n.j) && !cell_is_obstacle(n.i, n.j))
                    neighbors.push_back(n);
            }
        return neighbors;
    }

    void print(std::list<Node> path)
    {
        for (auto n : path)
        {
            if (path.front().i == n.i && path.front().j == n.j) grid[n.i][n.j] = START;
            else if (path.back().i == n.i && path.back().j == n.j) grid[n.i][n.j] = GOAL;
            else grid[n.i][n.j] = PATH;
        }

        for (int i = 0; i < height; i++) 
        {
            for (int j = 0; j < width; j++)
                if (grid[i][j] == OBSTACLE)
                    std::cout << "# ";
                else if (grid[i][j] == TRAVERSABLE)
                    std::cout << ". ";
                else if (grid[i][j] == START)
                    std::cout << "S ";
                else if (grid[i][j] == GOAL)
                    std::cout << "G ";
                else
                    std::cout << "@ ";

            std::cout << std::endl;
        }
        for (auto n : path)
            grid[n.i][n.j] = TRAVERSABLE;
    }
};
#endif //LAsB1V2_STRUCTS_H
