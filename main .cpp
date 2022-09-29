#include <iostream>
#include <list>
#include <set>
#include "structs.h"
#include "utils.h"
#include <chrono>
#include <cmath>

class BFS //�������� "������ � ������"
{
public:

    //������ ������� ������� ���� �� start �� goal �� ����� grid
    Result find_path(Node start, Node goal, Map grid, int connections ,double hweight)
    {
        //������ ���������� time_now ��� �������� �������, �� ������� ���������� ����� ������������ ���� � ��� ������ ����������
        auto time_now = std::chrono::high_resolution_clock::now();
        
        /* ���������� result ����� ������� ������ � ����, ������� �����
        ���������� ��������( ��������, "����� ����������", "����� ����" � �.�. */
        Result result;
        
        int steps = 0;
        
        //��������� ��� ��������� �������
        start.g = 0;

        //� ������ OPEN ����� �������� ���������� �������( � ������� �� ������������)(����� ��������� � �� start)
        std::list<Node> OPEN;
        OPEN.push_back(start);

        //� ������ CLOSED ����� �������� ������������� �������( ��� ����������)(� ���� ���� ��������� start)
        std::set<Node> CLOSED;
        CLOSED.insert(start);

        //���������� pathfound ����� �������������� ��� ����������� ������ ���� � ��� ������ ��� ���������
        bool pathfound = false;

        //���� ������ OPEN �� ����(�.�. ��� ������� ����� �����������) � �� pathfound=false(�.�. ���� ����� ������)
        while(!OPEN.empty() && !pathfound)
        {
            /* current ����� ������� ������ � ������� �������, �.�.��������������� � ������ ������,
            �����( ������� �� ���� �� ������ ������ "OPEN")*/
            Node current = OPEN.front();

            //�������� ��������� �������������� ������� ������� �� OPEN(�.�. �������� � ��� "��� �������������")
            OPEN.pop_front();

            steps++;

            //������� ��������(�.�. ��������� ��� �����������) ������� 
            auto neighbors = grid.get_neighbors(current, connections);

            //������������� ��� �������, � ������� ����� ������������� �� current
            for(auto n:neighbors)
            {
                //���� ��������������� ������� n ����������� � ������ CLOSED(�.�. ��� �� ���� �����������), �� �����..
                if (CLOSED.find(n) == CLOSED.end())
                {
                    //..������������ � ��������� ��� �������� �� ��������������� ������� n
                    n.g = current.g + count_Gvalue(current, n);

                    /*��������� ��� ������ ������� � �������� "��������" ����� current(�.�.����������,
                    ��� ������� � ������� (n) ��� ��������� �� ������� (current) ),*/
                    n.parent = &(*CLOSED.find(current));

                    //��������� ������ ������� � OPEN, ����� ����� ������������� � ��� current ��� ��������� ��������
                    OPEN.push_back(n);

                    //����� ��������� ������ ������� � CLOSED, �������������� ���������, ��� ��� ��� �����������
                    CLOSED.insert(n);

                    //���� ��������������� ������� �������� �������, �����..
                    if(n == goal)
                    {
                        //�������������� �������� ���� �� start � goal � ��������� ��� � path
                        result.path = reconstruct_path(n);

                        //��������� ��������� ����
                        result.cost = n.g;

                        //�.�. ���� ������
                        pathfound = true;
                        break;
                    }
                }
            }
        }
        //��������� ������, ��������� � ���� ������ ���������
        result.steps = steps;
        result.nodes_created = CLOSED.size();
        result.runtime = (std::chrono::high_resolution_clock::now() - time_now).count()/1e+9;
        return result;
    }

    //�������, ������� ��������� ���������� ����� ������� � ��������������� ���������
    double count_Gvalue(Node current, Node neighbor)
    {
        if ((std::abs(neighbor.j - current.j) == 1) && (std::abs(neighbor.i - current.i) == 1)) return std::sqrt(2);
        else return 1;
    }

    //�������, ������� ������������� � ��������� ���� �� start �� goal
    std::list<Node> reconstruct_path(Node n)
    {
        std::list<Node> path;
        while(n.parent != nullptr)
        {
            path.push_front(n);
            n = *n.parent;
        }
        path.push_front(n);
        return path;
    }
};

class AStar//��������� "A*" � "��������"
{
public:
    //������ ������� ������� ���� �� start �� goal �� ����� grid
    Result find_path(Node start, Node goal, Map grid, std::string metrictype = "Octile", int connections = 8, double hweight = 1)
    {
        //������ ���������� time_now ��� �������� �������, �� ������� ���������� ����� ������������ ���� � ��� ������ ����������
        auto time_now = std::chrono::high_resolution_clock::now();

        /* ���������� result ����� ������� ������ � ����, ������� �����
        ���������� ��������( ��������, "����� ����������", "����� ����" � �.�. */
        Result result;

        int steps = 0;

        //��������� ��� � �������� ��������� ��� ��������� �������
        start.g = 0;
        start.h = count_Hvalue(start, goal, metrictype, hweight);

        //� ������ OPEN ����� �������� ���������� �������( � ������� �� ������������)(����� ��������� � �� start)
        std::list<Node> OPEN;
        OPEN.push_back(start);

        //� ������ CLOSED ����� �������� ������������� �������( ��� ����������)(� ���� ���� ��������� start)
        std::set<Node> CLOSED;
        CLOSED.insert(start);

        //���������� pathfound ����� �������������� ��� ����������� ������ ���� � ��� ������ ��� ���������
        bool pathfound = false;

        //���� ������ OPEN �� ����(�.�. ��� ������� ����� �����������) � �� pathfound=false(�.�. ���� ����� ������)
        while (!OPEN.empty() && !pathfound)
        {
            /*current - ���������� ����� ������� ������ � ������� �������(�.�.��������������� � ������ ������,
            �����) �����, ������� �������� ���������� F-��������, �� ������ OPEN(������� ����� ���������� � ����� ����)*/
            Node current = OPEN.front();
            for (std::list<Node>::iterator iter = OPEN.begin(); iter != OPEN.end(); ++iter)
                if ((*iter).f < current.f) current = (*iter);

            //�������� ��������� �������������� ������� ������� �� OPEN(�.�. �������� � ��� "��� �������������")
            OPEN.remove(current);

            //� ��������� � � CLOSED, ��� ��� "�������������"
            CLOSED.insert(current);

            steps++;

            //���� ������� ������� �������� �������, �����..
            if (current == goal)
            {
                //�������������� �������� ���� �� start � goal � ��������� ��� � path
                result.path = reconstruct_path(current);

                //��������� ��������� ����
                result.cost = current.f;

                //�.�. ���� ������
                pathfound = true;
                break;
            }

            //������� ��������(�.�. ��������� ��� �����������) ������� 
            auto neighbors = grid.get_neighbors(current, connections);

            //������������� ��� �������, � ������� ����� ������������� �� current
            for (auto n : neighbors)
            {               
                //���� ��������������� ������� n ����������� � ������ CLOSED, �� �����..
                if (CLOSED.find(n) == CLOSED.end())
                {
                    //..������������ � ��������� ��� �������� �� ��������������� ������� n
                    n.g = current.g + count_Gvalue(current, n);

                    //..������������ � ��������� �������� ��������� ��� ���������������(�������� � �������) �������
                    n.h = count_Hvalue(n, goal, metrictype, hweight);

                    //��������� F-��������
                    n.f = n.g + n.h;

                    //std::cout << "F-value: " << n.f << std::endl;

                    /*��������� ��� ������ ������� � �������� "��������" ����� current(�.�.����������,
                    ��� ������� � ������� (n) ��� ��������� �� ������� (current) ),*/
                    n.parent = &(*CLOSED.find(current));

                    //��������� ������ ������� � OPEN, ����� ����� ������������� � ��� current ��� ��������� ��������
                    OPEN.push_back(n);

                    //����� ��������� ������ ������� � CLOSED, �������������� ���������, ��� ��� ��� �����������
                    CLOSED.insert(n);
                }

                //���� ��������������� ������� ���� � ������ OPEN(�.�. ��� ���� ����������� �����), �� �����..
                for (std::list<Node>::iterator iter = OPEN.begin(); iter != OPEN.end(); ++iter)
                    if (n == *iter)
                    {
                        //..������������ � ��������� ��� �������� �� ��������������� ������� n
                        n.g = current.g + count_Gvalue(current, n);

                        //..������������ � ��������� �������� ��������� ��� ���������������(�������� � �������) �������
                        n.h = count_Hvalue(n, goal, metrictype, hweight);

                        //��������� F-��������
                        n.f = n.g + n.h;

                        /*���� ������ ��������� ����(�.�. ����� g-��������,
                        ������ ��� ����������� ��������� �� start �� n, �
                        h-��������, �������� ���������, �������������� ��������� ���� �� n �� goal) ��������� ��� ����� n 
                        ������, ��� ���������� � OPEN, �� ����� ������� � ��-�� � ������� � ������ OPEN*/
                        if (n.f < (*iter).f) (*iter).f = n.f;
                    }
                
            }
        }
        //��������� ������, ��������� � ���� ������ ���������
        result.steps = steps;
        result.nodes_created = CLOSED.size();
        result.runtime = (std::chrono::high_resolution_clock::now() - time_now).count() / 1e+9;
        return result;
    }

    //�������, ������� ��������� ���������� ����� ������� � ��������������� ���������
    double count_Gvalue(Node current, Node neighbor)
    {
        if ((std::abs(neighbor.j - current.j) == 1) && (std::abs(neighbor.i - current.i) == 1)) return std::sqrt(2);
        else return 1;            
    }

    //�������, ������� ��������� �������� ���������, ������ �� ���������� ���� ������� ����������, ���������� � xml-�����
    double count_Hvalue(Node current, Node goal, std::string metrictype, int hweight)
    {
        if (metrictype == "Manhattan") 
            return hweight * (abs(goal.i - current.i) + abs(goal.j - current.j));
    
        else if (metrictype == "Octile") 
            return hweight * (abs(abs(goal.i - current.i) - abs(goal.j - current.j)) + sqrt(2 * hweight) * std::min(abs(goal.i - current.i), abs(goal.j - current.j)));
        
        else if (metrictype == "Euclidean")
            return hweight * sqrt((goal.i - current.i) * (goal.i - current.i) + (goal.j - current.j) * (goal.j - current.j));
    }

    //�������, ������� ������������� � ��������� ���� �� start �� goal
    std::list<Node> reconstruct_path(Node n)
    {
        //TODO - reconstruct path using back pointers
        std::list<Node> path;
        while (n.parent != nullptr)
        {
            path.push_front(n);
            n = *n.parent;
        }
        path.push_front(n);
        return path;
    }
};

int main(int argc, char* argv[])
{
    for(int i=0; i<argc; i++)
        std::cout<<argv[i]<<"\n";

    if(argc<2)
    {
        std::cout << "Name of the input XML file is not specified."<<std::endl;
        return 1;
    }

    Loader loader;
    loader.load_instance(argv[1]);

    Result result;

    if(loader.algorithm == "BFS")
    {
        BFS bfs;
        result = bfs.find_path(loader.start, loader.goal, loader.grid, loader.connections, loader.hweight);
    }
    else
    {
        if(loader.algorithm == "Dijkstra")
            loader.hweight = 0;
        AStar astar;
        result = astar.find_path(loader.start, loader.goal, loader.grid, loader.metrictype, loader.connections, loader.hweight);
    }

    loader.grid.print(result.path);

    std::cout<< "\n\tINPUT INFO\n\t----------\nAlgorithm: " << loader.algorithm<<"\nMetrictype: "<<loader.metrictype << 
        "\nNumber of cell's connections: "<<loader.connections<< "\nHweight value(w2-coefficient value): "<<loader.hweight << 
        "\nSTART and GOAL vertexes' co-ordinates(example: x(3,7)):\nSTART("<<loader.start.j<<','<<loader.start.i<<") ----> GOAL("<< loader.goal.j << ',' << loader.goal.i<<')'<<
        "\n\n\tOUTPUT INFO\n\t----------- \nCost: " << result.cost << "\nRuntime : " << result.runtime <<"\nSteps : "<<result.steps<<"\nNodes created : "<<result.nodes_created<<std::endl;
}