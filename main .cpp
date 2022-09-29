#include <iostream>
#include <list>
#include <set>
#include "structs.h"
#include "utils.h"
#include <chrono>
#include <cmath>

class BFS //алгоритм "Поиска в ширину"
{
public:

    //Данная функция находит путь от start до goal на карте grid
    Result find_path(Node start, Node goal, Map grid, int connections ,double hweight)
    {
        //Создаём переменную time_now для подсчёта времени, за которое происходит поиск оптимального пути и его полное построение
        auto time_now = std::chrono::high_resolution_clock::now();
        
        /* Переменная result будет хранить данные о пути, который будет
        постепенно строится( например, "время построения", "длина пути" и т.п. */
        Result result;
        
        int steps = 0;
        
        //определим вес стартовой вершины
        start.g = 0;

        //В списке OPEN будут хранится затронутые вершины( в очереди на рассмотрение)(сразу добавляем в неё start)
        std::list<Node> OPEN;
        OPEN.push_back(start);

        //В списке CLOSED будут хранится просмотренные вершины( уже посещённые)(в него тоже добавляем start)
        std::set<Node> CLOSED;
        CLOSED.insert(start);

        //Переменная pathfound будет использоваться для прекращения поиска пути и для начала его постройки
        bool pathfound = false;

        //Пока список OPEN не пуст(т.е. все вершины будут рассмотрены) и НЕ pathfound=false(т.е. путь будет найден)
        while(!OPEN.empty() && !pathfound)
        {
            /* current будет хранить данные о ТЕКУЩЕЙ вершине, т.е.рассматриваемой в данный момент,
            точке( которую мы берём из начала списка "OPEN")*/
            Node current = OPEN.front();

            //согласно алгоритму предварительно удаляем вершину из OPEN(т.о. отмечаем её как "уже рассмотренную")
            OPEN.pop_front();

            steps++;

            //находим соседние(т.е. доступные для перемещения) вершины 
            auto neighbors = grid.get_neighbors(current, connections);

            //Просматриваем все вершины, в которые можно переместиться из current
            for(auto n:neighbors)
            {
                //если рассматриваемая вершина n отсутствует в списке CLOSED(т.е. ещё не была рассмотрена), то тогда..
                if (CLOSED.find(n) == CLOSED.end())
                {
                    //..рассчитываем и сохраняем вес перехода из рассматриваемой вершины n
                    n.g = current.g + count_Gvalue(current, n);

                    /*сохраняем для данной вершины в качестве "родителя" точку current(т.е.обозначаем,
                    что переход в вершину (n) был произведён из вершины (current) ),*/
                    n.parent = &(*CLOSED.find(current));

                    //дабавляем данную вершину в OPEN, чтобы далее рассматривать её как current при следующей итерации
                    OPEN.push_back(n);

                    //также дабавляем данную вершину в CLOSED, предварительно обозначая, что она УЖЕ рассмотрена
                    CLOSED.insert(n);

                    //если рассматриваемая вершина является целевой, тогда..
                    if(n == goal)
                    {
                        //перестравиваем конечный путь из start в goal и сохраняем его в path
                        result.path = reconstruct_path(n);

                        //сохраняем стоимость пути
                        result.cost = n.g;

                        //т.о. путь найден
                        pathfound = true;
                        break;
                    }
                }
            }
        }
        //сохраняем данные, собранные в ходе работы алгоритма
        result.steps = steps;
        result.nodes_created = CLOSED.size();
        result.runtime = (std::chrono::high_resolution_clock::now() - time_now).count()/1e+9;
        return result;
    }

    //функция, которая вычисляет расстояние между текущей и рассматрвиаемой вершинами
    double count_Gvalue(Node current, Node neighbor)
    {
        if ((std::abs(neighbor.j - current.j) == 1) && (std::abs(neighbor.i - current.i) == 1)) return std::sqrt(2);
        else return 1;
    }

    //функция, которая перестраивает и сохраняет путь от start до goal
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

class AStar//алгоритмы "A*" и "Дейкстры"
{
public:
    //Данная функция находит путь от start до goal на карте grid
    Result find_path(Node start, Node goal, Map grid, std::string metrictype = "Octile", int connections = 8, double hweight = 1)
    {
        //Создаём переменную time_now для подсчёта времени, за которое происходит поиск оптимального пути и его полное построение
        auto time_now = std::chrono::high_resolution_clock::now();

        /* Переменная result будет хранить данные о пути, который будет
        постепенно строится( например, "время построения", "длина пути" и т.п. */
        Result result;

        int steps = 0;

        //определим вес и значение эвристики для стартовой вершины
        start.g = 0;
        start.h = count_Hvalue(start, goal, metrictype, hweight);

        //В списке OPEN будут хранится затронутые вершины( в очереди на рассмотрение)(сразу добавляем в неё start)
        std::list<Node> OPEN;
        OPEN.push_back(start);

        //В списке CLOSED будут хранится просмотренные вершины( уже посещённые)(в него тоже добавляем start)
        std::set<Node> CLOSED;
        CLOSED.insert(start);

        //Переменная pathfound будет использоваться для прекращения поиска пути и для начала его постройки
        bool pathfound = false;

        //Пока список OPEN не пуст(т.е. все вершины будут рассмотрены) и НЕ pathfound=false(т.е. путь будет найден)
        while (!OPEN.empty() && !pathfound)
        {
            /*current - переменная будет хранить данные о ТЕКУЩЕЙ вершине(т.е.рассматриваемой в данный момент,
            точке) такой, которая содержит НАИМЕНЬШЕЕ F-значение, из списка OPEN(которое будет определено в цикле ниже)*/
            Node current = OPEN.front();
            for (std::list<Node>::iterator iter = OPEN.begin(); iter != OPEN.end(); ++iter)
                if ((*iter).f < current.f) current = (*iter);

            //согласно алгоритму предварительно удаляем вершину из OPEN(т.о. отмечаем её как "уже рассмотренную")
            OPEN.remove(current);

            //и добавляем её в CLOSED, как уже "рассмотренную"
            CLOSED.insert(current);

            steps++;

            //если текущая вершина является целевой, тогда..
            if (current == goal)
            {
                //перестравиваем конечный путь из start в goal и сохраняем его в path
                result.path = reconstruct_path(current);

                //сохраняем стоимость пути
                result.cost = current.f;

                //т.о. путь найден
                pathfound = true;
                break;
            }

            //находим соседние(т.е. доступные для перемещения) вершины 
            auto neighbors = grid.get_neighbors(current, connections);

            //Просматриваем все вершины, в которые можно переместиться из current
            for (auto n : neighbors)
            {               
                //если рассматриваемая вершина n отсутствует в списке CLOSED, то тогда..
                if (CLOSED.find(n) == CLOSED.end())
                {
                    //..рассчитываем и сохраняем вес перехода из рассматриваемой вершины n
                    n.g = current.g + count_Gvalue(current, n);

                    //..рассчитываем и сохраняем значение эвристики для рассматриваемой(соседней к текущей) вершины
                    n.h = count_Hvalue(n, goal, metrictype, hweight);

                    //вычисляем F-значение
                    n.f = n.g + n.h;

                    //std::cout << "F-value: " << n.f << std::endl;

                    /*сохраняем для данной вершины в качестве "родителя" точку current(т.е.обозначаем,
                    что переход в вершину (n) был произведён из вершины (current) ),*/
                    n.parent = &(*CLOSED.find(current));

                    //дабавляем данную вершину в OPEN, чтобы далее рассматривать её как current при следующей итерации
                    OPEN.push_back(n);

                    //также дабавляем данную вершину в CLOSED, предварительно обозначая, что она УЖЕ рассмотрена
                    CLOSED.insert(n);
                }

                //если рассматриваемая вершина есть в списке OPEN(т.е. уже была рассмотрена ранее), то тогда..
                for (std::list<Node>::iterator iter = OPEN.begin(); iter != OPEN.end(); ++iter)
                    if (n == *iter)
                    {
                        //..рассчитываем и сохраняем вес перехода из рассматриваемой вершины n
                        n.g = current.g + count_Gvalue(current, n);

                        //..рассчитываем и сохраняем значение эвристики для рассматриваемой(соседней к текущей) вершины
                        n.h = count_Hvalue(n, goal, metrictype, hweight);

                        //вычисляем F-значение
                        n.f = n.g + n.h;

                        /*если полная стоимость пути(т.е. сумма g-значения,
                        точной уже накопленной стоимости от start до n, и
                        h-значения, значения эвристики, предполагаемой стоимости пути от n до goal) найденная для новой n 
                        меньше, чем хранящаяся в OPEN, то тогда передаём её св-ва в вершину в списке OPEN*/
                        if (n.f < (*iter).f) (*iter).f = n.f;
                    }
                
            }
        }
        //сохраняем данные, собранные в ходе работы алгоритма
        result.steps = steps;
        result.nodes_created = CLOSED.size();
        result.runtime = (std::chrono::high_resolution_clock::now() - time_now).count() / 1e+9;
        return result;
    }

    //функция, которая вычисляет расстояние между текущей и рассматрвиаемой вершинами
    double count_Gvalue(Node current, Node neighbor)
    {
        if ((std::abs(neighbor.j - current.j) == 1) && (std::abs(neighbor.i - current.i) == 1)) return std::sqrt(2);
        else return 1;            
    }

    //функция, которая вычисляет значение эвристики, исходя из выбранного вида расчёта расстояния, указанного в xml-файле
    double count_Hvalue(Node current, Node goal, std::string metrictype, int hweight)
    {
        if (metrictype == "Manhattan") 
            return hweight * (abs(goal.i - current.i) + abs(goal.j - current.j));
    
        else if (metrictype == "Octile") 
            return hweight * (abs(abs(goal.i - current.i) - abs(goal.j - current.j)) + sqrt(2 * hweight) * std::min(abs(goal.i - current.i), abs(goal.j - current.j)));
        
        else if (metrictype == "Euclidean")
            return hweight * sqrt((goal.i - current.i) * (goal.i - current.i) + (goal.j - current.j) * (goal.j - current.j));
    }

    //функция, которая перестраивает и сохраняет путь от start до goal
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