#include <iostream>
#include <list>
#include <set>
#include "structs.h"
#include "utils.h"
#include <chrono>
#include <iterator>
#include <cmath>

class BFS //breadth-first-search (поиск в ширину)
{
public:
    //Данная функция находит путь от start до goal на карте grid
    Result find_path(Node start, Node goal, Map grid, int connections)
    {
        //Создаём переменную time_now для подсчёта времени, за которое происходит поиск оптимального пути и его полное построение
        auto time_now = std::chrono::high_resolution_clock::now();

        /* Переменная result будет хранить данные о пути, который будет
        постепенно строится( например, "время построения", "длина пути" и т.п. */
        Result result;

        int steps = 0;
        start.g = 0;

        //В списке OPEN будут хранится затронутые вершины( в очереди на рассмотрение)
        std::list<Node> OPEN;
        OPEN.push_back(start);

        //В списке CLOSED будут хранится просмотренные вершины( уже посещённые)
        std::set<Node> CLOSED;
        CLOSED.insert(start);

        bool pathfound = false;

        //Путь не будет найден и построен, пока список "OPEN" НЕ пуст
        while (!OPEN.empty() && !pathfound)
        {
            /* current будет хранить данные о ТЕКУЩЕЙ, т.е.рассматриваемой в данный момент,
            точке( которую мы берём из начала списка "OPEN")*/
            Node current = OPEN.front();     

            //и, соответсвенно, удаляем эту вершину из "OPEN"
            OPEN.pop_front();

            steps++;

            // теперь рассмотрим для точки current на карте grid потенциальные вершины, в которые мы можем переместиться
            auto neighbors = grid.get_neighbors(current, connections);

            //Просматриваем все вершины, в которые можно переместиться из current
            for (auto n : neighbors) 
            {
                //Если рассматриваемая вершина из вектора neighbors приходится соседней к вершине current, тогда.. 
                if (CLOSED.find(n) == CLOSED.end())
                {
                    //..сохраняем для данной вершины новый вес перехода из неё,
                    n.g = current.g + 1;

                    /*сохраняем для данной вершины в качестве "родителя" точку current(т.е.обозначаем,
                    что переход в вершину (n) был произведён из вершины (current) ),*/
                    n.parent = &(*CLOSED.find(current));

                    //добавляем данную вершину в конец списка OPEN( данная вершина будет рассмотрена следующей) 
                    OPEN.push_back(n);

                    /*добавляем данную вершину в конец списка CLOSED
                    (чтобы при следующей итерации мы рассматривали соседние для НЕЁ вершины)*/
                    CLOSED.insert(n);

                    //если координаты рассматриваемой вершины совпадают координатами конечной,тогда..
                    if (n == goal) 
                    {
                        //переопределяем(перестраиваем) конечный путь
                        result.path = reconstruct_path(n);

                        //и пересчитываем итоговую стоимость пути( т.е. его длину)
                        result.cost = n.g;

                        //соответственно указываем, что путь найден и завершаем выполнение цикла
                        pathfound = true;
                        break;
                    }
                }
            }
        }

        //возвращаем в result основные данные по построенному пути
        result.steps = steps;
        result.nodes_created = CLOSED.size();
        result.runtime = (std::chrono::high_resolution_clock::now() - time_now).count() / 1e+9;
        return result;
    }

    //данная функция перестраивает путь, исходя из последней рассмотренной вершины
    std::list<Node> reconstruct_path(Node n)
    {
        //объявляем переменную-список, которая будет хранить путь
        std::list<Node> path;

        //последовательно сохраняем в списке path вершины( вплоть до n), образующие нынешний путь
        while (n.parent != nullptr)
        {
            path.push_front(n);
            n = *n.parent;
        }
        path.push_front(n);
        return path;
    }
};

class AStar
{
public:
    Result find_path(Node start, Node goal, Map grid, std::string metrictype = "Octile", int connections = 8, double hweight = 1)
    {
        //TODO - implement the main cycle of AStar algorithm

        //Создаём переменную time_now для подсчёта времени, за которое происходит поиск оптимального пути и его полное построение
        auto time_now = std::chrono::high_resolution_clock::now();

        /* Переменная result будет хранить данные о пути, который будет
        постепенно строится( например, "время построения", "длина пути" и т.п. */
        Result result;

        int steps = 0;
        start.g = 0;

        //В списке OPEN будут хранится затронутые вершины( в очереди на рассмотрение)
        std::list<Node> OPEN;
        OPEN.push_back(start);

        //В списке CLOSED будут хранится просмотренные вершины( уже посещённые)
        std::set<Node> CLOSED;

        bool pathfound = false;

        //!OPEN.empty() && !pathfound Путь не будет найден и построен, пока список "OPEN" НЕ пуст
        while (!OPEN.empty() && !pathfound)
        {
            std::list<Node>::iterator myIter = OPEN.begin();
            for (std::list<Node>::iterator iter = OPEN.begin(); iter == OPEN.end(); ++iter)
                if ((*myIter).f > (*iter).f) myIter = iter;  

            /* current будет хранить данные о ТЕКУЩЕЙ, т.е.рассматриваемой в данный момент,
            точке( которую мы берём из начала списка "OPEN")*/
            Node current = (*myIter);           
            
            //и, соответсвенно, удаляем эту вершину из "OPEN"
            OPEN.erase(myIter);

            CLOSED.insert(current);

            steps++;
           
            // теперь рассмотрим для точки current на карте grid потенциальные вершины, в которые мы можем переместиться
            auto neighbors = grid.get_neighbors(current, connections);           

            //Просматриваем все вершины, в которые можно переместиться из current
            for (auto n : neighbors)
            {
                //n.init_w1w2(algorithm);

                n.h = count_HValue(goal, current, metrictype, hweight);

                n.f = current.g + n.h;

                if (!(CLOSED.count(n)))
                {   
                    n.g = current.g + hweight;
                    n.parent = &(*CLOSED.find(current));            
                    
                    //добавляем данную вершину в конец списка OPEN( данная вершина будет рассмотрена следующей) 
                    OPEN.push_back(n);

                    /*добавляем данную вершину в конец списка CLOSED
                    (чтобы при следующей итерации мы рассматривали соседние для НЕЁ вершины)*/
                    CLOSED.insert(n);
                }
                else if((n.g>current.g + hweight) && CLOSED.count(n))
                    if (metrictype == "AStar") n.g = current.g + hweight;
                    else n.g = current.g + hweight;
            }
            //если координаты рассматриваемой вершины совпадают координатами конечной,тогда..
            if (current == goal)
            {
                //переопределяем(перестраиваем) конечный путь
                result.path = reconstruct_path(current);

                //и пересчитываем итоговую стоимость пути( т.е. его длину)
                result.cost = result.path.size() - 1;

                //соответственно указываем, что путь найден и завершаем выполнение цикла
                pathfound = true;
                break;
            }              
        }
    //возвращаем в result основные данные по построенному пути
    result.steps = steps;
    result.nodes_created = CLOSED.size();
    result.runtime = (std::chrono::high_resolution_clock::now() - time_now).count() / 1e+9;
    return result;
    }
    
    double count_HValue(Node goal, Node current, std::string metrictype, double hweight)
    {
        if (metrictype == "Euclidean") return hweight * sqrt(pow((goal.i - current.i), 2) + pow((goal.j - current.j), 2));
        else if (metrictype == "Manhattan") return hweight * (abs(goal.i - current.i) + abs(goal.j - current.j));
        else if (metrictype == "Octile")return hweight * (abs(goal.i - current.i) + abs(goal.j - current.j)) + (hweight - 2 * hweight) * std::min(abs(goal.i - current.i), abs(goal.j - current.j));
    }

    std::list<Node> reconstruct_path(Node current)
    {
        //TODO - reconstruct path using back pointers
         
        std::list<Node> path{};

        while (current.parent != nullptr)
        {
            path.push_front(current);
            current = *current.parent;
        }
        path.push_front(current);

        return path;
    }
};

int main(int argc, char* argv[]) //argc - argumnet counter, argv - argument values
{
    for (int i = 0; i < argc; i++)
        std::cout << argv[i] << "\n";

    if (argc < 2)
    {
        std::cout << "Name of the input XML file is not specified." << std::endl;
        return 1;
    }

    Loader loader;
    loader.load_instance(argv[1]);

    Result result;

    if (loader.algorithm == "BFS")
    {
        BFS bfs;
        result = bfs.find_path(loader.start, loader.goal, loader.grid, loader.connections);
    }
    else
    {
        //hweight - эвристека
        if (loader.algorithm == "Dijkstra") loader.hweight = 0;

        AStar astar;
        result = astar.find_path(loader.start, loader.goal, loader.grid, loader.metrictype, loader.connections, loader.hweight);
    }

    loader.grid.print(result.path);

    std::cout << "Cost: " << result.cost << "\nRuntime: " << result.runtime
        << "\nSteps: " << result.steps << "\nNodes created: " << result.nodes_created << std::endl;
}