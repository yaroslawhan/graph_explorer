#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>
#include <iostream>
#include <vector>
#include <cmath>
#include <string>
#include <limits>
#include <queue>
#include <map>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <queue>
#include <stack>

const int WINDOW_WIDTH = 800;
const int WINDOW_HEIGHT = 600;
const float VERTEX_RADIUS = 15.0f; // Радиус вершины

struct Edge {
    int from, to, weight;
};

class Graph {
private:
    std::vector<std::vector<Edge>> adjacencyList;
    int vertexCount;
    std::vector<std::pair<float, float>> vertexPositions; // Координаты вершин

public:
    Graph(int n) : vertexCount(n) {
        adjacencyList.resize(n);
        vertexPositions.resize(n, { 0, 0 }); // Инициализируем нулевыми координатами
    }

    void addEdge(int from, int to, int weight) {
        adjacencyList[from].push_back({ from, to, weight });
        adjacencyList[to].push_back({ to, from, weight }); // Для неориентированного графа
    }

    const std::vector<Edge>& getEdges(int vertex) const {
        return adjacencyList[vertex];
    }

    int getVertexCount() const {
        return vertexCount;
    }

    void addVertex(float x = 0, float y = 0) {
        adjacencyList.push_back({});
        vertexPositions.push_back({ x, y });
        ++vertexCount;
    }

    void setVertexPosition(int vertex, float x, float y) {
        if (vertex >= 0 && vertex < vertexCount) {
            vertexPositions[vertex] = { x, y };
        }
    }

    const std::pair<float, float>& getVertexPosition(int vertex) const {
        return vertexPositions[vertex];
    }



    // BFS (поиск в ширину)
    std::vector<int> bfs(int start) {
        std::vector<bool> visited(vertexCount, false);
        std::vector<int> traversal;
        std::queue<int> q;

        q.push(start);
        visited[start] = true;

        while (!q.empty()) {
            int u = q.front();
            q.pop();
            traversal.push_back(u);

            for (const auto& edge : adjacencyList[u]) {
                if (!visited[edge.to]) {
                    visited[edge.to] = true;
                    q.push(edge.to);
                }
            }
        }

        return traversal;
    }

    // DFS (поиск в глубину)
    std::vector<int> dfs(int start) {
        std::vector<bool> visited(vertexCount, false);
        std::vector<int> traversal;
        std::stack<int> s;

        s.push(start);

        while (!s.empty()) {
            int u = s.top();
            s.pop();

            if (!visited[u]) {
                visited[u] = true;
                traversal.push_back(u);

                for (const auto& edge : adjacencyList[u]) {
                    if (!visited[edge.to]) {
                        s.push(edge.to);
                    }
                }
            }
        }

        return traversal;
    }

    // Алгоритм Дейкстры для поиска кратчайшего пути
    std::vector<std::pair<int, int>> dijkstra(int start, int end) {
        std::vector<int> dist(vertexCount, std::numeric_limits<int>::max());
        std::vector<int> prev(vertexCount, -1);
        dist[start] = 0;

        using P = std::pair<int, int>; // (dist, vertex)
        std::priority_queue<P, std::vector<P>, std::greater<P>> pq;
        pq.push({ 0, start });

        while (!pq.empty()) {
            int u = pq.top().second;
            int d = pq.top().first;
            pq.pop();

            if (d > dist[u]) continue;

            for (const auto& edge : adjacencyList[u]) {
                int v = edge.to;
                int weight = edge.weight;

                if (dist[u] + weight < dist[v]) {
                    dist[v] = dist[u] + weight;
                    prev[v] = u;
                    pq.push({ dist[v], v });
                }
            }
        }

        std::vector<std::pair<int, int>> shortestPathEdges;
        if (dist[end] == std::numeric_limits<int>::max()) {
            std::cout << "No path from " << start << " to " << end << std::endl;
            return shortestPathEdges;
        }

        std::cout << "Shortest path from " << start << " to " << end << " is of length " << dist[end] << std::endl;
        std::cout << "Path: ";

        std::vector<int> path;
        for (int v = end; v != -1; v = prev[v]) {
            if (prev[v] != -1) {
                shortestPathEdges.emplace_back(prev[v], v);
            }
            path.push_back(v);
        }

        std::reverse(path.begin(), path.end());

        for (int v : path) {
            std::cout << v << " ";
        }
        std::cout << std::endl;

        return shortestPathEdges;
    }

    // Экспорт графа с координатами
    void exportToFile(const std::string& filename) const {
        std::ofstream outFile(filename);
        if (!outFile.is_open()) {
            throw std::ios_base::failure("Failed to open file for writing.");
        }

        outFile << vertexCount << "\n";

        // Сохраняем координаты вершин
        for (const auto& pos : vertexPositions) {
            outFile << pos.first << " " << pos.second << "\n";
        }

        // Сохраняем рёбра
        for (int i = 0; i < vertexCount; ++i) {
            for (const auto& edge : adjacencyList[i]) {
                if (edge.from < edge.to) { // Чтобы избежать дублирования рёбер
                    outFile << edge.from << " " << edge.to << " " << edge.weight << "\n";
                }
            }
        }

        outFile.close();
    }

    // Импорт графа с координатами
    void importFromFile(const std::string& filename) {
        std::ifstream inFile(filename);
        if (!inFile.is_open()) {
            throw std::ios_base::failure("Failed to open file for reading.");
        }

        inFile >> vertexCount;
        adjacencyList.clear();
        adjacencyList.resize(vertexCount);
        vertexPositions.clear();
        vertexPositions.resize(vertexCount);

        // Загружаем координаты вершин
        for (int i = 0; i < vertexCount; ++i) {
            float x, y;
            inFile >> x >> y;
            vertexPositions[i] = { x, y };
        }

        // Загружаем рёбра
        int from, to, weight;
        while (inFile >> from >> to >> weight) {
            std::cout << "Importing edge: " << from << " -> " << to << " with weight " << weight << "\n";
            addEdge(from, to, weight);
        }

        inFile.close();
    }

    void removeVertex(int vertex) {
        if (vertex < 0 || vertex >= vertexCount) {
            std::cerr << "Vertex out of bounds: " << vertex << "\n";
            return;
        }

        // Удаляем только те рёбра, которые больше не соединены с другими вершинами
        for (int i = 0; i < vertexCount; ++i) {
            if (i == vertex) continue;

            adjacencyList[i].erase(
                std::remove_if(adjacencyList[i].begin(), adjacencyList[i].end(),
                    [vertex, this](const Edge& edge) {
                        // Удаляем только если инцидентная вершина - это удаляемая
                        return edge.to == vertex &&
                            std::none_of(adjacencyList[edge.to].begin(), adjacencyList[edge.to].end(),
                                [edge](const Edge& otherEdge) {
                                    return otherEdge.to != edge.from;
                                });
                    }),
                adjacencyList[i].end());
        }

        // Удаляем саму вершину
        adjacencyList.erase(adjacencyList.begin() + vertex);
        vertexPositions.erase(vertexPositions.begin() + vertex);
        --vertexCount;

        // Обновляем индексы оставшихся рёбер
        for (auto& edges : adjacencyList) {
            for (auto& edge : edges) {
                if (edge.from > vertex) --edge.from;
                if (edge.to > vertex) --edge.to;
            }
        }
    }

    // Алгоритм Прима для нахождения остовного дерева
    std::vector<Edge> primMST() {
        std::vector<bool> inMST(vertexCount, false);
        std::vector<Edge> mstEdges;

        auto cmp = [](const Edge& a, const Edge& b) { return a.weight > b.weight; };
        std::priority_queue<Edge, std::vector<Edge>, decltype(cmp)> pq(cmp);

        inMST[0] = true;
        for (const auto& edge : adjacencyList[0]) {
            pq.push(edge);
        }

        while (!pq.empty() && mstEdges.size() < vertexCount - 1) {
            Edge edge = pq.top();
            pq.pop();

            if (inMST[edge.to]) continue;

            mstEdges.push_back(edge);
            inMST[edge.to] = true;

            for (const auto& nextEdge : adjacencyList[edge.to]) {
                if (!inMST[nextEdge.to]) {
                    pq.push(nextEdge);
                }
            }
        }

        return mstEdges;
    }
};



class GraphAnalyzerApp {
private:
    sf::RenderWindow window; // Основное окно
    Graph graph;
    std::vector<sf::CircleShape> vertices;
    std::vector<std::pair<int, int>> edges;
    std::map<std::pair<int, int>, int> edgeWeights;
    sf::Font font;
    std::vector<sf::Text> vertexNumbers;
    std::vector<std::pair<int, int>> highlightedEdges;


    bool isAddingEdge = false;
    int selectedVertex = -1;
    int nextVertex = -1;

    // Переменные для окна ввода веса
    bool inputWindowActive = false;
    sf::RenderWindow inputWindow;
    std::string inputBuffer;

    // Меню для выбора алгоритма
    enum Algorithm { NONE, DIJKSTRA, PRIM, BFS, DFS };
    Algorithm selectedAlgorithm = NONE;

    // Окно для ввода начальной и конечной вершины
    bool inputPathWindowActive = false;
    sf::RenderWindow pathInputWindow;
    std::string startVertexBuffer;
    std::string endVertexBuffer;

public:
    GraphAnalyzerApp() : window(sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT), "Graph Analyzer"), graph(0) {
        if (!font.loadFromFile("arial.ttf")) {
            std::cerr << "Error loading font!" << std::endl;
            exit(1);
        }
    }

    void run() {
        while (window.isOpen()) {
            sf::Event event;
            while (window.pollEvent(event)) {
                if (event.type == sf::Event::Closed) {
                    window.close();
                }

                if (!inputWindowActive && !inputPathWindowActive) {
                    handleMouseEvents(event);
                    handleMenuEvents(event);
                }
            }

            if (inputWindowActive) {
                handleInputWindow();
            }

            if (inputPathWindowActive) {
                handlePathInputWindow();
            }

            render();
        }
    }

    void highlightTraversal(const std::vector<int>& traversalOrder) {
        for (int vertex : traversalOrder) {
            vertices[vertex].setFillColor(sf::Color::Cyan);
            window.clear();
            render();
            sf::sleep(sf::milliseconds(500)); // Задержка для визуализации
        }
    }

    void openStartVertexInputWindow(const std::string& algorithmType) {
        selectedAlgorithm = (algorithmType == "BFS") ? Algorithm::BFS : Algorithm::DFS;
        pathInputWindow.create(sf::VideoMode(300, 200), "Enter Start Vertex");
        inputPathWindowActive = true;
        startVertexBuffer.clear();
    }

    void handleMenuEvents(sf::Event& event) {
        if (event.type == sf::Event::MouseButtonPressed) {
            if (event.mouseButton.button == sf::Mouse::Left) {
                // Координаты кнопки "Dijkstra"
                if (event.mouseButton.x >= 10 && event.mouseButton.x <= 150 &&
                    event.mouseButton.y >= 10 && event.mouseButton.y <= 40) {
                    // Открываем окно для ввода начальной и конечной вершины
                    selectedAlgorithm = DIJKSTRA;
                    openPathInputWindow();
                }

                // Координаты кнопки "Import"
                if (event.mouseButton.x >= 10 && event.mouseButton.x <= 150 &&
                    event.mouseButton.y >= 10 + 40 && event.mouseButton.y <= 40 + 40) {
                    try {
                        graph.importFromFile("graph_export.txt");
                        std::cout << "Graph imported from graph_export.txt" << std::endl;
                        // Обновляем визуальные элементы
                        reloadGraphVisuals();
                    }
                    catch (const std::ios_base::failure& e) {
                        std::cerr << "Failed to import graph: " << e.what() << std::endl;
                    }
                }

                // Координаты кнопки "Export"
                if (event.mouseButton.x >= 10 && event.mouseButton.x <= 150 &&
                    event.mouseButton.y >= 10+80 && event.mouseButton.y <= 40+80) {
                    try {
                        graph.exportToFile("graph_export.txt");
                        std::cout << "Graph exported to graph_export.txt" << std::endl;
                    }
                    catch (const std::ios_base::failure& e) {
                        std::cerr << "Failed to export graph: " << e.what() << std::endl;
                    }
                }

                if (event.mouseButton.button == sf::Mouse::Left) {
                    // Координаты кнопки "BFS"
                    if (event.mouseButton.x >= 10 && event.mouseButton.x <= 150 &&
                        event.mouseButton.y >= 10 + 160 && event.mouseButton.y <= 40 + 160) {
                        openStartVertexInputWindow("BFS");
                    }

                    // Координаты кнопки "DFS"
                    if (event.mouseButton.x >= 10 && event.mouseButton.x <= 150 &&
                        event.mouseButton.y >= 10 + 200 && event.mouseButton.y <= 40 + 200) {
                        openStartVertexInputWindow("DFS");
                    }
                }
                if (event.mouseButton.button == sf::Mouse::Left) {
                    // Координаты кнопки "BFS"
                    if (event.mouseButton.x >= 10 && event.mouseButton.x <= 150 &&
                        event.mouseButton.y >= 10 + 160 && event.mouseButton.y <= 40 + 160) {
                        openStartVertexInputWindow("BFS");
                    }

                    // Координаты кнопки "DFS"
                    if (event.mouseButton.x >= 10 && event.mouseButton.x <= 150 &&
                        event.mouseButton.y >= 10 + 200 && event.mouseButton.y <= 40 + 200) {
                        openStartVertexInputWindow("DFS");
                    }
                }

                // Координаты кнопки "Prim"
                if (event.mouseButton.x >= 10 && event.mouseButton.x <= 150 &&
                    event.mouseButton.y >= 10 + 120 && event.mouseButton.y <= 40 + 120) {
                    highlightedEdges.clear();
                    auto mstEdges = graph.primMST();
                    for (const auto& edge : mstEdges) {
                        highlightedEdges.emplace_back(edge.from, edge.to);
                    }
                }
            }
        }
    }

    void openPathInputWindow() {
        pathInputWindow.create(sf::VideoMode(300, 200), "Enter Path (Dijkstra)");
        inputPathWindowActive = true;
        startVertexBuffer.clear();
        endVertexBuffer.clear();
    }

    void highlightShortestPath(const std::vector<std::pair<int, int>>& pathEdges) {
        for (const auto& edge : pathEdges) {
            sf::Vertex line[] = {
                sf::Vertex(vertices[edge.first].getPosition() + sf::Vector2f(VERTEX_RADIUS, VERTEX_RADIUS), sf::Color::Red),
                sf::Vertex(vertices[edge.second].getPosition() + sf::Vector2f(VERTEX_RADIUS, VERTEX_RADIUS), sf::Color::Red)
            };
            window.draw(line, 2, sf::Lines);
        }

        for (const auto& edge : pathEdges) {
            vertices[edge.first].setFillColor(sf::Color::Red);
            vertices[edge.second].setFillColor(sf::Color::Red);
        }
    }

    void handlePathInputWindow() {
        sf::Event event;
        while (pathInputWindow.pollEvent(event)) {
            if (event.type == sf::Event::Closed) {
                pathInputWindow.close();
                inputPathWindowActive = false;
            }

            if (!startVertexBuffer.empty()) {
                int start = std::stoi(startVertexBuffer);
                std::vector<int> traversalOrder;

                if (selectedAlgorithm == Algorithm::BFS) {
                    traversalOrder = graph.bfs(start);
                    std::cout << "BFS traversal order: ";
                }
                else if (selectedAlgorithm == Algorithm::DFS) {
                    traversalOrder = graph.dfs(start);
                    std::cout << "DFS traversal order: ";
                }

                // Выводим последовательность вершин в консоль
                for (size_t i = 0; i < traversalOrder.size(); ++i) {
                    std::cout << traversalOrder[i];
                    if (i != traversalOrder.size() - 1) {
                        std::cout << " -> ";
                    }
                }
                std::cout << std::endl;

                // Визуализируем обход
                highlightTraversal(traversalOrder);

                pathInputWindow.close();
                inputPathWindowActive = false;
            }

            if (!startVertexBuffer.empty()) {
                int start = std::stoi(startVertexBuffer);
                std::vector<int> traversalOrder;

                if (selectedAlgorithm == Algorithm::BFS) {
                    traversalOrder = graph.bfs(start);
                }
                else if (selectedAlgorithm == Algorithm::DFS) {
                    traversalOrder = graph.dfs(start);
                }

                highlightTraversal(traversalOrder);
                pathInputWindow.close();
                inputPathWindowActive = false;
            }

            if (event.type == sf::Event::TextEntered) {
                if (event.text.unicode == '\n' || event.text.unicode == '\r') {
                    if (!startVertexBuffer.empty() && !endVertexBuffer.empty()) {
                        int start = std::stoi(startVertexBuffer);
                        int end = std::stoi(endVertexBuffer);
                        highlightedEdges = graph.dijkstra(start, end);
                        pathInputWindow.close();
                        inputPathWindowActive = false;
                    }
                }
                else if (event.text.unicode == 8) {
                    if (!startVertexBuffer.empty()) {
                        startVertexBuffer.pop_back();
                    }
                    else if (!endVertexBuffer.empty()) {
                        endVertexBuffer.pop_back();
                    }
                }
                else if (std::isdigit(event.text.unicode)) {
                    if (startVertexBuffer.empty()) {
                        startVertexBuffer += static_cast<char>(event.text.unicode);
                    }
                    else {
                        endVertexBuffer += static_cast<char>(event.text.unicode);
                    }
                }
            }
        }

        pathInputWindow.clear(sf::Color(255, 165, 0)); // Оранжевый фон
        sf::Text inputPrompt("Enter Verteces:", font, 20);
        inputPrompt.setFillColor(sf::Color::Black);
        inputPrompt.setPosition(50, 50);

        sf::Text inputStart(startVertexBuffer, font, 24);
        inputStart.setFillColor(sf::Color::Black);
        inputStart.setPosition(50, 100);

        sf::Text inputEnd(endVertexBuffer, font, 24);
        inputEnd.setFillColor(sf::Color::Black);
        inputEnd.setPosition(50, 150);

        pathInputWindow.draw(inputPrompt);
        pathInputWindow.draw(inputStart);
        pathInputWindow.draw(inputEnd);
        pathInputWindow.display();
    }

    void handleMouseEvents(sf::Event& event) {
        if (event.type == sf::Event::MouseButtonPressed) {
            if (event.mouseButton.button == sf::Mouse::Right) {
                int vertexIndex = findVertex(event.mouseButton.x, event.mouseButton.y);
                if (vertexIndex != -1) {
                    if (sf::Keyboard::isKeyPressed(sf::Keyboard::LShift) ||
                        sf::Keyboard::isKeyPressed(sf::Keyboard::RShift)) {
                        deleteVertex(vertexIndex); // Удаление вершины
                    }
                    else {
                        if (isAddingEdge) {
                            nextVertex = vertexIndex;
                            openInputWindow(); // Открываем окно ввода веса
                        }
                        else {
                            isAddingEdge = true;
                            selectedVertex = vertexIndex;
                        }
                    }
                }
            }
            else if (event.mouseButton.button == sf::Mouse::Left) {
                if (!(event.mouseButton.x >= 10 && event.mouseButton.x <= 150 &&
                    event.mouseButton.y >= 10 && event.mouseButton.y <= 40) && !(event.mouseButton.x >= 10 && event.mouseButton.x <= 150 &&
                        event.mouseButton.y >= 10 + 40 && event.mouseButton.y <= 40 + 40) && !(event.mouseButton.x >= 10 && event.mouseButton.x <= 150 &&
                            event.mouseButton.y >= 10 + 80 && event.mouseButton.y <= 40 + 80) && !(event.mouseButton.x >= 10 && event.mouseButton.x <= 150 &&
                                event.mouseButton.y >= 10 + 120 && event.mouseButton.y <= 40 + 120) && !(event.mouseButton.x >= 10 && event.mouseButton.x <= 150 &&
                                    event.mouseButton.y >= 10 + 200 && event.mouseButton.y <= 40 + 200) && !(event.mouseButton.x >= 10 && event.mouseButton.x <= 150 &&
                                        event.mouseButton.y >= 10 + 160 && event.mouseButton.y <= 40 + 160)) {
                    // Добавление новой вершины при клике левой кнопкой мыши
                    addVertex(event.mouseButton.x, event.mouseButton.y);
                }
                
            }
        }
    }

    void openInputWindow() {
        inputWindow.create(sf::VideoMode(300, 200), "Enter Edge Weight");
        inputWindowActive = true;
        inputBuffer.clear();
    }

    void handleInputWindow() {
        sf::Event event;
        while (inputWindow.pollEvent(event)) {
            if (event.type == sf::Event::Closed) {
                inputWindow.close();
                inputWindowActive = false;
                isAddingEdge = false; // Сбрасываем состояние добавления ребра
            }

            if (event.type == sf::Event::TextEntered) {
                if (event.text.unicode == '\n' || event.text.unicode == '\r') {
                    int weight = std::stoi(inputBuffer);
                    addEdge(nextVertex, weight);
                    inputWindow.close();
                    inputWindowActive = false;
                }
                else if (event.text.unicode == 8) {
                    if (!inputBuffer.empty()) {
                        inputBuffer.pop_back();
                    }
                }
                else if (std::isdigit(event.text.unicode) || event.text.unicode == '-') {
                    inputBuffer += static_cast<char>(event.text.unicode);
                }
            }
        }

        inputWindow.clear(sf::Color(255, 165, 0)); // Оранжевый фон
        sf::Text inputPrompt("Enter Weight:", font, 20);
        inputPrompt.setFillColor(sf::Color::Black);
        inputPrompt.setPosition(50, 50);

        sf::Text inputDisplay(inputBuffer, font, 24);
        inputDisplay.setFillColor(sf::Color::Black);
        inputDisplay.setPosition(50, 100);

        inputWindow.draw(inputPrompt);
        inputWindow.draw(inputDisplay);
        inputWindow.display();
    }


    void addVertex(float x, float y) {
        graph.addVertex(x, y); // Добавляем вершину в граф
        sf::CircleShape newVertex(VERTEX_RADIUS);
        newVertex.setFillColor(sf::Color::Green);
        newVertex.setPosition(x - VERTEX_RADIUS, y - VERTEX_RADIUS); // Центрируем вершину
        vertices.push_back(newVertex);

        // Создаем текст для номера вершины
        sf::Text vertexNumber;
        vertexNumber.setFont(font); // Используем загруженный шрифт
        vertexNumber.setString(std::to_string(graph.getVertexCount() - 1)); // Номер вершины
        vertexNumber.setCharacterSize(16); // Размер шрифта
        vertexNumber.setFillColor(sf::Color::Black);

        // Располагаем текст в центре вершины
        sf::FloatRect textBounds = vertexNumber.getLocalBounds();
        vertexNumber.setPosition(
            x - textBounds.width / 2,
            y - textBounds.height / 2
        );

        // Сохраняем текстовый объект
        vertexNumbers.push_back(vertexNumber);
    }

    void addEdge(int toVertex, int weight) {
        if (selectedVertex != -1 && toVertex != selectedVertex) {
            graph.addEdge(selectedVertex, toVertex, weight);
            edges.emplace_back(selectedVertex, toVertex);
            edgeWeights[{selectedVertex, toVertex}] = weight;
            edgeWeights[{toVertex, selectedVertex}] = weight; // Для неориентированного графа
            std::cout << "Edge added: " << selectedVertex << " -> " << toVertex << " with weight " << weight << std::endl;
        }
        else {
            std::cerr << "Unable to add edge: select different vertices." << std::endl;
        }
        isAddingEdge = false;
        selectedVertex = -1;
    }

    int findVertex(float x, float y) {
        for (size_t i = 0; i < vertices.size(); ++i) {
            float vx = vertices[i].getPosition().x + VERTEX_RADIUS;
            float vy = vertices[i].getPosition().y + VERTEX_RADIUS;
            if (std::sqrt((vx - x) * (vx - x) + (vy - y) * (vy - y)) <= VERTEX_RADIUS) {
                return i;
            }
        }
        return -1;
    }

    void reloadGraphVisuals() {
        vertices.clear();
        edges.clear();
        vertexNumbers.clear();
        edgeWeights.clear(); // Сбрасываем веса рёбер для корректной перезагрузки

        for (int i = 0; i < graph.getVertexCount(); ++i) {
            // Восстановление вершин
            auto [x, y] = graph.getVertexPosition(i);
            sf::CircleShape newVertex(VERTEX_RADIUS);
            newVertex.setFillColor(sf::Color::Green);
            newVertex.setPosition(x, y);
            vertices.push_back(newVertex);

            // Добавление номера вершины
            sf::Text vertexNumber;
            vertexNumber.setFont(font);
            vertexNumber.setString(std::to_string(i));
            vertexNumber.setCharacterSize(16);
            vertexNumber.setFillColor(sf::Color::Black);
            vertexNumber.setPosition(
                x + VERTEX_RADIUS / 2,
                y + VERTEX_RADIUS / 2
            );
            vertexNumbers.push_back(vertexNumber);

            // Восстановление рёбер
            for (int i = 0; i < graph.getVertexCount(); ++i) {
                for (const auto& edge : graph.getEdges(i)) {
                    if (edge.from < edge.to) { // Избегаем дублирования рёбер
                        edges.emplace_back(edge.from, edge.to);
                        edgeWeights[{edge.from, edge.to}] = edge.weight;
                    }
                }
            }
        }
    }

    void render() {
        window.clear(sf::Color::White);

        // Рисуем все рёбра
        for (const auto& edge : edges) {
            sf::Color lineColor = sf::Color::Black;

            // Проверяем, нужно ли подсветить ребро
            if (std::find(highlightedEdges.begin(), highlightedEdges.end(), edge) != highlightedEdges.end() ||
                std::find(highlightedEdges.begin(), highlightedEdges.end(), std::make_pair(edge.second, edge.first)) != highlightedEdges.end()) {
                lineColor = sf::Color::Red;
            }

            sf::Vertex line[] = {
                sf::Vertex(vertices[edge.first].getPosition() + sf::Vector2f(VERTEX_RADIUS, VERTEX_RADIUS), lineColor),
                sf::Vertex(vertices[edge.second].getPosition() + sf::Vector2f(VERTEX_RADIUS, VERTEX_RADIUS), lineColor)
            };
            window.draw(line, 2, sf::Lines);

            // Отображаем вес ребра
            int midX = (vertices[edge.first].getPosition().x + vertices[edge.second].getPosition().x) / 2;
            int midY = (vertices[edge.first].getPosition().y + vertices[edge.second].getPosition().y) / 2;

            sf::Text weightText(std::to_string(edgeWeights[edge]), font, 16);
            weightText.setFillColor(sf::Color::Black);
            weightText.setPosition(midX, midY);
            window.draw(weightText);
        }

        // Рисуем вершины
        for (const auto& vertex : vertices) {
            window.draw(vertex);
        }

        // Рисуем номера вершин
        for (const auto& number : vertexNumbers) {
            window.draw(number);
        }

        //// Обновляем метод handleMenuEvents
        //void handleMenuEvents(sf::Event & event) {
        //    if (event.type == sf::Event::MouseButtonPressed) {
        //        if (event.mouseButton.button == sf::Mouse::Left) {
        //            // Координаты кнопки "BFS"
        //            if (event.mouseButton.x >= 10 && event.mouseButton.x <= 150 &&
        //                event.mouseButton.y >= 10 + 160 && event.mouseButton.y <= 40 + 160) {
        //                openTraversalInputWindow(BFS);
        //            }

        //            // Координаты кнопки "DFS"
        //            if (event.mouseButton.x >= 10 && event.mouseButton.x <= 150 &&
        //                event.mouseButton.y >= 10 + 200 && event.mouseButton.y <= 40 + 200) {
        //                openTraversalInputWindow(DFS);
        //            }
        //        }
        //    }
        //}

        // Кнопка "BFS"
        sf::RectangleShape bfsButton(sf::Vector2f(140, 30));
        bfsButton.setFillColor(sf::Color(200, 200, 200));
        bfsButton.setPosition(10, 10 + 160);
        window.draw(bfsButton);

        sf::Text bfsText("BFS", font, 16);
        bfsText.setFillColor(sf::Color::Black);
        bfsText.setPosition(25, 15 + 160);
        window.draw(bfsText);

        // Кнопка "DFS"
        sf::RectangleShape dfsButton(sf::Vector2f(140, 30));
        dfsButton.setFillColor(sf::Color(200, 200, 200));
        dfsButton.setPosition(10, 10 + 200);
        window.draw(dfsButton);

        sf::Text dfsText("DFS", font, 16);
        dfsText.setFillColor(sf::Color::Black);
        dfsText.setPosition(25, 15 + 200);
        window.draw(dfsText);

        // Кнопка "Dijkstra"
        sf::RectangleShape dijkstraButton(sf::Vector2f(140, 30));
        dijkstraButton.setFillColor(sf::Color(200, 200, 200));
        dijkstraButton.setPosition(10, 10);
        window.draw(dijkstraButton);

        sf::Text dijkstraText("Dijkstra", font, 16);
        dijkstraText.setFillColor(sf::Color::Black);
        dijkstraText.setPosition(25, 15);
        window.draw(dijkstraText);

        // Кнопка "Import"
        sf::RectangleShape importButton(sf::Vector2f(140, 30));
        importButton.setFillColor(sf::Color(200, 200, 200));
        importButton.setPosition(10, 10+40);
        window.draw(importButton);

        sf::Text importText("Import", font, 16);
        importText.setFillColor(sf::Color::Black);
        importText.setPosition(25, 15+40);
        window.draw(importText);

        // Кнопка "Export"
        sf::RectangleShape exportButton(sf::Vector2f(140, 30));
        exportButton.setFillColor(sf::Color(200, 200, 200));
        exportButton.setPosition(10, 10 + 80);
        window.draw(exportButton);

        sf::Text exportText("Export", font, 16);
        exportText.setFillColor(sf::Color::Black);
        exportText.setPosition(25, 15 + 80);
        window.draw(exportText);

        // Кнопка "Prim"
        sf::RectangleShape primButton(sf::Vector2f(140, 30));
        primButton.setFillColor(sf::Color(200, 200, 200));
        primButton.setPosition(10, 10 + 120);
        window.draw(primButton);

        sf::Text primText("Prim", font, 16);
        primText.setFillColor(sf::Color::Black);
        primText.setPosition(25, 15 + 120);
        window.draw(primText);

        window.display();
    }

    void deleteVertex(int vertexIndex) {
        if (vertexIndex < 0 || vertexIndex >= static_cast<int>(vertices.size())) {
            std::cerr << "Invalid vertex index: " << vertexIndex << "\n";
            return;
        }

        // Удаляем вершину из графа
        graph.removeVertex(vertexIndex);

        // Удаляем визуальные элементы, связанные с вершиной
        vertices.erase(vertices.begin() + vertexIndex);
        vertexNumbers.erase(vertexNumbers.begin() + vertexIndex);

        // Обновляем рёбра и веса
        edges.clear();
        edgeWeights.clear();
        reloadGraphVisuals();

        std::cout << "Vertex " << vertexIndex << " and its edges were deleted.\n";
    }
};

int main() {
    GraphAnalyzerApp app;
    app.run();
    return 0;
}