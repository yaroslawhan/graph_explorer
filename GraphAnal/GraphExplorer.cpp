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

public:
    Graph(int n) : vertexCount(n) {
        adjacencyList.resize(n);
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

    void addVertex() {
        adjacencyList.push_back({});
        ++vertexCount;
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
    enum Algorithm { NONE, DIJKSTRA };
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
        sf::Text inputPrompt("Start Vertex:", font, 20);
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
            else if (event.mouseButton.button == sf::Mouse::Left) {
                // Добавление новой вершины при клике левой кнопкой мыши
                addVertex(event.mouseButton.x, event.mouseButton.y);
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
        graph.addVertex(); // Добавляем вершину в граф
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

        // Кнопка "Dijkstra"
        sf::RectangleShape dijkstraButton(sf::Vector2f(140, 30));
        dijkstraButton.setFillColor(sf::Color(200, 200, 200));
        dijkstraButton.setPosition(10, 10);
        window.draw(dijkstraButton);

        sf::Text dijkstraText("Dijkstra", font, 16);
        dijkstraText.setFillColor(sf::Color::Black);
        dijkstraText.setPosition(25, 15);
        window.draw(dijkstraText);

        window.display();
    }
};

int main() {
    GraphAnalyzerApp app;
    app.run();
    return 0;
}
