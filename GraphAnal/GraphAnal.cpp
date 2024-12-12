#include <SFML/Graphics.hpp> 
#include <SFML/Window.hpp> 
#include <iostream> 
#include <vector> 
#include <cmath> 
#include <string> 
#include <limits> 
#include <queue> 
#include <clocale> 

const int WINDOW_WIDTH = 800;
const int WINDOW_HEIGHT = 600;

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
        adjacencyList[to].push_back({ to, from, weight }); // For undirected graph 
    }

    const std::vector<Edge>& getEdges(int vertex) const {
        return adjacencyList[vertex];
    }

    int getVertexCount() const {
        return vertexCount;
    }

    // Dijkstra's Algorithm 
    std::vector<int> dijkstra(int start) {
        std::vector<int> dist(vertexCount, std::numeric_limits<int>::max());
        dist[start] = 0;

        std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>, std::greater<>> pq;
        pq.push({ 0, start });

        while (!pq.empty()) {
            int d = pq.top().first;
            int u = pq.top().second;
            pq.pop();

            if (d > dist[u]) continue;

            for (const Edge& edge : adjacencyList[u]) {
                int v = edge.to;
                int weight = edge.weight;

                if (dist[u] + weight < dist[v]) {
                    dist[v] = dist[u] + weight;
                    pq.push({ dist[v], v });
                }
            }
        }
        return dist;
    }
};

class GraphAnalyzerApp {
private:
    sf::RenderWindow window;
    Graph graph;
    std::vector<sf::CircleShape> vertices;
    std::vector<std::pair<int, int>> edges;
    std::map<std::pair<int, int>, int> edgeWeights;
    sf::Font font;
    bool isAddingEdge = false;
    int selectedVertex = -1;

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
                handleMouseEvents(event);
                handleKeyboardEvents(event);
            }
            render();
        }
    }

    void handleMouseEvents(sf::Event& event) {
        if (event.type == sf::Event::MouseButtonPressed) {
            if (event.mouseButton.button == sf::Mouse::Left) {
                addVertex(event.mouseButton.x, event.mouseButton.y);
            }
            else if (event.mouseButton.button == sf::Mouse::Right) {
                int vertexIndex = findVertex(event.mouseButton.x, event.mouseButton.y);
                if (vertexIndex != -1) {
                    if (isAddingEdge) {
                        // Adding an edge 
                        int weight = getEdgeWeight(); // Enter edge weight 
                        if (weight >= 0) { // Only if the weight is correctly entered 
                            graph.addEdge(selectedVertex, vertexIndex, weight);
                            edges.emplace_back(selectedVertex, vertexIndex);
                            edgeWeights[{selectedVertex, vertexIndex}] = weight;
                            edgeWeights[{vertexIndex, selectedVertex}] = weight;
                        }
                        isAddingEdge = false;
                        selectedVertex = -1;
                    }
                    else {
                        // Selecting the first vertex to add an edge 
                        isAddingEdge = true;
                        selectedVertex = vertexIndex;
                    }
                }
            }
        }
    }
    void handleKeyboardEvents(sf::Event& event) {
        if (event.type == sf::Event::KeyPressed) {
            if (event.key.code == sf::Keyboard::D) {
                executeDijkstra();
            }
        }
    }

    void addVertex(float x, float y) {
        sf::CircleShape vertex(15);
        vertex.setPosition(x - 15, y - 15);
        vertex.setFillColor(sf::Color::Green);
        vertices.push_back(vertex);
        graph = Graph(vertices.size());
    }

    int findVertex(float x, float y) {
        for (size_t i = 0; i < vertices.size(); ++i) {
            float vx = vertices[i].getPosition().x + 15;
            float vy = vertices[i].getPosition().y + 15;
            if (std::sqrt((vx - x) * (vx - x) + (vy - y) * (vy - y)) <= 15) {
                return i;
            }
        }
        return -1;
    }

    int getEdgeWeight() {
        int weight;
        std::cout << "Enter weight for the edge: ";
        std::cin >> weight;
        if (std::cin.fail() || weight < 0) {
            std::cerr << "Invalid weight input!" << std::endl;
            return -1;
        }
        return weight;
    }

    void executeDijkstra() {
        if (vertices.empty()) return;

        int startVertex, endVertex;
        std::cout << "Enter the start vertex: ";
        std::cin >> startVertex;
        std::cout << "Enter the end vertex: ";
        std::cin >> endVertex;

        if (startVertex < 0 || startVertex >= graph.getVertexCount() ||
            endVertex < 0 || endVertex >= graph.getVertexCount()) {
            std::cerr << "Invalid vertices!" << std::endl;
            return;
        }

        auto distances = graph.dijkstra(startVertex);

        if (distances[endVertex] != std::numeric_limits<int>::max()) {
            std::cout << "Shortest distance from vertex " << startVertex << " to vertex " << endVertex << ": "
                << distances[endVertex] << std::endl;
        }
        else {
            std::cout << "No path from vertex " << startVertex << " to vertex " << endVertex << "." << std::endl;
        }
    }

    void render() {
        window.clear(sf::Color::White);

        // Rendering edges 
        for (const auto& edge : edges) {
            sf::Vertex line[] = {
                sf::Vertex(vertices[edge.first].getPosition() + sf::Vector2f(15, 15), sf::Color::Black),
                sf::Vertex(vertices[edge.second].getPosition() + sf::Vector2f(15, 15), sf::Color::Black) };
            window.draw(line, 2, sf::Lines);

            // Rendering edge weight 
            auto midPoint = (vertices[edge.first].getPosition() + vertices[edge.second].getPosition()) / 2.0f + sf::Vector2f(15, 15);
            sf::Text text;
            text.setFont(font);
            text.setString(std::to_string(edgeWeights[{edge.first, edge.second}]));
            text.setCharacterSize(14);
            text.setFillColor(sf::Color::Red);
            text.setPosition(midPoint);
            window.draw(text);
        }

        // Rendering vertices 
        for (size_t i = 0; i < vertices.size(); ++i) {
            window.draw(vertices[i]);

            // Rendering vertex numbers 
            sf::Text text;
            text.setFont(font);
            text.setString(std::to_string(i));
            text.setCharacterSize(14);
            text.setFillColor(sf::Color::Black);
            text.setPosition(vertices[i].getPosition().x + 10, vertices[i].getPosition().y + 5);
            window.draw(text);
        }

        window.display();
    }
};

int main() {
    //std::setlocale(LC_ALL, "ru_RU.UTF-8"); 
    GraphAnalyzerApp app;
    app.run();
    return 0;
}