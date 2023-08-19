#include <iostream>
#include <vector>
#include <queue>
#include <stack>
#include <limits>
#include <unordered_map>
#include <sstream>

using namespace std ;

class Node {
public:
    string id;
    string type;
    string galaxy;

    Node(const string& _id, const string& _type, const string& _galaxy)
        : id(_id), type(_type), galaxy(_galaxy) {}
};

class Edge {
public:
    Node* start;
    Node* end;
    int cost;

    Edge(Node* _start, Node* _end, int _cost)
        : start(_start), end(_end), cost(_cost) {}
};

class Galaxy {
public:
    string name;
    vector<Node*> nodes;
    vector<Edge*> edges;

    Galaxy(const string& _name) : name(_name) {}

    void addNode(Node* node) {
        nodes.push_back(node);
    }

    void addEdge(Edge* edge) {
        edges.push_back(edge);
    }
};

class Universe {
public:
    vector<Galaxy*> galaxies;

    void addGalaxy(Galaxy* galaxy) {
        galaxies.push_back(galaxy);
    }
};

////////////////////////
unordered_map<Node*, int> dijkstra(Node* source, const vector<Edge*>& edges) {
    unordered_map<Node*, int> distances;
    priority_queue<pair<int, Node*>, vector<pair<int, Node*>>, greater<>> pq;

    for (Edge* edge : edges) {
        if (edge->start == source) {
            distances[edge->end] = edge->cost;
            pq.push(make_pair(edge->cost, edge->end));
        }
    }

    while (!pq.empty()) {
        int dist = pq.top().first;
        Node* current = pq.top().second;
        pq.pop();

        if (dist > distances[current]) {
            continue;
        }

        for (Edge* edge : edges) {
            if (edge->start == current) {
                int newDist = dist + edge->cost;
                if (!distances.count(edge->end) || newDist < distances[edge->end]) {
                    distances[edge->end] = newDist;
                    pq.push(make_pair(newDist, edge->end));
                }
            }
        }
    }

    return distances;
}

stack<Node*> getPath(const unordered_map<Node*, Node*>& prevMap, Node* target) {
    stack<Node*> path;
    Node* current = target;

    while (current != nullptr) {
        path.push(current);
        current = prevMap.at(current);
    }

    return path;
}



struct RoutingTable {
    unordered_map<Node*, unordered_map<Node*, stack<Node*> >> tables;

    void updateTable(Node* source, Node* target, const stack<Node*>& path) {
        tables[source][target] = path;
    }

    const stack<Node*>& getPath(Node* source, Node* target) {
        return tables[source][target];
    }
};


RoutingTable generateRoutingTables(const Universe& universe) {
    RoutingTable routingTable;

    for (Galaxy* galaxy : universe.galaxies) {
        for (Node* sourceNode : galaxy->nodes) {
            // Find the edges that connect to nodes within the same galaxy as the source node
            vector<Edge*> edgesWithinGalaxy;
            for (Edge* edge : galaxy->edges) {
                if (edge->start == sourceNode || edge->end == sourceNode) {
                    edgesWithinGalaxy.push_back(edge);
                }
            }
            unordered_map<Node*, int> distances = dijkstra(sourceNode, edgesWithinGalaxy);
            
            unordered_map<Node*, Node*> prevMap;
            for (Edge* edge : edgesWithinGalaxy) {
                if (edge->start == sourceNode) {
                    prevMap[edge->end] = sourceNode;
                }
            }

            for (Node* targetNode : galaxy->nodes) {
                if (sourceNode != targetNode) {
                    if (sourceNode->type == "BG") {
                        for (Galaxy* targetGalaxy : universe.galaxies) {
                            if (galaxy != targetGalaxy) {
                                for (Node* targetNode : targetGalaxy->nodes) {
                                    if (targetNode->type == "BG") {
                                        unordered_map<Node*, int> distances = dijkstra(sourceNode, edgesWithinGalaxy);
                                        stack<Node*> path = getPath(prevMap, targetNode);
                                        routingTable.updateTable(sourceNode, targetNode, path);
                                    }
                                }
                            }
                        }
                    }
                    else {
                    stack<Node*> path = getPath(prevMap, targetNode);
                    routingTable.updateTable(sourceNode, targetNode, path);
                    }
                }
            }

        }
    }

    

    return routingTable;
}

///////////////////////////////////////////


stack<Node*> findPathWithinGalaxy( RoutingTable& routingTable, Node* source, Node* target) {

    return routingTable.getPath(source, target);
     
}

stack<Node*> findPathAcrossGalaxies( RoutingTable& routingTable, Node* source, Node* target, const Universe& universe) {
    // Find the nearest BG node connected to the target galaxy
    Node* nearestBG = nullptr;
    int minDistance = 99999999; // infinte

    // Find the target node's galaxy
    Galaxy* targetGalaxy = nullptr;
    for (Galaxy* galaxy : universe.galaxies) {
        for (Node* node : galaxy->nodes) {
            if (node == target) {
                targetGalaxy = galaxy;
                break;
            }
        }
        if (targetGalaxy) {
            break;
        }
    }

    //Hot Potato ( find nearsest BG node from target galaxy )
    for (Node* node : targetGalaxy->nodes) {
        if (node->type == "BG") {
            stack<Node*> path = routingTable.getPath(source, node);
            if (path.size() < minDistance) {
                nearestBG = node;
                minDistance = path.size();
            }
        }
    }

    // Calculate path from source to nearest BG node
    stack<Node*> pathToNearestBG = routingTable.getPath(source, nearestBG);

    // Calculate path from nearest BG node to target galaxy
    stack<Node*> pathToTargetGalaxy = routingTable.getPath(nearestBG, target);

    // Combine paths
    stack<Node*> combinedPath = pathToNearestBG;
    while (!pathToTargetGalaxy.empty()) {
        combinedPath.push(pathToTargetGalaxy.top());
        pathToTargetGalaxy.pop();
    }

    return combinedPath;
}

void printPath( stack<Node*>& path, Galaxy& sourceGalaxy, Galaxy& targetGalaxy) {
    int totalCost = 0;
    string result;
    
    while (!path.empty()) {
        Node* node = path.top();
        path.pop();

        if (!path.empty()) {
            // Find the corresponding edge between node and path.top()
            Edge* edge = nullptr;

            vector<Edge*> allEdges;
            allEdges.reserve(sourceGalaxy.edges.size() + targetGalaxy.edges.size());
            allEdges.insert(allEdges.end(), sourceGalaxy.edges.begin(), sourceGalaxy.edges.end());
            allEdges.insert(allEdges.end(), targetGalaxy.edges.begin(), targetGalaxy.edges.end());

            for (Edge* e : allEdges) {
                if ((e->start == node && e->end == path.top()) || (e->end == node && e->start == path.top())) {
                    edge = e;
                    break;
                }
            }

            if (edge) {
                totalCost += edge->cost;
                result += node->id + "->";
            }
        } else {
            result += node->id;
        }
    }

    cout << result << " : " << totalCost << endl;
}



int main() {

    Universe universe;
    unordered_map<string, Node*> nodesMap;
    unordered_map<string, Galaxy*> galaxiesMap;

    string input;
    while (true) {
        cout << "Enter command: ";
        getline(cin, input);

        if (input == "EXIT") {
            break;
        }

        // Tokenize the input command
        vector<string> tokens;
        istringstream iss(input);
        string token;
        while (iss >> token) {
            tokens.push_back(token);
        }

        // Process different commands
        if (tokens[0] == "CREATE") {
            if (tokens[1] == "(A:" && tokens[2] == "Node," && tokens[3].substr(0, 5) == "{id:'"  ) {
                string id, type, galaxy;
                // Extract id, type, galaxy from tokens[3], tokens[4], tokens[6]

                id = tokens[3].substr(5, tokens[3].size() - 6);
                type = tokens[4].substr(6, tokens[4].size() - 7);
                galaxy = tokens[6].substr(9, tokens[6].size() - 8);

                // Create Galaxy if it doesn't exist
                if (galaxiesMap.find(galaxy) == galaxiesMap.end()) {
                    Galaxy* newGalaxy = new Galaxy(galaxy);
                    galaxiesMap[galaxy] = newGalaxy;
                    universe.addGalaxy(newGalaxy);
                }


                // Create Node object and add it to the galaxy
                Node* newNode = new Node(id, type, galaxy);
                galaxiesMap[galaxy]->addNode(newNode);
                nodesMap[id] = newNode;

                // Handle cases for BG and non-BG nodes
                if (type == "BG") {
                    // Create Galaxy if it doesn't exist
                    if (galaxiesMap.find(galaxy) == galaxiesMap.end()) {
                        Galaxy* newGalaxy = new Galaxy(galaxy);
                        galaxiesMap[galaxy] = newGalaxy;
                        universe.addGalaxy(newGalaxy);
                    }
                    galaxiesMap[galaxy]->addNode(newNode);
                } else {
                    // Add non-BG node to its galaxy
                    galaxiesMap[galaxy]->addNode(newNode);
                }
            }
            else if (tokens[1] == "(AS" && tokens[3] == ") - [:ROAD" && tokens[4].substr(0, 5) == "{cost:") {
                std::string startNodeId, endNodeId;
                int cost;
                // Extract startNodeId, endNodeId, and cost from tokens[1], tokens[5], tokens[4]
                startNodeId = tokens[1].substr(2);
                endNodeId = tokens[5].substr(0, tokens[5].size() - 1);
                cost = std::stoi(tokens[4].substr(7, tokens[4].size() - 8));

                if (cost <= 0) {
                    std::cerr << "Invalid edge cost " << std::endl;
                    break;
                }


                // Find nodes using nodesMap
                Node* startNode = nodesMap[startNodeId];
                Node* endNode = nodesMap[endNodeId];

                // Create Edge object and add it to the corresponding galaxy
                Edge* newEdge = new Edge(startNode, endNode, cost);
                galaxiesMap[startNode->galaxy]->addEdge(newEdge);
            }




        }
        else if (tokens[0] == "FIND") {
                if (tokens.size() >= 3 && tokens[1].substr(0, 2) == "AS" && tokens[2].substr(0, 2) == "AS") {
                        std::string sourceNodeId, targetNodeId;
                        sourceNodeId = tokens[1];
                        targetNodeId = tokens[2];

                        // Find nodes using nodesMap
                        Node* sourceNode = nodesMap[sourceNodeId];
                        Node* targetNode = nodesMap[targetNodeId];

                        if (!sourceNode || !targetNode) {
                            throw std::runtime_error("Source or target node not found.");
                        }

                        // Use generatedRoutingTables to find the shortest path and print it
                        RoutingTable routingTable = generateRoutingTables(universe);
                        std::stack<Node*> path = findPathAcrossGalaxies(routingTable, sourceNode, targetNode, universe);

                        if (path.empty()) {
                            throw std::runtime_error("No path found between source and target nodes.");
                        }

                        printPath(path, *galaxiesMap[sourceNode->galaxy], *galaxiesMap[targetNode->galaxy]);

                }
                else {
                    throw std::runtime_error("Invalid FIND command format.");
                }
            }
            else {
                throw std::runtime_error("Invalid command.");
            }
    }
    

    // Clean up memory (delete objects, etc.)

    return 0;
}


/*
int main() {
    Universe universe;

    Node* AS1_A = new Node("AS1.A", "NON-BG","AS1");
    Node* AS1_B = new Node("AS1.B", "BG","AS1") ;
    Node* AS1_C = new Node("AS1.C", "NON-BG","AS1") ;
    Node* AS1_D = new Node("AS1.D", "BG", "AS1");
    Node* AS2_A = new Node("AS2.A", "BG", "AS2");
    Node* AS2_B = new Node("AS2.B", "NON-BG", "AS2");
    Node* AS2_C = new Node("AS2.C", "BG", "AS2");
    Node* AS2_D = new Node("AS2.D", "NON-BG","AS2");

    vector<Edge*> edges = {
        new Edge(AS1_A, AS1_B, 20),
        new Edge(AS1_A, AS1_C, 10),
        new Edge(AS1_B, AS1_D, 5),
        new Edge(AS1_C, AS1_D, 30),
        new Edge(AS1_B, AS2_A, 10),
        new Edge(AS1_D, AS2_C, 30),
        new Edge(AS2_A, AS2_B, 5),
        new Edge(AS2_A, AS2_C, 10),
        new Edge(AS2_B, AS2_D, 4),
        new Edge(AS2_C, AS2_D, 20)
    };

    Galaxy* galaxyAS1 = new Galaxy("AS1");
    galaxyAS1->addNode(AS1_A);
    galaxyAS1->addNode(AS1_B);
    galaxyAS1->addNode(AS1_C);
    galaxyAS1->addNode(AS1_D);
    galaxyAS1->addEdge(edges[0]);
    galaxyAS1->addEdge(edges[1]);
    galaxyAS1->addEdge(edges[2]);
    galaxyAS1->addEdge(edges[3]);

    Galaxy* galaxyAS2 = new Galaxy("AS2");
    galaxyAS2->addNode(AS2_A);
    galaxyAS2->addNode(AS2_B);
    galaxyAS2->addNode(AS2_C);
    galaxyAS2->addNode(AS2_D);
    galaxyAS2->addEdge(edges[4]);
    galaxyAS2->addEdge(edges[5]);
    galaxyAS2->addEdge(edges[6]);
    galaxyAS2->addEdge(edges[7]);
    galaxyAS2->addEdge(edges[8]);
    galaxyAS2->addEdge(edges[9]);

    universe.addGalaxy(galaxyAS1);
    universe.addGalaxy(galaxyAS2);



///////////////////////////////

    RoutingTable routingTable = generateRoutingTables(universe);



    Node* sourceNode = AS1_A;
    Node* targetNode = AS2_C;
    stack<Node*> path2 = findPathAcrossGalaxies(routingTable, sourceNode, targetNode, universe );
    printPath(path2, *galaxyAS1, *galaxyAS2);

    
 

    

    return 0;
}
*/