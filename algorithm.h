#include <vector>
#include <cstdlib>
#include <iostream>
#include <ctime>
#include <cmath>
#include <queue>
#include <algorithm>
#include <stack>
using namespace std;
// alias
using Graph = vector<vector<pair<int,float>>>;

class Node
{
public:
    int id;
    float x;
    float y;
    Node *parent;
    Node() : id(0), x(0), y(0), parent(nullptr) {}
    Node(int id, float x, float y)
    {
        this->id = id;
        this->x = x;
        this->y = y;
        this->parent = nullptr;
    }
};


vector<vector<float>> generatepoints(float x1, float x2, float y1, float y2) {
    vector<vector<float>> Points;
    srand(time(0));

    int n = 32; // number of points to generate
    float dx = x2 - x1;
    float dy = y2 - y1;
    float length = sqrt(dx * dx + dy * dy);

    // Unit vector along the line
    float ux = dx / length;
    float uy = dy / length;

    // Perpendicular unit vector
    float px = -uy;
    float py = ux;

    for (int i = 0; i < n; ++i) {
        // t is a position along the line, from 0 to 1
        float t = ((float)rand() / RAND_MAX);

        // offset is a small perpendicular displacement, e.g., up to ±0.2 units (not to go out of the space)
        float offset = (((float)rand() / RAND_MAX) - 0.5) * 0.4;

        // Base point on the line
        float baseX = x1 + t * dx;
        float baseY = y1 + t * dy;

        // Final point with offset
        float xPoint = baseX + offset * px;
        float yPoint = baseY + offset * py;

        //guarantee points inside the space
        if (xPoint < -0.5 || xPoint > 0.5 || yPoint < -0.5 || yPoint > 0.5)
         continue;

        Points.push_back({xPoint, yPoint});
    }

    return Points;
}



 vector<vector<float>> FreePoints(vector<vector<float>> obstacles ,vector<vector<float>> Points ){
    vector<vector<float>> freePoints;
    //whether in or out
    int sign;
    for(int i = 0 ; i < Points.size() ; ++i)
    {
        for(int j = 0 ; j < obstacles.size() ; ++j)
        {
            sign = (Points[i][0]- obstacles[j][0])*(Points[i][0]- obstacles[j][0]) + 
                   (Points[i][1]- obstacles[j][1])*(Points[i][1]- obstacles[j][1]) - 
                   ((obstacles[j][2]/2)*(obstacles[j][2]/2));
                
            //negative so in (no need to see other obstacles)
            if(sign < 0) break;
        }
        // it's not free just skip and continue for other points 
        if(sign < 0 ) continue;
        freePoints.push_back(Points[i]);
    }
    return freePoints;
}


// Check intersection between edges and obstacles
bool checkEdges(double startPointX, double endPointX, double startPointY, double endPointY, vector<vector<float>> obstacles) {
    
    for (int i = 0; i < obstacles.size(); ++i) {
        //needed parameters to solve the parametric equation

        float cx = obstacles[i][0];  // circle center x
        float cy = obstacles[i][1];  // circle center y
        float r = obstacles[i][2] / 2;  // circle radius
        
        // Line segment from (startPointX, startPointY) to (endPointX, endPointY)
        float dx = endPointX - startPointX;
        float dy = endPointY - startPointY;
        float fx = startPointX - cx;
        float fy = startPointY - cy;
        
        // Quadratic equation coefficients for line-circle intersection
        float a = dx * dx + dy * dy;
        float b = 2 * (fx * dx + fy * dy);
        float c = (fx * fx + fy * fy) - r * r;
        
        float discriminant = b * b - 4 * a * c;
        
        // No intersection if discriminant < 0 (no real solution)
        if (discriminant < 0) continue;
        
        // Check if intersection points are within the line segment [0, 1]
        discriminant = sqrt(discriminant);
        float t1 = (-b - discriminant) / (2 * a);
        float t2 = (-b + discriminant) / (2 * a);
        
        // If either intersection point is within [0, 1], there's a collision (so on the line)
        if ((t1 >= 0 && t1 <= 1) || (t2 >= 0 && t2 <= 1)) {
            return false;  // Collision detected
        }
        
        // Also check if the entire line segment is inside the circle
        if (c < 0) return false;  // Start point is inside circle
    }
    
    return true;  // No collisions
}
//building undirected grapgh (visibilty one)
// each node stores {int, float} as (V,E) E->weight V->neighbour 
Graph buildGraph(vector<Node> freeNodes , vector<vector<float>> obstacles){
    Graph graph(freeNodes.size());
    
   for (int i = 0; i < freeNodes.size(); ++i) {
    for (int j = i + 1; j < freeNodes.size(); ++j) {
        float x1 = freeNodes[i].x;
        float y1 = freeNodes[i].y;
        float x2 = freeNodes[j].x;
        float y2 = freeNodes[j].y;

        if (checkEdges(x1, x2, y1, y2, obstacles)) {
            float dx = x1 - x2;
            float dy = y1 - y2;
            float dist = sqrt(dx * dx + dy * dy);

            graph[i].push_back({j, dist});
            graph[j].push_back({i, dist});
        }
    }
}
  return graph;
}


vector<int> YourChosenAlgorithm(vector<vector<float>> obstacles, vector<vector<float>> nodes, vector<vector<float>> &pathNodes, vector<vector<int>> &edges)
{
    // 1. Generate and filter points
    vector<vector<float>> Points = generatepoints(nodes[0][1], nodes[1][1], nodes[0][2], nodes[1][2]);
    vector<vector<float>> freePoints = FreePoints(obstacles, Points);

    // 2. Insert start & goal as {x, y}
    vector<vector<float>> StEnPoints = {
        {nodes[0][1], nodes[0][2]},  // Start
        {nodes[1][1], nodes[1][2]}   // Goal
    };
    freePoints.insert(freePoints.begin(), StEnPoints.begin(), StEnPoints.end());

    // 3. Wrap all free points as Nodes (to export to CSV files)
    vector<Node> freeNodes;
    for (int i = 0; i < freePoints.size(); ++i) {
        freeNodes.push_back(Node(i, freePoints[i][0], freePoints[i][1]));
    }

    // 4. Build visibility graph
    Graph graph = buildGraph(freeNodes, obstacles);

    // 5. Dijkstra
    int START_ID = 0;
    int GOAL_ID = 1;
    int n = freeNodes.size();

    vector<float> dist(n, INFINITY);
    vector<int> parent(n, -1);
    priority_queue<pair<float, int>, vector<pair<float, int>>, greater<>> pq;

    dist[START_ID] = 0;
    pq.push({0, START_ID});

    while (!pq.empty()) {
        auto [currDist, u] = pq.top(); pq.pop();

        if (u == GOAL_ID) break;

        for (auto& [v, weight] : graph[u]) {
            if (dist[v] > currDist + weight) {
                dist[v] = currDist + weight;
                parent[v] = u;
                pq.push({dist[v], v});
            }
        }
    }

    // 6. Reconstruct path
    vector<int> path;
    for (int v = GOAL_ID; v != -1; v = parent[v])
        path.push_back(v);
    reverse(path.begin(), path.end());

    for (int i = 0; i < path.size(); ++i) {
        if (i != path.size() - 1) {
            edges.push_back({path[i], path[i + 1]});
        }
        pathNodes.push_back( { ((float)freeNodes[path[i]].id),
            freeNodes[path[i]].x,
            freeNodes[path[i]].y } );
    }

    return path;
}

/* You should implement the algorithm you decided to use after conducting your own research over the problem statement. Please remember to do the following:
1- Comment your code.
2- Keep your code modular.
3- Keep in mind memory efficiency.
4- Use the the `pathNodes` variable to store the nodes generated by your algorithm in the form of
id,x,y
5- Use the `edges` variable to store the connections/edges between your nodes.
6- This task is designed for a specific set of Path-Planning algorithms. If your chosen implementation REQUIRES changes in the task template, you are allowed to change ONLY if you provide explanation for all the changes you needed to make and why.
*/