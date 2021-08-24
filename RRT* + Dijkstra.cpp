#include <bits/stdc++.h>
#include <cstdio>
#include <random>
#include <SFML/Graphics.hpp>
#include "geometry.h"

using namespace std ; 

const int Width = 800 ;
const int Height = 600 ;
const int Radius = 5 ; 
const double GOAL_SAMPLING_PROB = 0.05;
const double INF = 1e18;

const double JumpSize = (Width/100.0 * Height/100.0)/1.5;
const double DISK_SIZE = JumpSize ; 
int whichRRT = 3 ; 
vector < Polygon > obstacles ; 
Point start, stop ; 
int obstacle_cnt = 1 ;
vector < Point > nodes ; 
vector < int > parent, nearby ; 
vector < double > cost, jumps ; 
int nodeCnt = 0, goalIndex = -1 ; 
vector <sf::ConvexShape> polygons ;
sf::CircleShape startingPoint, endingPoint ; 
bool pathFound = 0 ;

template <typename T> 
T randomCoordinate(T low, T high){
    random_device random_device;
    mt19937 engine{random_device()};
    uniform_real_distribution<double> dist(low, high);     
    return dist(engine);
}

vector<Point> path2;
vector<double> dis;
vector<int> parent2;
sf :: Vertex line2[2];
int n;
int iterations = 0 ; 
int c;
int main() {

    auto getInput = [&](){
        cout << "Hello! Welcome to the demonstration" << endl;
        cout << "Please note : The screen available for the demonstration is " << Height << " * " << Width << " Pixels." << endl;
        cout << "            : Maximum Possible Jump is equal to " << JumpSize << " units." << endl;
        cout << "Please enter the co-ordinates of the Starting Point." << endl;
        cin >> start.x >> start.y;
        cout << "Please enter the co-ordinates of the Ending Point.";
        cin >> stop.x >> stop.y;
        cout << "Please enter the number of obstacles you wish to have in the system." << endl;
        cin >> obstacle_cnt;
        obstacles.resize(obstacle_cnt);
        int pnts = 0;
        Point pnt;
        vector<Point> poly;
        for(int i = 0; i < obstacle_cnt; i++){
            poly.clear();
            cout << "How many points in the obstacle #" << i + 1 << endl; 
            cin >> pnts;
            poly.resize(pnts);
            cout << "Enter the co-ordinates in clockwise order" << endl;
            for(int j = 0; j < pnts; j++){
                cin >> pnt.x >> pnt.y;
                obstacles[i].addPoint(pnt);
            }
        }
    };


    auto prepareInput = [&](){
        startingPoint.setRadius(Radius);
        startingPoint.setFillColor(sf::Color(208, 0, 240));
        startingPoint.setPosition(start.x, start.y);
        startingPoint.setOrigin(Radius / 2, Radius / 2);
        endingPoint.setRadius(Radius);
        endingPoint.setFillColor(sf::Color::Blue);
        endingPoint.setPosition(stop.x, stop.y);
        endingPoint.setOrigin(Radius / 2, Radius / 2);
        polygons.resize(obstacle_cnt);
        for(int i = 0; i < obstacle_cnt; i++){
            polygons[i].setPointCount(obstacles[i].pointCnt);
            polygons[i].setFillColor(sf :: Color(89, 87, 98));
            for(int j = 0; j < obstacles[i].pointCnt; j++){
                polygons[i].setPoint(j, sf :: Vector2f(obstacles[i].points[j].x, obstacles[i].points[j].y));
            }
        }
    };


    auto isEdgeObstacleFree = [&](Point a, Point b) {
        for(auto& poly: obstacles)
            if(lineSegmentIntersectsPolygon(a, b, poly))
                return false ; 
        return true ; 
    };
    auto draw = [&](sf::RenderWindow& window, int flag) {
        sf :: Vertex line[2];
        sf :: CircleShape nodeCircle;
        for(auto & poly : polygons)
            window.draw(poly);
        for(int i = (int)nodes.size() - 1; i; i--){
            Point par = nodes[parent[i]];
            line[0] = sf :: Vertex(sf :: Vector2f(par.x, par.y));
            line[1] = sf :: Vertex(sf :: Vector2f(nodes[i].x, nodes[i].y));
            window.draw(line, 2, sf :: Lines);
        }
        window.draw(startingPoint);
        window.draw(endingPoint);
        if(pathFound){
            int node = goalIndex;
            while(parent[node] != node){
                int par = parent[node];
                line[0] = sf :: Vertex(sf :: Vector2f(nodes[par].x, nodes[par].y));
                line[1] = sf :: Vertex(sf :: Vector2f(nodes[node].x, nodes[node].y));
                line[0].color = line[1].color = sf :: Color :: Red;
                window.draw(line, 2, sf :: Lines);
                node = par;
            }
            if(iterations < 500) goto here; 
            c = path2.size() - 1;
            while(c != parent2[c]){
                Point par = path2[parent2[c]];
                line2[0] = sf :: Vertex(sf :: Vector2f(par.x, par.y));        
                line2[1] = sf :: Vertex(sf :: Vector2f(path2[c].x, path2[c].y));
                line2[0].color = line2[1].color = sf :: Color :: Green;
                window.draw(line2, 2, sf :: Lines);
                c = parent2[c];
            }
            here :;
        }

    };

    auto pickRandomPoint = [&]() {
        double random_sample = randomCoordinate(0.0, 1.0); 
        if((random_sample - GOAL_SAMPLING_PROB) <= EPS and !pathFound) return stop + Point(Radius, Radius) ;
        return Point(randomCoordinate(0, Width), randomCoordinate(0, Height)); 
    };

    auto checkDestinationReached = [&]() {
        sf::Vector2f position = endingPoint.getPosition(); 
        if(checkCollision(nodes[parent[nodeCnt - 1]], nodes.back(), Point(position.x, position.y), Radius)) {
            pathFound = 1 ; 
            goalIndex = nodeCnt - 1;
            cout << "Reached!! With a distance of " << cost.back() << " units. " << endl << endl ; 
        }
    };

    auto rewire = [&]() {
        int lastInserted = nodeCnt - 1 ; 
        for(auto nodeIndex: nearby) {
            int par = lastInserted, cur = nodeIndex;
            while( ((cost[par] + distance(nodes[par], nodes[cur])) - cost[cur]) <= EPS) { 
                int oldParent = parent[cur] ;
                parent[cur] = par; 
                cost[cur] = cost[par] + distance(nodes[par], nodes[cur]);
                par = cur, cur = oldParent; 
            }
        }
    };

    auto RRT = [&](){
        Point newPoint, nearestPoint, nextPoint; 
        bool updated = false ; 
        int cnt = 0 ; 
        int nearestIndex = 0 ; 
        double minCost = INF; 
        nearby.clear(); 
        jumps.resize(nodeCnt); 
        while(!updated) {
            newPoint = pickRandomPoint(); 
            nearestPoint = *nodes.begin(); 
            nearestIndex = 0;
            for(int i = 0; i < nodeCnt; i++) {
                if(pathFound and randomCoordinate(0.0, 1.0) < 0.25) 
                    cost[i] = cost[parent[i]] + distance(nodes[parent[i]], nodes[i]);  
                jumps[i] = randomCoordinate(0.3, 1.0) * JumpSize ; 
                auto pnt = nodes[i] ; 
                if((pnt.distance(newPoint) - nearestPoint.distance(newPoint)) <= EPS and isEdgeObstacleFree(pnt, pnt.steer(newPoint, jumps[i])))
                    nearestPoint = pnt, nearestIndex = i ; 
            }
            nextPoint = stepNear(nearestPoint, newPoint, jumps[nearestIndex]);
            if(!isEdgeObstacleFree(nearestPoint, nextPoint)) continue ; 
            if( (whichRRT == 1) or (!pathFound and whichRRT == 3)) {
                updated = true ;
                nodes.push_back(nextPoint); nodeCnt++;
                parent.push_back(nearestIndex);
                cost.push_back(cost[nearestIndex] + distance(nearestPoint, nextPoint));
                if(!pathFound) checkDestinationReached();
                continue ; 
            }
            for(int i = 0; i < nodeCnt; i++)
                if((nodes[i].distance(nextPoint) - DISK_SIZE) <= EPS and isEdgeObstacleFree(nodes[i], nextPoint))
                    nearby.push_back(i);
            int par = nearestIndex; minCost = cost[par] + distance(nodes[par], nextPoint);
            for(auto nodeIndex: nearby) {
                if( ( (cost[nodeIndex] + distance(nodes[nodeIndex], nextPoint)) - minCost) <= EPS)
                    minCost = cost[nodeIndex] + distance(nodes[nodeIndex], nextPoint), par = nodeIndex;
            }
            parent.push_back(par); cost.push_back(minCost);
            nodes.push_back(nextPoint); nodeCnt++; 
            updated = true ; 
            if(!pathFound) checkDestinationReached(); 
            rewire();
        }
    };

    auto euc = [&](Point a, Point b){
       for(auto& poly: obstacles)
            if(lineSegmentIntersectsPolygon(a, b, poly))
                return -1.0 ; 
        return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
    };

    getInput();
    prepareInput(); 
    sf::RenderWindow window(sf::VideoMode(Width, Height), "Basic Anytime RRT");
    nodeCnt = 1;
    nodes.push_back(start); 

    parent.push_back(0); 
    cost.push_back(0);
    sf::Time delayTime = sf::milliseconds(1);
    cout << endl << "Starting node is in Pink and Destination node is in Blue" << endl << endl ; 
    while (iterations < 3000)
    {
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
            {
                window.close();
                return 0; exit(0);
            }
        }
        RRT(); 
        iterations++;
        if(iterations % 500 == 0) {
            cout << "Iterations: " << iterations << endl ; 
            if(!pathFound) cout << "Not reached yet :( " << endl ;
            else cout << "Shortest distance till now: " << cost[goalIndex] << " units." << endl ;
            cout << endl ;
        }
        if(iterations % 500 == 0){
            
            path2.clear();
            int curr = goalIndex;
            int cnt = 0;
            while(parent[curr] != curr){
                int par = parent[curr];
                path2.push_back(nodes[curr]);
                curr = par;
            }
            path2.push_back(start);

            // making a strongly connected graph from the nodes above
            int n = (int)path2.size();
            vector<vector<double>> adj(n, vector<double> (n));
            for(int i = 0; i < n; i++){
                for(int j = 0; j < n; j++){
                    adj[i][j] = adj[j][i] = euc(path2[i], path2[j]);
                }
            }

            // // djikstra

            dis = vector<double> (n,(double)INT_MAX);
            parent2 = vector<int> (n);
            dis[0] = 0.0;
            priority_queue<pair<double, int>> q;
            q.push({0.0 ,  0});
            cnt = 0;
            while(!q.empty()){
                auto top = q.top();
                q.pop();
                double d = -top.first;
                int    u = top.second;
                if(dis[u] != d) continue;
                for(int v = 0; v < n; v++){
                    if(v != u && adj[u][v] != -1){
                        if(dis[v] > d + adj[u][v]){
                            dis[v] = d + adj[u][v];
                            parent2[v] = u;
                            q.push({-dis[v], v});
                        }
                    }
                }
                
            }
            cout << dis[n - 1] << endl;
            // getting the final path
            // DIDN'T DRAW THE STARTING AND ENDING POINT AS OF NOW
           
            window.clear();
            draw(window, 1); 
            window.display();

        }
        else{
            window.clear();
            draw(window, 0); 
            window.display();
        }
    }
    // window.clear();
    // storing the nodes of the best path found so far

    
    // draw(window); 
    // window.display();
    // 
}


