#ifndef GRAPH_H_INCLUDED
#define GRAPH_H_INCLUDED

#include <auxiliary.h>
#include <GraphStructure.h>

template<class T> class Graph
{
    private:
        GraphStructure<T> *adj;
        int n,m;
        vector<int> degrees;
        bool weighted,negativelyWeighted;
    public:
         Graph(GraphStructure<T>* graphStructure)
        {
            this->adj=graphStructure;
            this->n=adj->N();
            this->m=adj->M();
            this->degrees=adj->Degrees();
            this->weighted=adj->IsWeighted();
            this->negativelyWeighted=adj->IsNegativelyWeighted();
        }
        int N()
        {
            return this->n;
        }
        int M()
        {
            return this->m;
        }
        vector<int> Degrees()
        {
            return this->degrees;
        }
        bool IsWeighted()
        {
            return weighted;
        }
        bool IsNegativelyWeighted()
        {
            return negativelyWeighted;
        }
        // Returns the neighbourhood of vertex v
         vector<T> Neighbourhood(T v)
        {
            //int* neighbourhood=new int[degrees[v]];
            //vector<int> r;
            return adj->Neighbourhood(v);
        }

        // Returns the average of the degrees
         float AverageDegree()
        {
            float avg=0;
            for (int i=0;i<n;i++)
            {
                avg+=degrees[i];
            }
            return avg/n;
        }

        // Returns the distribution of degrees
         string DegreeDistribution()
        {
            int highestDegree=0;
            unordered_map<int,int> degreeFrequency;
            unordered_map<int,int>::const_iterator iter;
            float avg=0;
            for (int i=0;i<n;i++)
            {
                if (degrees[i]>highestDegree)
                {
                    highestDegree=degrees[i];
                }
                avg+=degrees[i];
                if ((degreeFrequency.find(degrees[i]))!=degreeFrequency.end())
                {
                    degreeFrequency[degrees[i]]++;
                }
                else
                {
                    degreeFrequency[degrees[i]]=1;
                }
            }
            avg/=n;
            string output;
            output+=("# d_medio: ")+(ConvertToString(avg)+"\n");
            for (int i=1;i<=highestDegree;i++)
            {
               if (degreeFrequency.find(i)!=degreeFrequency.end())
               {
                 output+=ConvertToString(i)+" "+ConvertToString((float)degreeFrequency[i]/(float)n)+"\n";
               }
               else
               {
                 //output+=ConvertToString(i)+" 0\n";
               }
            }
            return output;
        }

         string ToString()
        {
            return adj->ToString();
        }

        // Breadth-first search from a vertex
         auto BFS(T sourceIndex)
        {
            return BFS(sourceIndex,(T)(this->n+1));
        }
        // Breadth-first search between two vertices
         auto BFS(T sourceIndex,T endIndex)
        {
            sourceIndex--;
            endIndex--;
            vector<bool> checked(n,false);
            vector<T> parents(n,-1);
            vector<int> levels(n,std::numeric_limits<float>::infinity());
            queue<T> discovered;
            parents[sourceIndex]=-1;
            levels[sourceIndex]=0;
            checked[sourceIndex]=true;
            discovered.push(sourceIndex);
            int currentLevel;
            T currentVertex;
            vector<T> currentNeighbourhood;
            bool endFound=false;
            while ((!discovered.empty())&&(!endFound))
            {
                currentVertex=discovered.front();
                currentLevel=levels[currentVertex]+1;
                discovered.pop();
                currentNeighbourhood=this->Neighbourhood(currentVertex);
                for (T w:currentNeighbourhood)
                {
                    if (!checked[w])
                    {
                        parents[w]=currentVertex;
                        levels[w]=currentLevel;
                        checked[w]=true;
                        discovered.push(w);
                        if (w==endIndex)
                        {
                            endFound=true;
                        }
                    }
                }
            }
            return make_tuple(checked,parents,levels);
        }

        // Depth-first search from a vertex
        auto DFS(T sourceIndex)
        {
            return DFS(sourceIndex,(T)(this->n+1));
        }
        // Depth-first search between two vertices
         auto DFS(T sourceIndex,T endIndex)
        {
            sourceIndex--;
            vector<bool> checked(n,false);
            vector<T> parents(n,-1);
            vector<int> levels(n,std::numeric_limits<float>::infinity());
            vector<bool> explored(n,0);
            stack<T> discovered;
            parents[sourceIndex]=-1;
            levels[sourceIndex]=0;
            explored[sourceIndex]=true;
            discovered.push(sourceIndex);
            int currentLevel;
            T currentVertex;
            vector<T> currentNeighbourhood;
            bool endFound=false;
            while (!discovered.empty()&&!endFound)
            {
                currentVertex=discovered.top();
                currentLevel=levels[currentVertex]+1;
                discovered.pop();
                if (!checked[currentVertex])
                {
                    checked[currentVertex]=true;
                    currentNeighbourhood=this->Neighbourhood(currentVertex);
                    for (T w:currentNeighbourhood)
                    {
                        if (!explored[w])
                        {
                            parents[w]=currentVertex;
                            levels[w]=currentLevel;
                            explored[w]=true;
                            if (w==endIndex)
                            {
                                endFound=true;
                                break;
                            }
                        }
                        discovered.push(w);
                    }
                }
            }
            return make_tuple(checked,parents,levels);
        }


         auto ConnectedComponents()
        {
            vector<vector<T>> connectedComponents;
            vector<bool> checked(n,false);
            unordered_map<T,bool> unchecked;
            for (int i=0;i<n;i++)
            {
                unchecked[i]=true;
            }

            T currentSource;
            //int componentsIndex=0;
            while (!unchecked.empty())
            {
                currentSource=unchecked.begin()->first;
                queue<T> discovered;
                //cout<<componentsIndex<<"\n";
                //vector<int> w=connectedComponents[componentsIndex];
                //cout<<connectedComponents[componentsIndex][0]<<"\n";
                vector<T> newComponent;
                newComponent.push_back(currentSource);
                connectedComponents.push_back(newComponent);
                unchecked.erase(unchecked.find(currentSource));
                checked[currentSource]=true;
                discovered.push(currentSource);
                T currentVertex;
                vector<T> currentNeighbourhood;
                while (!discovered.empty())
                {
                    currentVertex=discovered.front();
                    discovered.pop();
                    currentNeighbourhood=this->Neighbourhood(currentVertex);
                    for (T w:currentNeighbourhood)
                    {
                        if (!checked[w])
                        {
                            checked[w]=true;
                            connectedComponents.back().push_back(w);
                            unchecked.erase(w);
                            discovered.push(w);
                        }
                    }
                }
            }
            return make_tuple(checked,connectedComponents);
        }

         auto Diameter(T maxIterations,T firstSource)
        {
            maxIterations=min(maxIterations,(T)n);
            typename unordered_map<T,bool>::iterator uncheckedIterator;
            vector<int> levels(n,std::numeric_limits<float>::infinity());
            T currentSource=firstSource;
            int currentLevel=0;
            int currentDiameter=0;
            std::pair<T,T> farthestPair(currentSource,currentSource);
            for (int i=0;i<maxIterations;i++)
            {
                vector<bool> checked(n,false);
                currentSource=farthestPair.second;
                levels[currentSource]=0;
                queue<T> discovered;
                checked[currentSource]=true;
                discovered.push(currentSource);
                T currentVertex;
                vector<T> currentNeighbourhood;
                while (!discovered.empty())
                {
                    currentVertex=discovered.front();
                    currentLevel=levels[currentVertex]+1;
                    discovered.pop();
                    currentNeighbourhood=this->Neighbourhood(currentVertex);
                    for (T w:currentNeighbourhood)
                    {
                        if (!checked[w])
                        {
                            levels[w]=currentLevel;
                            checked[w]=true;
                            discovered.push(w);
                            if (currentLevel>currentDiameter)
                            {
                                farthestPair.first=currentSource;
                                farthestPair.second=w;
                                currentDiameter=currentLevel;
                            }
                        }
                    }
                }

            }
            return make_tuple(currentDiameter,farthestPair);
        }
        auto Dijkstra(T sourceIndex)
        {
            return Dijkstra(sourceIndex,(T)(this->n+1));
        }
        auto Dijkstra(T sourceIndex,T endIndex)
        {
            sourceIndex--;
            endIndex--;
            T currentVertex=sourceIndex;
            int currentLevel=0;
            double currentDistance=0;
            pair<T,double> top;
            vector<bool> checked(n,false);
            vector<int> levels(n,std::numeric_limits<float>::infinity());
            vector<double> distances(n,std::numeric_limits<float>::infinity());
            vector<T> parents(n,-1);
            vector<T> currentNeighbourhood;
            unordered_map<pair<T,T>,double> weights=this->adj->Weights();
            priority_queue<pair<T,double>,vector<pair<T,double>>,SmallerValuePair<T,double>> distancesQueue;
            checked[sourceIndex]=true;
            levels[sourceIndex]=currentLevel;
            distances[sourceIndex]=0;
            distancesQueue.push(make_pair(sourceIndex,distances[sourceIndex]));

            while (!distancesQueue.empty())
            {
                top =distancesQueue.top();
                currentVertex=top.first;
                currentDistance=top.second;
                distancesQueue.pop();
                if (currentDistance<=distances[currentVertex])
                {
                    checked[currentVertex]=true;
                    currentLevel=levels[currentVertex]+1;
                    currentNeighbourhood=this->Neighbourhood(currentVertex);
                    for (T w:currentNeighbourhood)
                    {
                        if (!checked[w])
                        {
                            pair<T,T> currentPair=make_pair(currentVertex,w);
                            double newDistance=distances[currentVertex]+weights[currentPair];
                            if (distances[w]>newDistance)
                            {
                                distances[w]=newDistance;
                                parents[w]=currentVertex;
                                levels[w]=currentLevel;
                                distancesQueue.push(make_pair(w,distances[w]));
                            }
                        }
                    }
                }
            }
            return make_tuple(checked,parents,levels,distances);
        }
        // Tracks distanceSum and pathCount in order to calculate mean distance
        void Dijkstra(T sourceIndex,double &distanceSum,long long int &pathCount,vector<bool> checked,double infinity,vector<double> distances)
        {
            Dijkstra(sourceIndex,(T)(this->n+1),distanceSum,pathCount,checked,infinity,distances);
        }
        void Dijkstra(T sourceIndex,T endIndex,double &distanceSum,long long int &pathCount,vector<bool> checked,double infinity,vector<double> distances)
        {
            sourceIndex--;
            endIndex--;
            T currentVertex=sourceIndex;
            double currentDistance;
            //vector<bool> checked(n,false);
            //double infinity=std::numeric_limits<double>::infinity();
            //vector<double> distances(n,infinity);
            unordered_map<pair<T,T>,double> weights=this->adj->Weights();
            priority_queue<pair<T,double>,vector<pair<T,double>>,SmallerValuePair<T,double>> distancesQueue;
            checked[sourceIndex]=true;
            distances[sourceIndex]=0;
            distancesQueue.push(make_pair(sourceIndex,distances[sourceIndex]));

            while (!distancesQueue.empty())
            {
                pair<T,double> top =distancesQueue.top();
                currentVertex=top.first;
                currentDistance=top.second;
                distancesQueue.pop();
                if (currentDistance<=distances[currentVertex])
                {
                    if((currentDistance!=infinity)&&(currentVertex!=sourceIndex))
                    {
                        distanceSum+=currentDistance;
                        pathCount++;
                    }
                    checked[currentVertex]=true;
                    vector<T> currentNeighbourhood=this->Neighbourhood(currentVertex);
                    for (T w:currentNeighbourhood)
                    {
                        if (!checked[w])
                        {
                            pair<T,T> currentPair=make_pair(currentVertex,w);
                            double edgeWeight=(this->weighted)?(weights[currentPair]):(1.0);
                            double newDistance=distances[currentVertex]+edgeWeight;
                            if (distances[w]>newDistance)
                            {
                                distances[w]=newDistance;
                                distancesQueue.push(make_pair(w,distances[w]));
                            }
                        }
                    }
                }
            }
        }
        // Find the distance from a given vertex to every other vertex
        auto Distance(T sourceIndex)
        {
            vector<T> parents;
            vector<double> distances;
            if (!this->weighted)
            {
                auto BFSTuple=this->BFS(sourceIndex);
                parents=std::get<1>(BFSTuple);
                vector<int> levels=std::get<2>(BFSTuple);
                distances.assign(levels.begin(),levels.end());
            }
            else
            {
                auto DijkstraTuple=this->Dijkstra(sourceIndex);
                parents=std::get<1>(DijkstraTuple);
                distances=std::get<3>(DijkstraTuple);
            }
            return make_tuple(parents,distances);
        }

        // Find the distance from sourceIndex to endIndex
        auto Distance(T sourceIndex, T endIndex)
        {
            vector<T> parents;
            vector<double> distances;
            if (!this->weighted)
            {
                auto BFSTuple=this->BFS(sourceIndex,endIndex);
                parents=std::get<1>(BFSTuple);
                vector<int> levels=std::get<2>(BFSTuple);
                distances.assign(levels.begin(),levels.end());
            }
            else
            {
                auto DijkstraTuple=this->Dijkstra(sourceIndex,endIndex);
                parents=std::get<1>(DijkstraTuple);
                distances=std::get<3>(DijkstraTuple);
            }
            return distances[endIndex-1];
        }
        auto MeanDistance()
        {
            long long int pathCount=0;
            vector<double> distances;
            double distanceSum=0;
            vector<bool> defaultChecked(n,false);
            double defaultInfinity=std::numeric_limits<double>::infinity();
            vector<double> defaultDistances(n,defaultInfinity);
            double start = omp_get_wtime( );
            double time;
            T currentSource;

            #pragma omp parallel for default(shared) reduction(+:distanceSum,pathCount)
            for(currentSource=0;currentSource<this->n;currentSource++)
            {
                if(currentSource%100==0)
                {
                    time= omp_get_wtime( )-start;
                    cout<<currentSource<<" "<<time<<" "<<distanceSum<<" "<<pathCount<<endl;

                }
                Dijkstra(currentSource+1,distanceSum,pathCount,defaultChecked,defaultInfinity,defaultDistances);
                /*for(int i=0;i<this->n;i++)
                {
                    if((distances[i]!=infinity)&&(i!=currentSource))
                    {
                        meanDistance+=distances[i];
                        pathCount++;
                    }
                }*/
            }
            return distanceSum/(pathCount);
        }
        // Find the shortest path from a given vertex to every other vertex
        vector<vector<T>> ShortestPath(T sourceIndex)
        {
            vector<T> parents;
            vector<int> levels;
            vector<vector<T>> shortestPath;
            shortestPath.resize(this->n);
            if (!this->weighted)
            {
                tie(std::ignore,parents,levels)=this->BFS(sourceIndex);
            }
            else
            {
                tie(std::ignore,parents,levels,std::ignore)=this->Dijkstra(sourceIndex);
            }
            T sourceIndexZB=sourceIndex-1; // 0-based source index
            T w;
            for (T i=0;i<this->n;i++)
            {
                //shortestPath[i].resize(levels[i]);
                w=i;
                while (parents[w]!=parents[sourceIndexZB])
                {
                    shortestPath[i].push_back(w);
                    w=parents[w];
                }
                shortestPath[i].push_back(sourceIndexZB);
                std::reverse(shortestPath[i].begin(),shortestPath[i].end());
            }

            return shortestPath;
        }

        // Find the shortest path from sourceIndex to endIndex
        vector<T> ShortestPath(T sourceIndex, T endIndex)
        {
            vector<T> parents;
            vector<int> levels;
            vector<int> shortestPath;

            if (!this->weighted)
            {
                tie(std::ignore,parents,levels)=this->BFS(sourceIndex,endIndex);
            }
            else
            {
                tie(std::ignore,parents,levels,std::ignore)=this->Dijkstra(sourceIndex,endIndex);
            }
            T sourceIndexZB=sourceIndex-1; // 0-based source index
            T w=endIndex-1;

            while (parents[w]!=parents[sourceIndexZB])
            {
                shortestPath.push_back(w);
                w=parents[w];
            }
            shortestPath.push_back(sourceIndexZB);
            std::reverse(shortestPath.begin(),shortestPath.end());
            return shortestPath;
        }

        // Finds a minimum spanning tree starting from sourceIndex
        auto Prim(T sourceIndex)
        {
            return Prim(sourceIndex,(T)(this->n+1));
        }
        // Finds the minimum spanning tree between two vertices
        auto Prim(T sourceIndex,T endIndex)
        {
            sourceIndex--;
            endIndex--;
            T currentVertex=sourceIndex;
            int currentLevel=0;
            double currentCost=0;
            unordered_map<pair<T,T>,double> weights=this->adj->Weights();
            pair<T,double> top;
            vector<bool> checked(n,false);
            vector<int> levels(n,std::numeric_limits<float>::infinity());
            vector<double> costs(n,std::numeric_limits<float>::infinity());
            vector<T> parents(n,-1);
            vector<T> currentNeighbourhood;
            priority_queue<pair<T,double>,vector<pair<T,double>>,SmallerValuePair<T,double>> costsQueue;
            double totalCost=0;
            checked[sourceIndex]=true;
            levels[sourceIndex]=currentLevel;
            costs[sourceIndex]=0;
            costsQueue.push(make_pair(sourceIndex,costs[sourceIndex]));
            while (!costsQueue.empty())
            {
                top =costsQueue.top();
                currentVertex=top.first;
                currentCost=top.second;
                costsQueue.pop();
                if (currentCost<=costs[currentVertex])
                {
                    totalCost+=currentCost;
                    checked[currentVertex]=true;
                    currentLevel=levels[currentVertex]+1;
                    currentNeighbourhood=this->Neighbourhood(currentVertex);
                    for (T w:currentNeighbourhood)
                    {
                        if (!checked[w])
                        {
                            pair<T,T> currentPair=make_pair(currentVertex,w);
                            double newCost=(this->weighted)?(weights[currentPair]):(1);
                            if (costs[w]>newCost)
                            {
                                costs[w]=newCost;
                                parents[w]=currentVertex;
                                levels[w]=currentLevel;
                                costsQueue.push(make_pair(w,costs[w]));
                            }
                        }
                    }
                }
            }
            return make_tuple(checked,parents,levels,totalCost);
        }

        // Find the minimum spanning tree from a vertex 1 to every other vertex
        auto MST()
        {
            return MST(1);
        }
        // Find the minimum spanning tree from a given vertex to every other vertex
        auto MST(T sourceIndex)
        {
            vector<T> parents;
            vector<int> levels;
            double totalCost;
            tie(std::ignore,parents,levels,totalCost)=this->Prim(sourceIndex);

            return make_tuple(parents,totalCost);
        }
};
#endif // GRAPH_H_INCLUDED
