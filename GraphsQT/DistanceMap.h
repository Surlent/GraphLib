#ifndef DISTANCE_MAP_H_INCLUDED
#define DISTANCE_MAP_H_INCLUDED

#include <GraphStructure.h>

template<typename T> class DistanceMap:public GraphStructure<T>
{
    private:
        vector<vector<T>> adj;
        vector<double> xCoordinates;
        vector<double> yCoordinates;
    public:
        DistanceMap(string filename)
        {
            ifstream input(filename);                     
            input>>this->n;
            int k,currentIndex;
            k=0;
            currentIndex=0;
            double x,y;                       
            this->degrees.resize(this->n);
            //this->adj.resize(this->n);
            this->xCoordinates.resize(this->n);
            this->yCoordinates.resize(this->n);
            this->m=0;
            this->weighted=true;
            while (input.good())
            {
                if (k%2==0) // First number in line
                {
                    input>>x;
                }
                else if(k%2==1) // Second number in line
                {
                    input>>y;
                    this->xCoordinates[currentIndex]=x;
                    this->yCoordinates[currentIndex]=y;
                    this->degrees[currentIndex]=0;
                    // Fills edgbes
                    /*for(int previousIndex=0;previousIndex<currentIndex;previousIndex++)
                    {
                        weight=Distance(make_pair(xCoordinates[currentIndex],yCoordinates[currentIndex]),make_pair(xCoordinates[currentIndex-1],yCoordinates[currentIndex-1]));
                        this->weights[make_pair(currentIndex,previousIndex)]=weight;
                        //this->weights[make_pair(previousIndex,currentIndex)]=weight;
                    }*/
                    currentIndex++;
                }
                k++;
            }
            if (this->n<=0)
            {
                cout<<"No nodes found."<<endl;
            }            
        }

        // Returns the neighbourhood of vertex v
        vector<T> Neighbourhood(const T &v)
        {
            return adj[v];
        }

        vector<double> XCoordinates()
        {
            return this->xCoordinates;
        }
        vector<double> YCoordinates()
        {
            return this->yCoordinates;
        }

        pair<double,double> Coordinates(const T &v)
        {
            return make_pair(this->xCoordinates[v],this->yCoordinates[v]);
        }

        void SetDegrees(T v,int value)
        {
            this->degrees[v]=value;
        }

        // Returns a string representation of this List
        string ToString()
        {
            int i,j;            
            ostringstream oss;
            oss<<this->n<<"\n";
            for(i = 0;i < this->n;i++)
            {
                for(j = 0; j < adj[i].size(); j++)
                {
                    if(this->weighted)
                    {
                        oss<<i+1<<" "<<this->adj[i][j]+1<<" "<<fixed<<this->weights[make_pair(i,adj[i][j])]<<"\n";
                    }
                    else
                    {
                        oss<<i+1<<" "<<this->adj[i][j]+1<<"\n";
                    }
                }
            }
            return oss.str();
        }
        double EdgeDistance(const T &aNode,const T &bNode)
        {
            return Distance(this->Coordinates(aNode),this->Coordinates(bNode));
        }

        double DistanceToSegment(const T &pNode,const pair<T,T> &lEdge) {
          pair<double,double> p=this->Coordinates(pNode);
          pair<pair<double,double>,pair<double,double>> l=make_pair(this->Coordinates(lEdge.first),this->Coordinates(lEdge.second));
          // Return minimum distance between line segment l and point p
          pair<double,double> segmentA=l.first; // first end of segment
          pair<double,double> segmentB=l.second; // last end of segment
          const double distSquared = DistanceSquared<double>(segmentA, segmentB);  // i.e. |w-v|^2 -  avoid a sqrt
          if (distSquared == 0.0) return Distance<double>(p, segmentA);   // v == w case
          // Consider the line extending the segment, parameterized as v + t (w - v).
          // We find projection of point p onto the line.
          // It falls where t = [(p-v) . (w-v)] / |w-v|^2
          const double t = DotProduct<double>(Subtract<double>(p,segmentA), Subtract<double>(segmentB,segmentA)) / distSquared;
          if (t < 0.0) return Distance<double>(p, segmentA);       // Beyond the 'v' end of the segment
          else if (t > 1.0) return Distance<double>(p, segmentB);  // Beyond the 'w' end of the segment
          pair<double,double> projection = Add<double>(segmentA, MultiplyByConstant<double>(t,Subtract<double>(segmentB,segmentA)));  // Projection falls on the segment
          return Distance(p, projection);
        }

        void SetMinimumPath(const T &p, const pair<T,T> &edge,list<T> &path,unordered_map<pair<T,T>,bool> &chosenEdges,unordered_map<pair<T,T>,bool> &uncheckedEdges,unordered_map<T,bool> &unchecked,auto &distancesQueue,double &pathLength)
        {
            T a=edge.first;
            T b=edge.second;
            pair<double,double> coordinatesP=Coordinates(p);
            pair<double,double> coordinatesA=Coordinates(a);
            pair<double,double> coordinatesB=Coordinates(b);
            double pathLength1,pathLength2,pathLength3,minimumPath;
            if((this->degrees[a]==1)&&(this->degrees[b]==1))
            {
                pathLength1=Distance<double>(coordinatesA,coordinatesB)+Distance<double>(coordinatesB,coordinatesP);
                pathLength2=Distance<double>(coordinatesA,coordinatesP)+Distance<double>(coordinatesB,coordinatesP);
                pathLength3=Distance<double>(coordinatesA,coordinatesB)+Distance<double>(coordinatesA,coordinatesP);
            }
            else if(this->degrees[a]==2&&this->degrees[b]==1)
            {
                pathLength1=Distance<double>(coordinatesA,coordinatesB)+Distance<double>(coordinatesB,coordinatesP);
                pathLength2=Distance<double>(coordinatesA,coordinatesP)+Distance<double>(coordinatesB,coordinatesP);
                pathLength3=std::numeric_limits<double>::infinity();
            }            
            else if(this->degrees[a]==1&&this->degrees[b]==2)
            {
                pathLength1=std::numeric_limits<double>::infinity();
                pathLength2=Distance<double>(coordinatesA,coordinatesP)+Distance<double>(coordinatesB,coordinatesP);
                pathLength3=Distance<double>(coordinatesA,coordinatesB)+Distance<double>(coordinatesA,coordinatesP);
            }
            else if(this->degrees[a]==2&&this->degrees[b]==2)
            {
                pathLength1=std::numeric_limits<double>::infinity();
                pathLength2=Distance<double>(coordinatesA,coordinatesP)+Distance<double>(coordinatesB,coordinatesP);
                pathLength3=std::numeric_limits<double>::infinity();
            }
            else
            {
                cout<<"Warning: unexpected case for path."<<endl;
                cout<<"P: "<<p<<", Degree:"<<this->degrees[p]<<endl;
                cout<<"A: "<<a<<", Degree:"<<this->degrees[a]<<endl;
                cout<<"B: "<<b<<", Degree:"<<this->degrees[b]<<endl;
            }
            minimumPath=min({pathLength1,pathLength2,pathLength3});
            uncheckedEdges.clear();
            if(minimumPath==pathLength1)
            {
//               this->adj[b].push_back(p);
//               this->adj[p].push_back(b);
               this->degrees[b]++;
               this->degrees[p]++;               
               uncheckedEdges[make_pair(b,p)]=true;
               chosenEdges[make_pair(b,p)]=true;               
               pathLength+=this->EdgeDistance(b,p);
               auto bIterator=std::find(path.begin(),path.end(),b);
               auto beforeB=std::prev(bIterator);
               auto afterB=std::next(bIterator);
               if(beforeB==path.end())
               {
                    path.push_front(p);
               }
               else if(afterB==path.end())
               {
                    path.push_back(p);
               }
               else
               {
                   cout<<"Unexpected case for list iterator - Vertex "<<p<<" - B:"<<*(bIterator)<<",BNext:"<<*(afterB)<<",BPrev:"<<*beforeB<<endl;
               }
            }
            else if(minimumPath==pathLength2)
            {
                //this->adj[a].erase(std::find(this->adj[a].begin(),this->adj[a].end(),b));
                //this->adj[b].erase(std::find(this->adj[b].begin(),this->adj[b].end(),a));
//                this->adj[a].push_back(p);
//                this->adj[p].push_back(a);
//                this->adj[b].push_back(p);
//                this->adj[p].push_back(b);
                this->degrees[p]+=2;
                uncheckedEdges[make_pair(a,p)]=true;
                uncheckedEdges[make_pair(b,p)]=true;                
                chosenEdges[make_pair(a,p)]=true;
                chosenEdges[make_pair(b,p)]=true;
                chosenEdges.erase(chosenEdges.find(make_pair(a,b)));
                pathLength+=this->EdgeDistance(a,p);
                pathLength+=this->EdgeDistance(b,p);
                pathLength-=this->EdgeDistance(a,b);
                auto aIterator=std::find(path.begin(),path.end(),a);
                auto bIterator=std::find(path.begin(),path.end(),b);
                auto beforeA=std::prev(aIterator);
                auto beforeB=std::prev(bIterator);
                if(aIterator==beforeB)
                {
                     path.insert(bIterator,p);
                }
                else if(bIterator==beforeA)
                {
                     path.insert(aIterator,p);
                }
            }
            else if(minimumPath==pathLength3)
            {
//                this->adj[a].push_back(p);
//                this->adj[p].push_back(a);
                this->degrees[a]++;
                this->degrees[p]++;
                uncheckedEdges[make_pair(a,p)]=true;
                chosenEdges[make_pair(a,p)]=true;
                pathLength+=this->EdgeDistance(a,p);
                auto aIterator=std::find(path.begin(),path.end(),a);
                auto beforeA=std::prev(aIterator);
                auto afterA=std::next(aIterator);
                if(beforeA==path.end())
                {
                     path.push_front(p);
                }
                else if(afterA==path.end())
                {
                     path.push_back(p);
                }
                else
                {
                    cout<<"Unexpected case for list iterator - Vertex "<<p<<" - A:"<<*(aIterator)<<",Degree[A]:"<<this->degrees[*aIterator]<<",Degree[B]:"<<this->degrees[b]<<",ANext:"<<*(afterA)<<",APrev:"<<*beforeA<<endl;
                }
            }
            else
            {
                cout<<"Unexpected case for minimum path length."<<endl;
            }
            for(auto keyValue:unchecked)
            {
                T index=keyValue.first;
                for(auto keyValueEdge:uncheckedEdges)
                {
                  pair<T,T> newEdge=keyValueEdge.first;
                  //distancesQueue.push(make_pair(make_pair(index,newEdge),this->EdgeDistance(index,newEdge.first)+this->EdgeDistance(index,newEdge.second)));
                  distancesQueue.push(make_pair(make_pair(index,newEdge),this->DistanceToSegment(index,newEdge)));
                }
            }

        }

        auto TwoOptSwap( const vector<T> &path,const T &i, const T &k )
        {
            T size = path.size();
            vector<T> newPath;
            double newLength=0;

            // 1. take route[0] to route[i-1] and add them in order to new_route
            for ( T c = 0; c <= i - 1; ++c )
            {
                newPath.push_back(path[c]);
                if(newPath.size()>1)
                {
                  newLength+=this->EdgeDistance(newPath[newPath.size()-1],newPath[newPath.size()-2]);
                }
            }

            // 2. take route[i] to route[k] and add them in reverse order to new_route
            for ( T c = k; c >= i; --c )
            {
                newPath.push_back(path[c]);
                if(newPath.size()>1)
                {
                  newLength+=this->EdgeDistance(newPath[newPath.size()-1],newPath[newPath.size()-2]);
                }
            }
            // 3. take route[k+1] to end and add them in order to new_route
            for ( T c = k + 1; c < size; ++c )
            {
                newPath.push_back(path[c]);
                if(newPath.size()>1)
                {
                  newLength+=this->EdgeDistance(newPath[newPath.size()-1],newPath[newPath.size()-2]);
                }
            }            
            return make_tuple(newPath,newLength);
        }

        void TwoOpt(const int &nIterations,vector<T> &path,double &pathLength)
        {
            // Get tour size
            T size = path.size();

            // repeat until no improvement is made
            int improve = 0;
            clock_t initialTime=clock();
            clock_t lastOutTime=0;
            while ( improve < nIterations )
            {

                for ( T i = 1; i < size - 1; i++ )
                {
                        for ( T k = i + 1; k < size-1; k++)
                        {
                                vector<T> newPath;
                                double newLength;
                                tie(newPath,newLength)=this->TwoOptSwap( path,i, k );                                
                                if ( newLength < pathLength )
                                {
                                    // Improvement found so reset
                                    improve = 0;
                                    path=newPath;
                                    pathLength = newLength;
                                }
                                clock_t lastInterval=clock()-lastOutTime;
                                clock_t currentTime=clock()-initialTime;
                                if((double)lastInterval/CLOCKS_PER_SEC>=120)
                                {
                                    ofstream output("outputMid.txt");
                                    for(auto p:path)
                                    {
                                        output<<p+1<<" ";
                                    }
                                    output<<endl;
                                    output<<pathLength<<endl;
                                    output<<"Elapsed:"<<(double)currentTime/CLOCKS_PER_SEC<<endl;
                                    lastOutTime=currentTime;
                                }
                        }
                }

                improve ++;
            }
        }
        auto TSP(T firstA,T firstB)
        {
            list<T> path;
            path.push_back(firstA);
            path.push_back(firstB);
            double pathLength;
            //vector<T> parents(this->n,-1);
            //vector<T> children(this->n,-1);
            priority_queue<pair<pair<T,pair<T,T>>,double>,deque<pair<pair<T,pair<T,T>>,double>>,SmallerValuePair<pair<T,pair<T,T>>,double>> distancesQueue;
            unordered_map<T,bool> unchecked;
            unchecked.rehash(this->n);
            unordered_map<pair<T,T>,bool> uncheckedEdges;
            unordered_map<pair<T,T>,bool> chosenEdges;            
            for (int i=0;i<this->n;i++)
            {
                unchecked[i]=true;
            }
            //this->adj[firstA].push_back(firstB);
            //this->adj[firstB].push_back(firstA);
            //children[firstA]=firstB;
            //parents[firstB]=firstA;
            //T currentA,currentB;
            T currentVertex;
            pair<T,T> currentEdge=make_pair(firstA,firstB);
            //pair<pair<double,double>,pair<double,double>> newestSegment=make_pair(Coordinates(firstA),Coordinates(firstB));
            uncheckedEdges[currentEdge]=true;
            chosenEdges[currentEdge]=true;
            unchecked.erase(unchecked.find(firstA));
            unchecked.erase(unchecked.find(firstB));
            this->degrees[firstA]++;
            this->degrees[firstB]++;
            pathLength=this->EdgeDistance(firstA,firstB);
            for(auto keyValue:unchecked)
            {
                T index=keyValue.first;
                for(auto keyValueEdge:uncheckedEdges)
                {
                  pair<T,T> edge=keyValueEdge.first;
                  //distancesQueue.push(make_pair(make_pair(index,edge),this->EdgeDistance(index,edge.first)+this->EdgeDistance(index,edge.second)));
                  distancesQueue.push(make_pair(make_pair(index,edge),this->DistanceToSegment(index,edge)));
                }
            }
            while (!unchecked.empty())
            {
                while((unchecked.find(currentVertex)==unchecked.end()||chosenEdges.find(currentEdge)==chosenEdges.end())&&(!distancesQueue.empty()))
                {
                        pair<pair<T,pair<T,T>>,double> top=distancesQueue.top();
                        top=distancesQueue.top();
                        currentVertex=top.first.first;
                        currentEdge=top.first.second;
                        distancesQueue.pop();
                }                
                //cout<<"Vertex "<<currentVertex<<" reached"<<endl;
                this->SetMinimumPath(currentVertex,currentEdge,path,chosenEdges,uncheckedEdges,unchecked,distancesQueue,pathLength);
                unchecked.erase(unchecked.find(currentVertex));
            }
            this->degrees[path.front()]++;
            this->degrees[path.back()]++;
            chosenEdges[make_pair(path.front(),path.back())]=true;
            pathLength+=this->EdgeDistance(path.front(),path.back());
            path.push_back(path.front());
            //T fixedVertex=path.front();
            //std::vector<T> pathVector{ path.begin(), path.end() }; // Convert from list to vector for 2-opt
            std::vector<T> pathVector{ std::make_move_iterator(std::begin(path)),std::make_move_iterator(std::end(path)) }; // Move list to vector for 2-opt
            this->TwoOpt(20,pathVector,pathLength);
            return make_tuple(pathVector,pathLength);
        }

};
#endif


