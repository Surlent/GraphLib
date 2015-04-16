#ifndef GRAPHWRITER_H_INCLUDED
#define GRAPHWRITER_H_INCLUDED

#include <auxiliary.h>
#include <Graph.h>

template<class T> class GraphWriter
{
    public:
         GraphWriter(Graph<T>* graph)
        {
            this->graph=graph;
        }
        void SetGraph(Graph<T>* graph)
        {
            this->graph=graph;
        }
        Graph<T> GetGraph()
        {
            return this->graph;
        }
        void NeighbourhoodOutput(ostream& output,T vertex)
        {
            vertex--;
            vector<T> neighbourhood=this->graph->Neighbourhood(vertex);
            for (T neighbour:neighbourhood)
            {
                output<<neighbour+1<<" ";
            }
        }
        void BFSOutput(ostream& output,T sourceIndex)
        {

            vector<int> parents,levels;
            tie(std::ignore,parents,levels)=this->graph->BFS(sourceIndex);
            int n=this->graph->N();
            for (int i=0;i<n;i++)
            {
                output<<(i+1)<<" "<<parents[i]+1<<" "<<levels[i]<<"\n";
            }

        }
        void BFSOutput(ostream& output,T sourceIndex,T endIndex)
        {

            vector<int> parents,levels;
            tie(std::ignore,parents,levels)=this->graph->BFS(sourceIndex,endIndex);
            int n=this->graph->N();
            for (int i=0;i<n;i++)
            {
                output<<(i+1)<<" "<<parents[i]+1<<" "<<levels[i]<<"\n";
            }

        }
        void DFSOutput(ostream& output,T sourceIndex)
        {

            vector<int> parents,levels;
            tie(std::ignore,parents,levels)=this->graph->DFS(sourceIndex);
            int n=this->graph->N();
            for (int i=0;i<n;i++)
            {
                output<<(i+1)<<" "<<parents[i]+1<<" "<<levels[i]<<"\n";
            }

        }
        void DFSOutput(ostream& output,T sourceIndex,T endIndex)
        {

            vector<int> parents,levels;
            tie(std::ignore,parents,levels)=this->graph->DFS(sourceIndex,endIndex);
            int n=this->graph->N();
            for (int i=0;i<n;i++)
            {
                output<<(i+1)<<" "<<parents[i]+1<<" "<<levels[i]<<"\n";
            }

        }

        void ConnectedComponentsOutput(ostream& output)
        {
            vector<vector<T>> connectedComponents;
            tie(std::ignore,connectedComponents)=this->graph->ConnectedComponents();
            sort(connectedComponents.begin(),connectedComponents.end(),BiggerIntVector);


            int componentsSize=connectedComponents.size();
            output<<"Graph has "<<componentsSize<<" components"<<"\n";
            for (int i=0;i<componentsSize;i++)
            {
                int currentComponentSize=connectedComponents[i].size();
                output<<"Component "<<i+1<<" - Size "<<currentComponentSize<<"\n";
                for (int j=0;j<currentComponentSize;j++)
                {
                    output << connectedComponents[i][j]+1<<"\n";
                }
                output<<"\n";
            }

        }
        void DiameterOutput(ostream& output,T maxIterations,T firstSource)
        {
            int diameter;
            std::pair<T,T> farthestPair;
            tie(diameter,farthestPair)=this->graph->Diameter(maxIterations,firstSource);

            output<<diameter<<" "<<farthestPair.first+1<<" "<<farthestPair.second+1<<"\n";

        }
        void DistanceOutput(ostream& output,T sourceIndex)
        {
            int n=this->graph->N();
            auto distancePackage=this->graph->Distance(sourceIndex);
            vector<int> parents=std::get<0>(distancePackage);
            auto distances=std::get<1>(distancePackage);


            for(int i=0;i<n;i++)
            {
                output<<i+1<<" "<<parents[i]+1<<" "<<distances[i]<<endl;
            }

        }
        void DistanceOutput(ostream& output,T sourceIndex,T endIndex)
        {
            double distance=this->graph->Distance(sourceIndex,endIndex);


            output<<sourceIndex<<" "<<endIndex<<" "<<distance;

        }

        void ShortestPathOutput(ostream& output,T sourceIndex)
        {
            int n=this->graph->N();
            vector<vector<T>> shortestPath=this->graph->ShortestPath(sourceIndex);


            for (int i=0;i<n;i++)
            {
                for(T vertex: shortestPath[i])
                {
                    output<<vertex+1<<" ";
                }
                output<<endl;
            }

        }
        void ShortestPathOutput(ostream& output,T sourceIndex,T endIndex)
        {
            vector<int> shortestPath=this->graph->ShortestPath(sourceIndex,endIndex);

            for (T vertex:shortestPath)
            {
             output<<vertex+1<<" ";
            }
        }
        void MSTOutput(ostream& output)
        {
            MSTOutput(output,1);
        }
        void MSTOutput(ostream& output,T sourceIndex)
        {
            vector<T> parents;
            double totalCost;
            tie(parents,totalCost)=this->graph->MST(sourceIndex);
            int n=this->graph->N();

            ostringstream oss;
            int mstSize=1; // Counts source index, which is not accounted for in loop below
            for(T i=0;i<n;i++)
            {
                if(parents[i]!=-1)
                {
                    oss<<parents[i]+1<<" "<<i+1<<endl;
                    mstSize++;
                }
            }
            output<<"Cost: "<<totalCost<<endl;
            output<<"MST Size: "<<mstSize<<endl;
            output<<n<<endl;
            output<<oss.str();

        }
        void MeanDistanceOutput(ostream& output)
        {
            double meanDistance=this->graph->MeanDistance();
            output<<meanDistance<<endl;

        }

        // Prints a report of the graph to a file
        void ReportOutput(ostream& output)
        {
            output<<"#n: "<<this->graph->N()<<endl;
            output<<"#m: "<<this->graph->M()<<endl;
            unordered_map<int,int> degreeFrequency;
            unordered_map<int,int>::const_iterator iter;
            double avg;
            int highestDegree;
            tie(degreeFrequency,avg,highestDegree)=this->graph->DegreeDistribution();
            output<<"# d_medio: "<<avg<<endl;
            for (int i=1;i<=highestDegree;i++)
            {
               if (degreeFrequency.find(i)!=degreeFrequency.end())
               {
                 output<<i<<" "<<(double)degreeFrequency[i]/(double)(this->graph->N())<<endl;
               }
               else
               {}
            }
        }
        // Writes a representation of the graph to a file
        void GraphOutput(ostream& output)
        {

            output<<this->graph->ToString();

        }
    private:
        Graph<T> *graph;
};

#endif
