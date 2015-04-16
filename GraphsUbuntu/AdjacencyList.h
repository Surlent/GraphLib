#ifndef ADJACENCY_LIST_H_INCLUDED
#define ADJACENCY_LIST_H_INCLUDED

#include <GraphStructure.h>

template<typename T> class AdjacencyList:public GraphStructure<T>
{
    private:
        vector<vector<T>> adj;
    public:
        AdjacencyList(string filename)
        {
            ifstream input(filename);
            string line;
            input>>this->n;
            int i,j,k;
            k=0;
            double weight;
            bool weightedSet=false;
            this->degrees.resize(this->n);
            adj.resize(this->n);
            T currentNumber;
            //getline(input,line); // Dummy call, value is not used (already got first value as this->n above
            //getline(input,line); // Gets first line with a valid edge
            if(!weightedSet)
            {
                getline(input,line); // Dummy call
                getline(input,line); // First line
                istringstream iss(line);
                int numberCount=0;
                string number;
                while( iss >> number )
                {
                    ++numberCount;
                }
                iss.clear();
                iss.str(line);
                if(numberCount==2)
                {
                    this->weighted=false;
                    iss>>i>>j;
                    //cout<<"Graph is not weighted."<<endl;
                }
                else if(numberCount==3)
                {
                    iss>>i>>j>>weight;
                    this->weighted=true;
                    //cout<<"Graph is weighted."<<endl;
                }
                else
                {
                    cout<<"Invalid number count per line.("<<numberCount<<")"<<endl;
                }
                i--;
                j--;
                this->m++;
                adj[i].push_back(j);
                adj[j].push_back(i);
                this->degrees[i]++;
                this->degrees[j]++;
                if (this->weighted)
                {
                    if(weight<0)
                    {
                        this->negativelyWeighted=true;
                        cout<<"Negative weight detected!";
                    }
                    this->weights[std::make_pair(j,i)]=weight;
                    this->weights[std::make_pair(i,j)]=weight;
                }
                weightedSet=true;
            }
            while (input.good())
            {
                if (k%3==0) // First number in line
                {
                    input>>currentNumber;
                    i=currentNumber-1;
                }
                else if(k%3==1) // Second number in line
                {
                    input>>currentNumber;
                    this->m++;
                    j=currentNumber-1;
                    adj[i].push_back(j);
                    adj[j].push_back(i);
                    this->degrees[i]++;
                    this->degrees[j]++;
                    if(!this->weighted)
                    {
                        k++;
                    }
                }
                else // Third number in line
                {
                    input>>weight;
                    if(weight<0)
                    {
                            this->negativelyWeighted=true;
                    }
                    this->weights[std::make_pair(j,i)]=weight;
                    this->weights[std::make_pair(i,j)]=weight;
                }
                k++;
            }

            if (this->n<=0)
            {
                cout<<"No nodes found."<<endl;
            }
            else if (this->m<=0)
            {
                cout<<"No edges found."<<endl;
            }
        }

        // Returns the neighbourhood of vertex v
        vector<T> Neighbourhood(int v)
        {
            return adj[v];
        }

        // Returns a string representation of this List
        string ToString()
        {
            int i,j;
            string output="";
            ostringstream oss;
            oss<<this->n<<"\n";
            for(i = 0;i < this->n;i++)
            {
                for(j = 0; j < this->degrees[i]; j++)
                {
                    if(this->weighted)
                    {
                        oss<<i+1<<" "<<this->adj[i][j]+1<<" "<<this->weights[make_pair(i,adj[i][j])]<<"\n";
                    }
                    else
                    {
                        oss<<i+1<<" "<<this->adj[i][j]+1<<"\n";
                    }
                }
            }
            return oss.str();
        }


};
#endif

