#ifndef ADJACENCY_MATRIX_H_INCLUDED
#define ADJACENCY_MATRIX_H_INCLUDED

#include <GraphStructure.h>
template<typename T,typename T2> class AdjacencyMatrix:public GraphStructure<T2>
{
    private:
        T **adj;
    public:
        AdjacencyMatrix(string filename)
        {
            ifstream input(filename);
            input>>this->n;
            int i,j,k;
            this->degrees=new int[this->n];
            adj=new T* [this->n];
            for (k = 0; k < this->n; k++)
            {
                adj[k] = new T[this->n];
            }
            k=0;
            T2 currentNumber;
            while (input>>currentNumber)
            {
                if (k%3==0)
                {
                    i=currentNumber-1;
                }
                else if(k%3==1)
                {
                    this->m++;
                    j=currentNumber-1;
                    adj[i][j]=1;
                    adj[j][i]=1;
                    this->degrees[i]++;
                    this->degrees[j]++;
                }
                else
                {
                    this->weights[i][j]=currentNumber;
                    this->weights[j][i]=currentNumber;
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
        vector<T2> Neighbourhood(T2 v)
        {
            vector<T2> neighbourhood;
            for (int k=0;k<this->n;k++)
            {
                if (adj[v][k])
                {
                    neighbourhood.push_back(k);
                }
            }
            return neighbourhood;
        }

        // Returns a string representation of this matrix
        string ToString()
        {
            int i,j;
            string output;
            output.reserve(this->n*this->n*sizeof(ConvertToString(this->n)));
            //cout << "nodes:"<<this->n<<endl;
            for(i = 0;i < this->n;i++)
            {
                for(j = 0; j < this->n; j++)
                {
                    output.append(ConvertToString(adj[i][j])+" ");
                }
                output.append("\n");
            }
            return output;
        }
};
#endif
