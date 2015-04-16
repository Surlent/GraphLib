#include <auxiliary.h>
#include <AdjacencyMatrix.h>
#include <AdjacencyList.h>
#include <Graph.h>
#include <GraphWriter.h>
using namespace std;

int main()
{
    string command;
    string outputFileName="output.txt";
    while (true)
    {
        cout<<"Enter command (mode,graphname,function,params): ";
        getline(cin,command);
        if(command!="0")
        {
            string mode,graphName,graphFunction;
            istringstream iss(command);
            iss>>mode>>graphName;
            string fileName=("assets/"+graphName+".txt");
            fstream file(fileName);
            if (file.good())
            {
                if (mode=="findbyname")
                {
                    string fullName="",namePart;
                    while( iss >> namePart )
                    {
                        if(fullName!="")
                        {
                            fullName+=" ";
                        }
                        fullName+=namePart;
                    }

                    string wantedName=fullName;
                    ifstream file(fileName);
                    int id;
                    string name,line;
                    int vCounter=0;
                    bool found=false;
                    while(getline(file,line)&&!found)
                    {
                        istringstream issfind(line);
                        string v;
                        while(getline(issfind,v,','))
                        {
                            if(vCounter%2==0)
                            {
                                id=ConvertFromString<int>(v);
                                //cout<<id<<" ";
                            }
                            else
                            {
                                name=v;
                                if(wantedName==name)
                                {
                                    found=true;
                                    cout<<"The id for this name is: "<<id<<endl;
                                    break;
                                }
                            }
                            vCounter++;

                        }

                    }
                    if(!found)
                    {
                        cout<<"Name not found."<<endl;
                    }
                }
                if (mode=="findbyid")
                {
                    int wantedID;
                    iss>>wantedID;

                    ifstream file(fileName);
                    int id;
                    string name,line;
                    int vCounter=0;
                    bool found=false;
                    while(getline(file,line)&&!found)
                    {
                        istringstream issfind(line);
                        string v;
                        while(getline(issfind,v,','))
                        {
                            if(vCounter%2==0)
                            {
                                id=ConvertFromString<int>(v);
                                //cout<<id<<" ";
                            }
                            else
                            {
                                name=v;
                                if(wantedID==id)
                                {
                                    found=true;
                                    cout<<"The name for this id is: "<<name<<endl;
                                    break;
                                }
                            }
                            vCounter++;

                        }

                    }
                    if(!found)
                    {
                        cout<<"Name not found."<<endl;
                    }
                }
                else if((mode=="write")||(mode=="run"))
                {
                    ostream* outputStream(NULL);
                    ofstream outputFile;
                    if (mode=="run")
                    {
                        outputStream=&cout;
                    }
                    else if (mode=="write")
                    {
                        outputFile.open(outputFileName);
                        outputStream=&outputFile;
                    }
                    iss>>graphFunction;
                    vector<int> parameters;
                    int parameter;
                    while( iss >> parameter )
                    {
                        parameters.push_back(parameter);
                    }
                    int paramCount=parameters.size();
                    AdjacencyList<int>* adj=new AdjacencyList<int>(fileName);
                    Graph<int>* g=new Graph<int>(adj);
                    GraphWriter<int>* gw=new GraphWriter<int>(g);
                    if (graphFunction=="usecase")
                    {
                        if (parameters[0]==1)
                        {
                            for(int i=10;i<=50;i+=10)
                            {
                                gw->DistanceOutput(*outputStream,1,i);
                            }
                        }
                        else if (parameters[0]==2)
                        {
                            for(int i=10;i<=50;i+=10)
                            {
                                gw->ShortestPathOutput(*outputStream,1,i);
                                *outputStream<<endl;
                            }
                        }
                        else if(parameters[0]==3)
                        {
                            double start = omp_get_wtime( );
                            gw->MSTOutput(*outputStream);
                            double time= omp_get_wtime( )-start;
                            *outputStream<<time<<endl;
                        }
                        else if(parameters[0]==4)
                        {
                            vector<int> pesquisadores;
                            int dijkstra=2722;

                            pesquisadores.push_back(11365);
                            pesquisadores.push_back(471365);
                            pesquisadores.push_back(5709);
                            pesquisadores.push_back(11386);
                            pesquisadores.push_back(343930);
                            pesquisadores.push_back(309497);

                            for(int p : pesquisadores)
                            {
                                gw->DistanceOutput(*outputStream,dijkstra,p);
                                *outputStream<<endl;
                                gw->ShortestPathOutput(*outputStream,dijkstra,p);
                                *outputStream<<endl;
                            }
                        }
                        else if(parameters[0]==5)
                        {
                           int dijkstra=2722;
                           gw->NeighbourhoodOutput(*outputStream,dijkstra);
                        }
                        else if(parameters[0]==6)
                        {
                           vector<int> degrees=g->Degrees();
                           //vector<int> indices=OrderDecreasingValue(degrees);
                           //std::sort(degrees.begin(),degrees.end(),SmallerValuePair<int,int>());
                           for(auto i:OrderDecreasingValue(degrees))
                           {
                              *outputStream<<i+1<<" "<<degrees[i]<<endl;
                           }

                        }
                        else if(parameters[0]==7)
                        {
                           int ratton=343930;
                           int marroquim=309497;
                           gw->NeighbourhoodOutput(*outputStream,ratton);
                           *outputStream<<endl;
                           gw->NeighbourhoodOutput(*outputStream,marroquim);
                        }
                    }
                    else if(graphFunction=="neigh")
                    {
                        if(paramCount==1)
                        {
                            gw->NeighbourhoodOutput(*outputStream,parameters[0]);
                        }
                        else
                        {
                            cout<<"Invalid number of arguments."<<endl;
                        }
                    }
                    else if(graphFunction=="bfs")
                    {
                        if(paramCount==1)
                        {
                            gw->BFSOutput(*outputStream,parameters[0]);
                        }
                        else if(paramCount==2)
                        {
                            gw->BFSOutput(*outputStream,parameters[0],parameters[1]);
                        }
                        else
                        {
                            cout<<"Invalid number of arguments."<<endl;
                        }
                    }
                    else if(graphFunction=="dfs")
                    {
                        if(paramCount==1)
                        {
                            gw->DFSOutput(*outputStream,parameters[0]);
                        }
                        else if(paramCount==2)
                        {
                            gw->DFSOutput(*outputStream,parameters[0],parameters[1]);
                        }
                        else
                        {
                            cout<<"Invalid number of arguments."<<endl;
                        }
                    }
                    else if(graphFunction=="cc")
                    {
                        if(paramCount==0)
                        {
                            gw->ConnectedComponentsOutput(*outputStream);
                        }
                        else
                        {
                            cout<<"Invalid number of arguments"<<endl;
                        }
                    }
                    else if(graphFunction=="diam")
                    {
                        if(paramCount==2)
                        {
                            gw->DiameterOutput(*outputStream,parameters[0],parameters[1]);
                        }
                        else
                        {
                            cout<<"Invalid number of arguments."<<endl;
                        }
                    }
                    else if(graphFunction=="dist")
                    {
                        if(paramCount==1)
                        {
                            gw->DistanceOutput(*outputStream,parameters[0]);
                        }
                        else if(paramCount==2)
                        {
                            gw->DistanceOutput(*outputStream,parameters[0],parameters[1]);
                        }
                        else
                        {
                            cout<<"Invalid number of arguments."<<endl;
                        }
                    }
                    else if(graphFunction=="path")
                    {
                        if(paramCount==1)
                        {
                            gw->ShortestPathOutput(*outputStream,parameters[0]);
                        }
                        else if(paramCount==2)
                        {
                            gw->ShortestPathOutput(*outputStream,parameters[0],parameters[1]);
                        }
                        else
                        {
                            cout<<"Invalid number of arguments."<<endl;
                        }
                    }
                    else if(graphFunction=="mst")
                    {
                        if(paramCount==0)
                        {
                            gw->MSTOutput(*outputStream);
                        }
                        else if(paramCount==1)
                        {
                            gw->MSTOutput(*outputStream,parameters[0]);
                        }
                        else
                        {
                            cout<<"Invalid number of arguments."<<endl;
                        }
                    }
                    else if(graphFunction=="meandist")
                    {
                        if (paramCount==0)
                        {
                            double start = omp_get_wtime( );
                            gw->MeanDistanceOutput(*outputStream);
                            double time= omp_get_wtime( )-start;
                            *outputStream<<"Execution time: "<<time<<endl;
                        }
                        else
                        {
                            cout<<"Invalid number of arguments."<<endl;
                        }
                    }
                    else
                    {
                        cout<<"Function not found."<<endl;
                    }
                    if(mode=="write")
                    {
                        outputFile.close();
                    }
                }
                cout<<"Command completed."<<endl;
                //g->DiameterOutput("output.txt",10,1);
                /*ofstream output;
                output.open("output.txt");
                //output<<g->ToString();*/
            }
            else
            {
                cout<<"Invalid graph name. Try again."<<endl;
            }
        }
        else
        {
            break;
        }
    }
    return 0;
}
