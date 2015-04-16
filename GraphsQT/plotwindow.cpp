#include "plotwindow.h"
#include "ui_plotwindow.h"
#include <DistanceMap.h>
#include <Graph.h>
#include <iostream>
PlotWindow::PlotWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::PlotWindow)
{
    ui->setupUi(this);

}

PlotWindow::~PlotWindow()
{
    delete ui;
}

void PlotWindow::on_pushButton_clicked()
{
    clock_t initialTime=clock();
    QCustomPlot* customPlot=ui->customPlot;
    customPlot->clearGraphs();
    // generate some data:
    DistanceMap<int>* adj=new DistanceMap<int>(ui->fileName->text().toStdString());
    cout<<"Graph loaded"<<endl;
    //Graph<long>* g=new Graph<long>(adj);
    //long n=adj->N();
    QVector<double> x=QVector<double>::fromStdVector(adj->XCoordinates());
    QVector<double> y=QVector<double>::fromStdVector(adj->YCoordinates());    
    cout<<"Reached"<<endl;
    auto tspTuple=adj->TSP(0,1);
    //auto chosenEdges=std::get<0>(tspTuple);
    auto path=std::get<0>(tspTuple);
    double pathLength=std::get<1>(tspTuple);


    // create graph and assign data to it:
    customPlot->addGraph();

    QCPScatterStyle myScatter;
    myScatter.setShape(QCPScatterStyle::ScatterShape::ssDisc);
    myScatter.setSize(10);
    customPlot->graph(0)->setScatterStyle(myScatter);
    customPlot->graph(0)->setData(x, y);
    customPlot->graph(0)->setLineStyle(QCPGraph::lsNone);

    QCPScatterStyle edgeScatterStyle;
    edgeScatterStyle.setShape(QCPScatterStyle::ScatterShape::ssNone);
    edgeScatterStyle.setSize(10);    
    int i=1;
    ofstream output("output.txt");
    for ( auto it = std::next(path.begin()); it != path.end(); ++it )
    {
        auto previousIt=std::prev(it);
        customPlot->addGraph();
        customPlot->graph(i)->setScatterStyle(edgeScatterStyle);
        customPlot->graph(i)->setLineStyle(QCPGraph::lsLine);
        customPlot->graph(i)->setPen(QPen(Qt::red));
        customPlot->graph(i)->addData(x[*previousIt], y[*previousIt]);
        customPlot->graph(i)->addData(x[*it], y[*it]);
        //totalDistance+=Distance<double>(make_pair(x[*previousIt],y[*previousIt]),make_pair(x[*it],y[*it]));
        //cout<<x[*previousIt]<<" "<<y[*previousIt]<<endl;
        //cout<<x[*it]<<" "<<y[*it]<<endl;
        //cout<<*previousIt<<" "<<*it<<" "<<Distance<double>(make_pair(x[*previousIt],y[*previousIt]),make_pair(x[*it],y[*it]))<<endl;
        //cout<<i-1<<"-"<<graphDegrees[i-1]<<"-"<<Distance(adj->Coordinates(*previousIt),adj->Coordinates(*it))<<endl;
        i++;

        output<<(*previousIt)+1<<" ";
    }    
    output<<endl;
    output<<pathLength<<endl;
    output<<"Elapsed:"<<(double)(clock()-initialTime)/CLOCKS_PER_SEC;
    //cout<<endl;
    ui->labelResult->setText(QString::fromStdString(ConvertToString(pathLength)));

    /*for(auto it=path.begin();it!=path.end();++it)
    {
        cout<<(*it)<<" ";
    }
    cout<<endl;*/
//    unsigned pathSize=path.size();
//    for (unsigned i=1; i<pathSize+1; ++i)
//    {
//        customPlot->addGraph();
//        customPlot->graph(i)->setScatterStyle(edgeScatterStyle);
//        customPlot->graph(i)->setPen(QPen(Qt::red));
//        if(i==pathSize)
//        {
//            customPlot->graph(i)->addData(x[path[pathSize-1]], y[pathSize-1]);
//            customPlot->graph(i)->addData(x[path[0]], y[path[0]]);
//        }
//        else
//        {
//            customPlot->graph(i)->addData(x[path[i-1]], y[path[i-1]]);
//            customPlot->graph(i)->addData(x[path[i]], y[path[i]]);
//        }
//        customPlot->graph(i)->setLineStyle(QCPGraph::lsLine);
//    }

    // give the axes some labels:
    customPlot->xAxis->setLabel("x");
    customPlot->yAxis->setLabel("y");
    // set axes ranges, so we see all data:
    customPlot->xAxis->setRange(0, 1000000);
    customPlot->yAxis->setRange(0, 1000000);
    customPlot->replot();
}
