#-------------------------------------------------
#
# Project created by QtCreator 2014-12-02T22:36:07
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets printsupport
TARGET = GraphsQT
TEMPLATE = app


SOURCES += main.cpp\
        plotwindow.cpp \
    auxiliary.cpp \
    qcustomplot/qcustomplot.cpp

HEADERS  += plotwindow.h \
    AdjacencyList.h \
    AdjacencyMatrix.h \
    auxiliary.h \
    DistanceMap.h \
    Graph.h \
    GraphStructure.h \
    GraphWriter.h \
    qcustomplot/qcustomplot.h

FORMS    += plotwindow.ui
QMAKE_CXXFLAGS += -std=c++14 -fexceptions -Wall -fopenmp -O3 -mtune=native -march=native
QMAKE_LFLAGS += -s
