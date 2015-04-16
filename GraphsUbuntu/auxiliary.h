#ifndef AUXILIARY_H_INCLUDED
#define AUXILIARY_H_INCLUDED
#include <iostream>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
#include <algorithm>
#include <list>
#include <vector>
#include <queue>
#include <stack>
#include <unordered_map>
#include <omp.h>

using namespace std;

template <typename T>
string ConvertToString ( T Number )
{
 ostringstream ss;
 ss << Number;
 return ss.str();
}
template <typename T> T ConvertFromString( const string& Text )
{
 istringstream ss(Text);
 T result;
 return ss >> result ? result : 0;
}

bool BiggerIntVector(const vector<int>& a,const vector<int>& b);

template <typename T,typename T2> struct SmallerValuePair{
  bool operator()(pair<T,T2> const& lhs, pair<T,T2> const& rhs){
    return lhs.second > rhs.second;
  }
};

template <typename T>
std::vector<T> OrderDecreasingValue(std::vector<T> const& values) {
    std::vector<T> indices(values.size());
    std::iota(begin(indices), end(indices), static_cast<T>(0));

    std::sort(
        begin(indices), end(indices),
        [&](T a, T b) { return values[a] > values[b]; }
    );
    return indices;
}
#endif
