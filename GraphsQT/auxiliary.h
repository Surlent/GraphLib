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
#include <math.h>
#include <time.h>
using namespace std;

template <typename T>
string ConvertToString ( const T &Number )
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

template<typename T> T DistanceSquared(const pair<T,T> &a,const pair<T,T> &b)
{
    return (T)(pow(a.first-b.first,2)+pow(a.second-b.second,2));
}

template<typename T> T Distance(const pair<T,T> &a,const pair<T,T> &b)
{
    return (T)sqrt(DistanceSquared<T>(a,b));
}
template<typename T> pair<T,T> MultiplyByConstant(const T &t,const pair<T,T> &a)
{
    return make_pair(a.first*t,a.second*t);
}

// Adds a to b
template<typename T> pair<T,T> Add(const pair<T,T> &a, const pair<T,T> &b)
{
    return make_pair(a.first+b.first,a.second+b.second);
}

// Subtracts b from a
template<typename T> pair<T,T> Subtract(const pair<T,T> &a, const pair<T,T> &b)
{
    return make_pair(a.first-b.first,a.second-b.second);
}

// Returns dot product of two pairs
template<typename T> T DotProduct(const pair<T,T> &a, const pair<T,T> &b)
{
    return (T)((a.first*b.first)+(a.second*b.second));
}

template<typename T> T DistanceToSegment(const pair<T,T> &p,const pair<pair<T,T>,pair<T,T>> &l) {
  // Return minimum distance between line segment l and point p
  pair<T,T> segmentA=l.first; // first end of segment
  pair<T,T> segmentB=l.second; // last end of segment
  const float distSquared = DistanceSquared<T>(segmentA, segmentB);  // i.e. |w-v|^2 -  avoid a sqrt
  if (distSquared == 0.0) return Distance<T>(p, segmentA);   // v == w case
  // Consider the line extending the segment, parameterized as v + t (w - v).
  // We find projection of point p onto the line.
  // It falls where t = [(p-v) . (w-v)] / |w-v|^2
  const T t = DotProduct<T>(Subtract<T>(p,segmentA), Subtract<T>(segmentB,segmentA)) / distSquared;
  if (t < 0.0) return Distance<T>(p, segmentA);       // Beyond the 'v' end of the segment
  else if (t > 1.0) return Distance<T>(p, segmentB);  // Beyond the 'w' end of the segment
  pair<T,T> projection = Add<T>(segmentA, MultiplyByConstant<T>(t,Subtract<T>(segmentB,segmentA)));  // Projection falls on the segment
  return Distance(p, projection);
}

template<typename T> bool SegmentsIntersect(const pair<pair<T,T>,pair<T,T>> &edge1,const pair<pair<T,T>,pair<T,T>> &edge2)
{
    pair<T,T> A=edge1.first;
    pair<T,T> B=edge1.second;
    pair<T,T> C=edge2.first;
    pair<T,T> D=edge2.second;
    T xA=A.first;
    T yA=A.second;
    T xB=B.first;
    T yB=B.second;
    T xC=C.first;
    T yC=C.second;
    T xD=D.first;
    T yD=D.second;
    return (((xC-xA)*(yB-yA) - (yC-yA)*(xB-xA)) * ((xD-xA)*(yB-yA) - (yD-yA)*(xB-xA)) < 0) && (((xA-xC)*(yD-yC) - (yA-yC)*(xD-xC)) * ((xB-xC)*(yD-yC) - (yB-yC)*(xD-xC)) < 0);
}

#endif
