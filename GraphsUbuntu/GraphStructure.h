#ifndef GRAPH_STRUCTURE_H_INCLUDED
#define GRAPH_STRUCTURE_H_INCLUDED

#include <auxiliary.h>

template<typename T> class GraphStructure
{
    protected:
        int n=0,m=0;
        vector<int> degrees;
        unordered_map<pair<T,T>,double> weights;
        bool weighted=false;
        bool negativelyWeighted=false;
    public:
        virtual std::string ToString(){return "undefined graph structure";};
        virtual vector<T> Neighbourhood(int v){vector<T> r; return r;};
        int N(){return n;};
        int M(){return m;};
        unordered_map<pair<T,T>,double> Weights(){return weights;}
        int IsWeighted(){return this->weighted;}
        int IsNegativelyWeighted(){return this->negativelyWeighted;}
        vector<int> Degrees(){return degrees;};

};
namespace std {
    template<typename a, typename b>
    struct hash< std::pair<a, b> > {
    private:
       const hash<a> ah;
       const hash<b> bh;
    public:
       hash() : ah(), bh() {}
       size_t operator()(const std::pair<a, b> &p) const {
           //return ah((p.first + p.second)*(p.first + p.second + 1)/2) + bh(p.second);
      //return ah(p.first) ^ bh(p.second);
      return ah( p.first << 16 ) ^ bh(p.second);
   }
};
}
#endif
