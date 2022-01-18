#include <iostream>
#include "static_mat.h"
#include <unistd.h>
using namespace rp;
using namespace std;

using Vec3f=Vec_<float, 3>;
using Mat3f=Mat_<float, 3,3>;
using Mat3_4f=Mat_<float, 3,4>;

template <typename T>
void fill(T& x, typename T::Scalar v) {
  for (int i=0; i<x.dimension(); ++i)
    x.at(i)=v;
}

template <typename T>
void randFill(T& x) {
  for (int i=0; i<x.dimension(); ++i)
    x.at(i)=drand48();
}

int main() {
  Vec3f v1;
  v1.at(0)=1;
  v1.at(1)=2;
  v1.at(2)=3;
  cout << v1 << endl;

  Mat3f m;
  fill(m,0.);
  cout << m;
  Mat3f m2;
  fill(m2,0);
  m2.at(0,0)=1;
  m2.at(1,1)=2;
  m2.at(2,2)=3;

  cout << m2 << endl;
  cout << m + m2 << endl;
  Mat3f m3=m + m2;
 
  cout << m + m2 << endl;

  cout  << m3-m2 << endl;
  cout  << m * v1 << endl;
  cout  << m3 * v1 << endl;

  Mat3_4f m4;
  cerr << "m4 " << m4.dimension() << endl;
  fill(m4, 1);
  cout << m4 << endl;
  
  cout << (m3*m4).transpose() << endl;

  Mat3_4f m5;
  randFill(m5);
  cout << m5 << endl;
  
  cout << m5*m5.transpose() << endl;

  randFill(m);
  cout << m << endl;

  cout << m-m5*m5.transpose() << endl;

  //cout << m4*m4 << endl; << compile time error
}
