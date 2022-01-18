#include <iostream>
#include "static_vec.h"
using namespace rp;
using namespace std;

using Vec3f=Vec_<float, 3>;

int main() {
  Vec3f v1;
  v1.at(0)=1;
  v1.at(1)=2;
  v1.at(2)=3;
  cout << v1 << endl;

  Vec3f v2(v1);
  cout << v1 << endl;

  v2+=v1;
  cout << v2 << endl;

  v2-=v1;
  cout << v2 << endl;
  cout << v2.squaredNorm();
  cout << v2*3.5f << endl;
  
}
