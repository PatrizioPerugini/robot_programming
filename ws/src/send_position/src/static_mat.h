#pragma once
#include "static_vec.h"
#include <iostream>
// untested

namespace rp {
  template <typename Scalar_, int Rows_, int Cols_>
  class Mat_ {
  public:
    using Scalar=Scalar_;
    static constexpr int Rows=Rows_;
    static constexpr int Cols=Cols_;
    static constexpr int Dimension=Rows_*Cols_;
    using ThisType=Mat_<Scalar, Rows, Cols>;
    using ThisTypeTransposed=Mat_<Scalar, Cols_,Rows_>;
    using ColVecType=Vec_<Scalar, Rows_>;
    using RowVecType=Vec_<Scalar, Cols_>;

    ThisType& operator+=(const ThisType& other) {
      for (int i=0; i<Dimension; ++i)
        _values[i]+=other._values[i];
      return *this;
    }
    
    ThisType& operator-=(const ThisType& other) {
      for (int i=0; i<Dimension; ++i)
        _values[i]-=other._values[i];
      return *this;
    }

    ThisType& operator*=(Scalar& scalar) {
      for (int i=0; i<Dimension; ++i)
        _values[i]*=scalar;
      return *this;
    }
    
    ColVecType operator*(RowVecType& v) const {
      ColVecType ret;
      Scalar acc=Scalar(0);
      for (int r=0; r<rows(); ++r) {
        Scalar& dest = ret.at(r);
        dest=Scalar(0);
        for (int c=0; c<cols(); ++c){
          dest+=at(r,c)*v.at(c);
        }
      }
      return ret;
    }

    ThisTypeTransposed transpose() const {
      ThisTypeTransposed ret;
      for (int r=0; r<rows(); ++r) {
        for (int c=0; c<cols(); ++c){
          ret.at(c,r)=at(r,c);
        }
      }
      return ret;
    }
    
    ThisType operator+(const ThisType& other) const {
      ThisType ret(*this);
      ret+=other;
      return ret;
    }
    
    ThisType operator-(const ThisType& other) const {
      ThisType ret(*this);
      ret-=other;
      return ret;
    }
    
    ThisType operator*(const Scalar& s) const {
      ThisType ret(*this);
      ret*=s;
      return ret;
    }

    template<int OtherCols>
    Mat_<Scalar, Rows, OtherCols> operator *(const Mat_<Scalar, Cols, OtherCols>& other) {
      Mat_<Scalar, Rows, OtherCols> ret;
      for (int dc=0; dc<OtherCols; ++dc) {
        for (int r=0; r<Rows; ++r) {
          Scalar& acc=ret.at(r,dc);
          acc=Scalar(0);
          for (int c=0; c<Cols; ++c)
            acc+=at(r,c)*other.at(c,dc);
        }
      }
      return ret;
    }
    
    static const int dimension()   {return Dimension;}
    static const int rows()  {return Rows;}
    static const int cols()  {return Cols;}
    inline Scalar& at(int pos) {return _values[pos];}
    inline const Scalar& at(int pos) const {return _values[pos];}
    inline Scalar& at(int r, int c) {return _values[r*Cols+c];}
    inline const Scalar& at(int r, int c) const {return _values[r*Cols+c];}
  protected:
    Scalar _values[Dimension];
  };
  
  template <typename Scalar_, int Rows_, int Cols_>
  std::ostream&  operator << (std::ostream& os, const Mat_<Scalar_, Rows_, Cols_>& m) {
    for (int r=0; r<m.rows(); ++r) { 
      for (int c=0; c<m.cols(); ++c) {
        os << m.at(r,c) << " ";
      }
      os << std::endl;
    }
    return os;
  }
    
}
