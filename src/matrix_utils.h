/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   matrix_utils.h
 * Author: connect
 *
 * Created on 15 February 2017, 18:09
 */

#include <vector>

#ifndef MATRIX_UTILS_H
#define MATRIX_UTILS_H

template<typename T>
class Matrix
{
private:
    size_t n_rows = 0;
    size_t n_cols = 0;
    std::vector<T> mat;
    
public:
    typedef typename std::vector<T>::iterator iterator;
    Matrix() = default;
    Matrix(size_t Nrows, size_t Ncols) : n_rows(Nrows), n_cols(Ncols), mat(n_rows*n_cols) 
    {
    }
    inline T& at(size_t i, size_t j)
    {
        mat[i*n_cols + j];
    }
    inline size_t rows() const
    {
        return n_rows;
    }
    inline size_t cols() const
    {
        return n_cols;
    }
    inline iterator begin()
    {
        return mat.begin();
    }
    inline iterator end()
    {
        return mat.end();
    }
    inline size_t total_size() const
    {
        return n_rows*n_cols;
    }
};

#endif /* MATRIX_UTILS_H */

