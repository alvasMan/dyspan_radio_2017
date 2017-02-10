/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   general_utils.hpp
 * Author: connect
 *
 * Created on 31 January 2017, 10:11
 */

#include <boost/format.hpp>
#include <sstream>      // std::stringstream

#ifndef GENERAL_UTILS_HPP
#define GENERAL_UTILS_HPP

template<typename Iterator> 
std::string print_container(Iterator vec_begin, Iterator vec_end)
{
    std::stringstream os;
    Iterator it = vec_begin;
    os << "[";
    os << *vec_begin;
    for (++it; it != vec_end; ++it)
        os << ", " << *it;
    os << "]";
    return os.str();
}

template<typename Iterator, typename Op> 
std::string print_container(Iterator vec_begin, Iterator vec_end, Op func)
{
    std::ostringstream os;
    Iterator it = vec_begin;
    os << "[" << func(*vec_begin);
    for (++it; it != vec_end; it++)
        os << ", " << func(*it);
    os << "]";
    return os.str();
}

template<typename It>
std::string print_container_dB(const It itbegin, const It itend) 
{
    return print_container(itbegin, itend, [](decltype(*itbegin) a)
    {
        return boost::format("%1.11f") % (10 * log10(abs(a)));
    });
}

template<typename Range> 
std::string print_range(const Range& r)
{
    return print_container(r.begin(), r.end());
}

template<typename Range, typename Op> 
std::string print_range(const Range& r, Op func)
{
    return print_container(r.begin(), r.end(), func);
}


template<typename Range>
std::string print_range_dB(const Range& r) 
{
    return print_container(r.begin(),r.end());
}

#endif /* GENERAL_UTILS_HPP */

