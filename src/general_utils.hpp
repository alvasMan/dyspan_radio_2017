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
#include <numeric>

#ifndef GENERAL_UTILS_HPP
#define GENERAL_UTILS_HPP

namespace containers
{
template<typename Iterator>
std::string print(Iterator b, Iterator e)
{
    std::stringstream os;
    Iterator it = b;
    os << "[";
    os << *b;
    for (++it; it != e; ++it)
        os << ", " << *it;
    os << "]";
    return os.str();
}

template<typename Iterator, typename Op> 
std::string print(Iterator b, Iterator e, Op func)
{
    std::stringstream os;
    Iterator it = b;
    os << "[" << func(*b);
    for (++it; it != e; it++)
        os << ", " << func(*it);
    os << "]";
    return os.str();
}

template<typename Iterator, typename Op> 
std::string print(Iterator b, Iterator e, const std::string& separator, Op func)
{
    std::stringstream os;
    Iterator it = b;
    os << "[" << func(*b);
    for (++it; it != e; it++)
        os << separator << func(*it);
    os << "]";
    return os.str();
}

template<typename Iterator, typename Op> 
std::string print(Iterator b, Iterator e, const std::string& separator, const std::string& format, Op func)
{
    std::stringstream os;
    Iterator it = b;
    os << "[" << func(*b);
    for (++it; it != e; it++)
        os << separator << boost::format(format) % func(*it);
    os << "]";
    return os.str();
}

template<typename It, typename Op>
size_t argmax(It b, It e, Op op)
{
    return std::distance(b,std::max_element(b, e, op));
}
template<typename It, typename Op>
size_t argmin(It b, It e, Op op)
{
    return std::distance(b,std::min_element(b, e, op));
}
};

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

template<typename Iterator, typename Op> 
std::string print_container(Iterator vec_begin, Iterator vec_end, const std::string& separator, Op func)
{
    std::ostringstream os;
    Iterator it = vec_begin;
    os << "[" << func(*vec_begin);
    for (++it; it != vec_end; it++)
        os << separator << func(*it);
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

namespace ranges
{
template<typename C>
size_t argmax(const C& c)
{
    return std::distance(c.begin(),std::max_element(c.begin(), c.end()));
}

template<typename C, typename Op>
size_t argmax(const C& c, Op op)
{
    return std::distance(c.begin(),std::max_element(c.begin(), c.end(), op));
}

template<typename C>
size_t argmin(const C& c)
{
    return std::distance(c.begin(),std::min_element(c.begin(), c.end()));
}

template<typename C, typename Op>
size_t argmin(const C& c, Op op)
{
    return std::distance(c.begin(),std::max_element(c.begin(), c.end(), op));
}


template<typename T = int>
constexpr std::vector<T> make_range(int max)
{
    std::vector<T> c(max);
    std::iota(c.begin(), c.end(),0);
    return c;
}

template<typename T = int>
constexpr std::vector<T> make_range(int min, int max)
{
    std::vector<T> c(max-min);
    std::iota(c.begin(), c.end(),min);
    return c;
}
};

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

template<typename Range, typename Op> 
std::string print_range(const Range& r, const std::string& separator, Op func)
{
    return print_container(r.begin(), r.end(), separator, func);
}

template<typename Range>
std::string print_range_dB(const Range& r) 
{
    return print_container(r.begin(),r.end());
}

#endif /* GENERAL_UTILS_HPP */

