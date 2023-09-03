////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2016, Andrew Dornbush
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     1. Redistributions of source code must retain the above copyright notice
//        this list of conditions and the following disclaimer.
//     2. Redistributions in binary form must reproduce the above copyright
//        notice, this list of conditions and the following disclaimer in the
//        documentation and/or other materials provided with the distribution.
//     3. Neither the name of the copyright holder nor the names of its
//        contributors may be used to endorse or promote products derived from
//        this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////

/// \author Andrew Dornbush

#ifndef SMPL_GRID_HPP
#define SMPL_GRID_HPP

#include "grid.h"

#include <algorithm>
#include <stdexcept>

namespace smpl {

template <typename T>
Grid3<T>::Grid3() : m_data(nullptr), m_dims()
{
    m_dims[0] = 0;
    m_dims[1] = 0;
    m_dims[2] = 0;
}

template <typename T>
Grid3<T>::Grid3(
    size_type xdim,
    size_type ydim,
    size_type zdim,
    const T& value)
:
    m_data(new T[xdim * ydim * zdim]),
    m_dims()
{
    std::fill(m_data, m_data + (xdim * ydim * zdim), value);
    m_dims[0] = xdim;
    m_dims[1] = ydim;
    m_dims[2] = zdim;
}

template <typename T>
Grid3<T>::Grid3(
    size_type xdim,
    size_type ydim,
    size_type zdim)
:
    m_data(new T[xdim * ydim * zdim]),
    m_dims()
{
    m_dims[0] = xdim;
    m_dims[1] = ydim;
    m_dims[2] = zdim;
}

template <typename T>
Grid3<T>::Grid3(const Grid3& other) :
    m_data(new T[other.m_dims[0] * other.m_dims[1] * other.m_dims[2]]),
    m_dims()
{
    size_type total_size = other.m_dims[0] * other.m_dims[1] * other.m_dims[2];
    std::copy(other.m_data, other.m_data + total_size, m_data);
    m_dims[0] = other.m_dims[0];
    m_dims[1] = other.m_dims[1];
    m_dims[2] = other.m_dims[2];
}

template <typename T>
Grid3<T>::Grid3(Grid3&& other)
{
    m_data = other.m_data;
    m_dims[0] = other.m_dims[0];
    m_dims[1] = other.m_dims[1];
    m_dims[2] = other.m_dims[2];
    other.m_data = nullptr;
    other.m_dims[0] = 0;
    other.m_dims[1] = 0;
    other.m_dims[2] = 0;
}

template <typename T>
Grid3<T>::~Grid3()
{
     if (m_data) {
         delete [] m_data;
     }
     m_dims[0] = 0;
     m_dims[1] = 0;
     m_dims[2] = 0;
}

template <typename T>
Grid3<T>& Grid3<T>::operator=(const Grid3& rhs)
{
    if (this != &rhs) {
        if (m_data) {
            delete [] m_data;
        }

        size_type total_size = rhs.m_dims[0] * rhs.m_dims[1] * rhs.m_dims[2];
        m_data = new T[total_size];
        std::copy(rhs.m_data, rhs.m_data + total_size, m_data);
        m_dims[0] = rhs.m_dims[0];
        m_dims[1] = rhs.m_dims[1];
        m_dims[2] = rhs.m_dims[2];
    }
    return *this;
}

template <typename T>
Grid3<T>& Grid3<T>::operator=(Grid3&& rhs)
{
    if (this != &rhs) {
        if (m_data) {
            delete [] m_data;
        }

        m_data = rhs.m_data;
        m_dims[0] = rhs.m_dims[0];
        m_dims[1] = rhs.m_dims[1];
        m_dims[2] = rhs.m_dims[2];
        rhs.m_data = nullptr;
        rhs.m_dims[0] = 0;
        rhs.m_dims[1] = 0;
        rhs.m_dims[2] = 0;
    }
    return *this;
}

template <typename T>
void Grid3<T>::assign(
    size_type xdim,
    size_type ydim,
    size_type zdim,
    const T& value)
{
    size_type total_size = xdim * ydim * zdim;
    resize(total_size);
    m_dims[0] = xdim;
    m_dims[1] = ydim;
    m_dims[2] = zdim;
    std::fill(m_data, m_data + total_size, value);
}

template <typename T>
typename Grid3<T>::reference
Grid3<T>::at(size_type x, size_type y, size_type z)
{
    if (!in_bounds(x, y, z)) {
        throw std::out_of_range("Grid3<T>::at called with invalid coordinates");
    }
    return m_data[coord_to_index(x, y, z)];
}

template <typename T>
typename Grid3<T>::const_reference
Grid3<T>::at(size_type x, size_type y, size_type z) const
{
    if (!in_bounds(x, y, z)) {
        throw std::out_of_range("Grid3<T>::at called with invalid coordinates");
    }
    return m_data[coord_to_index(x, y, z)];
}

template <typename T>
typename Grid3<T>::reference
Grid3<T>::operator[](size_type pos)
{
    return m_data[pos];
}

template <typename T>
typename Grid3<T>::const_reference
Grid3<T>::operator[](size_type pos) const
{
    return m_data[pos];
}

template <typename T>
typename Grid3<T>::reference
Grid3<T>::operator()(size_type x, size_type y, size_type z)
{
    return m_data[coord_to_index(x, y, z)];
}

template <typename T>
typename Grid3<T>::const_reference
Grid3<T>::operator()(size_type x, size_type y, size_type z) const
{
    return m_data[coord_to_index(x, y, z)];
}

template <typename T>
T* Grid3<T>::data()
{
    return m_data;
}

template <typename T>
const T* Grid3<T>::data() const
{
    return m_data;
}

template <typename T>
typename Grid3<T>::iterator Grid3<T>::begin()
{
    return m_data;
}

template <typename T>
typename Grid3<T>::const_iterator Grid3<T>::begin() const
{
    return m_data;
}

template <typename T>
typename Grid3<T>::const_iterator Grid3<T>::cbegin() const
{
    return m_data;
}

template <typename T>
typename Grid3<T>::iterator Grid3<T>::end()
{
    return m_data + size();
}

template <typename T>
typename Grid3<T>::const_iterator Grid3<T>::end() const
{
    return m_data + size();
}

template <typename T>
typename Grid3<T>::const_iterator Grid3<T>::cend() const
{
    return m_data + size();
}

template <typename T>
typename Grid3<T>::reverse_iterator Grid3<T>::rbegin()
{
    // TODO: implement
    return reverse_iterator(begin());
}

template <typename T>
typename Grid3<T>::const_reverse_iterator
Grid3<T>::rbegin() const
{
    // TODO: implement
    return const_reverse_iterator(begin());
}

template <typename T>
typename Grid3<T>::const_reverse_iterator
Grid3<T>::crbegin() const
{
    // TODO: implement
    return const_reverse_iterator(begin());
}

template <typename T>
typename Grid3<T>::reverse_iterator Grid3<T>::rend()
{
    // TODO: implement
    return reverse_iterator(end());
}

template <typename T>
typename Grid3<T>::const_reverse_iterator
Grid3<T>::rend() const
{
    // TODO: implement
    return const_reverse_iterator(end());
}

template <typename T>
typename Grid3<T>::const_reverse_iterator
Grid3<T>::crend() const
{
    // TODO: implement
    return const_reverse_iterator(end());
}

template <typename T>
typename Grid3<T>::size_type Grid3<T>::size() const
{
    return m_dims[0] * m_dims[1] * m_dims[2];
}

template <typename T>
typename Grid3<T>::size_type Grid3<T>::max_size() const
{
    return std::numeric_limits<size_type>::max();
}

template <typename T>
void Grid3<T>::clear()
{
    if (m_data) {
        delete [] m_data;
        m_data = nullptr;
    }
    m_dims[0] = 0;
    m_dims[1] = 0;
    m_dims[2] = 0;
}

template <typename T>
void Grid3<T>::resize(size_type xdim, size_type ydim, size_type zdim)
{
    size_type total_size = xdim * ydim * zdim;
    resize(total_size);
    m_dims[0] = xdim;
    m_dims[1] = ydim;
    m_dims[2] = zdim;
}

template <typename T>
void Grid3<T>::resize(
    size_type xdim,
    size_type ydim,
    size_type zdim,
    const value_type& value)
{
    assign(xdim, ydim, zdim, value);
}

template <typename T>
void Grid3<T>::swap(Grid3& other)
{
    // TODO: implement
}

template <typename T>
typename Grid3<T>::size_type
Grid3<T>::coord_to_index(size_type x, size_type y, size_type z) const
{
    return m_dims[2] * (x * m_dims[1] + y) + z;
}

template <typename T>
void
Grid3<T>::index_to_coord(
    size_type i,
    size_type& x,
    size_type& y,
    size_type& z) const
{
    x = i / (m_dims[1] * m_dims[2]);
    y = (i - x * m_dims[1] * m_dims[2]) / m_dims[2];
    z = i - (x * m_dims[1] * m_dims[2]) - y * m_dims[2];
}

template <typename T>
bool Grid3<T>::in_bounds(size_type x, size_type y, size_type z) const
{
    return x < m_dims[0] & y < m_dims[1] & z < m_dims[2];
}

template <typename T>
void Grid3<T>::resize(size_type count)
{
    if (size() != count) {
        if (m_data) {
            delete [] m_data;
        }
        m_data = new T[count];
    }
}

} // namespace smpl

#endif
