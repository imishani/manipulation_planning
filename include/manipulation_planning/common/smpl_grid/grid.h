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

#ifndef SMPL_GRID_H
#define SMPL_GRID_H

#include <cstdlib>
#include <iterator>

namespace smpl {

template <typename T>
class Grid3
{
public:

    typedef T           value_type;

    typedef std::size_t     size_type;
    typedef std::ptrdiff_t  difference_type;

    typedef value_type&         reference;
    typedef const value_type&   const_reference;

    typedef T*       pointer;
    typedef const T* const_pointer;

    typedef pointer         iterator;
    typedef const_pointer   const_iterator;

    typedef std::reverse_iterator<iterator>         reverse_iterator;
    typedef std::reverse_iterator<const_iterator>   const_reverse_iterator;

    Grid3();

    Grid3(
        size_type xdim,
        size_type ydim,
        size_type zdim,
        const T& value);

    explicit Grid3(
        size_type xdim,
        size_type ydim,
        size_type zdim);

    Grid3(const Grid3& other);
    Grid3(Grid3&& other);

    ~Grid3();

    Grid3& operator=(const Grid3& rhs);
    Grid3& operator=(Grid3&& rhs);

    void assign(size_type xdim, size_type ydim, size_type zdim, const T& value);

    reference       at(size_type x, size_type y, size_type z);
    const_reference at(size_type x, size_type y, size_type z) const;

    reference       operator[](size_type pos);
    const_reference operator[](size_type pos) const;

    reference       operator()(size_type x, size_type y, size_type z);
    const_reference operator()(size_type x, size_type y, size_type z) const;

    T* data();
    const T* data() const;

    iterator begin();
    const_iterator begin() const;
    const_iterator cbegin() const;

    iterator end();
    const_iterator end() const;
    const_iterator cend() const;

    reverse_iterator rbegin();
    const_reverse_iterator rbegin() const;
    const_reverse_iterator crbegin() const;

    reverse_iterator rend();
    const_reverse_iterator rend() const;
    const_reverse_iterator crend() const;

    size_type xsize() const { return m_dims[0]; }
    size_type ysize() const { return m_dims[1]; }
    size_type zsize() const { return m_dims[2]; }
    size_type size() const;
    size_type max_size() const;

    void clear();

    void resize(size_type xdim, size_type ydim, size_type zdim);
    void resize(size_type xdim, size_type ydim, size_type zdim, const value_type& value);

    void swap(Grid3& other);

    size_type coord_to_index(size_type x, size_type y, size_type z) const;
    void index_to_coord(size_type i, size_type& x, size_type& y, size_type& z) const;

    bool in_bounds(size_type x, size_type y, size_type z) const;

private:

    value_type* m_data;
    size_type m_dims[3];

    void resize(size_type count);
};

template <typename T>
void swap(Grid3<T>& lhs, Grid3<T>& rhs);

} // namespace smpl

#include "grid.hpp"

#endif
