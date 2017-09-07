// The MIT License (MIT)
// Copyright (c) 2014 Matthew Klingensmith and Ivan Dryanovski
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <open_chisel/geometry/Plane.h>

namespace chisel
{

    Plane::Plane()
    {

    }

    Plane::Plane(const Vec4& params) :
        normal(Vec3(params(0), params(1), params(2))), distance(params(3))
    {

    }

    Plane::Plane(const Vec3& _normal, float _distance) :
            normal(_normal), distance()
    {

    }

    /**
     * 按照平面的法向量和平面到原点的距离初始化平面
     * 以Ax+By+Cz+D=0
     * normal(A,B,C)
     * distance = D
     * 参考：http://www.songho.ca/math/plane/plane.html 中第一部分即可
     */
    Plane::Plane(const Vec3& a, const Vec3& b, const Vec3& c)
    {
        //！假设点a,b,c是平面上的三个点
        
        Vec3 ab = b - a;
        Vec3 ac = c - a;

        Vec3 cross = ab.cross(ac);
        normal = cross.normalized();
        distance = -(cross.dot(a));
    }

    Plane::Plane(float a, float b, float c, float d) :
                normal(a, b, c), distance(d)
    {

    }

    /**
     * [Plane::GetSignedDistance 点到平面的距离，注意normal是单位正交向量，所以没有分母]
     * @param  point [description]
     * @return       [description]
     */
    float Plane::GetSignedDistance(const Vec3& point) const
    {
        return point.dot(normal) + distance;
    }

    Plane::~Plane()
    {

    }

} // namespace chisel 
