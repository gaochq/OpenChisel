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

#include <open_chisel/geometry/Frustum.h>

#include <assert.h>
#include <algorithm>

namespace chisel
{

    Frustum::Frustum()
    {
        // TODO Auto-generated constructor stub

    }

    Frustum::~Frustum()
    {
        // TODO Auto-generated destructor stub
    }

    //! 对应论文 III.G 部分，建立视椎体,判断chunk是否与锥体交叉
    bool Frustum::Intersects(const AABB& box) const
    {
        //！提取椎体的六个面
        const Plane* planes[] = { &far, &near, &top, &bottom, &left, &right };

        for (const Plane* plane : planes)
        {
            Vec3 axisVert;
            const Vec3& normal = plane->normal;

            // x-axis
            // Which AABB vertex is furthest down (plane normals direction) the x axis.
            if (normal(0) < 0.0f)
                axisVert(0) = box.min(0);
            else
                axisVert(0) = box.max(0);

            // y-axis
            // Which AABB vertex is furthest down (plane normals direction) the y axis.
            if (normal(1) < 0.0f)
                axisVert(1) = box.min(1);
            else
                axisVert(1) = box.max(1);

            // z-axis
            // Which AABB vertex is furthest down (plane normals direction) the z axis.
            if (normal(2) < 0.0f)
                axisVert(2) = box.min(2);
            else
                axisVert(2) = box.max(2);

            // Now we get the signed distance from the AABB vertex that's furthest down
            // the frustum planes normal, and if the signed distance is negative, then
            // the entire bounding box is behind the frustum plane.
            if (axisVert.dot(normal) + plane->distance > 0.0f)
                return true;
        }

        return false;
    }
    /**
     * 判断point是否在锥体内，只要有一个点在外部，则
     * 对应论文 III.G 部分，建立视椎体,判断chunk是否在锥体内部
     */ 
    bool Frustum::Contains(const Vec3& point) const
    {
        // Compute distance to all the planes of the frustum.
        const Plane* planes[] = { &far, &near, &top, &bottom, &left, &right };

        int i = 0;
        for (const Plane* plane : planes)
        {
            if (plane->ClassifyPoint(point) == Plane::IntersectionType::Outside)
            {
                return false;
            }
            i++;
        }

        // Frustrum is convex intersection of positive parts of 6 planes.
        // If it is inside all the planes, it is inside the frustum.
        return true;
    }

    /**
     * [Frustum::ComputeBoundingBox 求取box的边界，这个地方和求多边形凸包的想法一致]
     * @param box [输入的box]
     */
    void Frustum::ComputeBoundingBox(AABB* box) const
    {
        assert(box != nullptr);
        float bigNum = std::numeric_limits<float>::max();

        Vec3 tempMin(bigNum, bigNum, bigNum);
        Vec3 tempMax(-bigNum, -bigNum, -bigNum);
        for (int i = 0; i < 8; i++)
        {
          const Vec3& corner = corners[i];
          tempMin(0) = std::min<float>(tempMin(0), corner(0));
          tempMin(1) = std::min<float>(tempMin(1), corner(1));
          tempMin(2) = std::min<float>(tempMin(2), corner(2));

          tempMax(0) = std::max<float>(tempMax(0), corner(0));
          tempMax(1) = std::max<float>(tempMax(1), corner(1));
          tempMax(2) = std::max<float>(tempMax(2), corner(2));
        }

        box->min = tempMin;
        box->max = tempMax;
    }

    /**
     * [Frustum::SetFromOpenGLViewProjection description]
     * @param view [description]
     * @param proj [description]
     * 函数没用到，没看懂
     */
    void Frustum::SetFromOpenGLViewProjection(const Mat4x4& view, const Mat4x4& proj)
    {
        Vec3 right = view.transpose().block(0, 0, 3, 1).cast<float>();
        Vec3 up = view.transpose().block(0, 1, 3, 1).cast<float>();
        Vec3 d = -view.transpose().block(0, 2, 3, 1).cast<float>();
        Vec3 p = view.block(0, 3, 3, 1).cast<float>();
        const float& aa = proj.data()[0];
        const float& bb = proj.data()[5];
        const float& cc = proj.data()[10];
        const float& dd = proj.data()[14];

        float aspect = bb / aa;
        float fov = 2.0f * atan(1.0f / bb);
        float kk = (cc - 1.0f) / (cc + 1.0f);
        float n = (dd * (1.0f - kk)) / (2.0f * kk);
        float f = kk * n;
        SetFromVectors(d, p, right, up, n, f, fov, aspect);
    }

    /**
     * [Frustum::SetFromParams 提取相机的位姿、内参及构建视锥]
     * @param view      [相机位姿]
     * @param nearDist  [最小深度值]
     * @param farDist   [最大深度值]
     * @param fx        [相机内参]
     * @param fy        [相机内参]
     * @param float     [相机内参]
     * @param cy        [相机内参]
     * @param imgWidth  [相机内参]
     * @param imgHeight [相机内参]
     */
    void Frustum::SetFromParams(const Transform& view, float nearDist, float farDist, float fx, float fy, float /*cx*/, float cy, float imgWidth, float imgHeight)
    {
        //! 将旋转矩阵分解为右手坐标系下的三个单位向量，P2 = linear*P1 + t
        Mat3x3 r = view.linear();
        Vec3 right = r.col(0);
        Vec3 up = -r.col(1);
        Vec3 d = r.col(2);
        Vec3 p = view.translation();
        float aspect = (fx * imgWidth) / (fy * imgHeight);
        float fov = atan2(cy, fy) + atan2(imgHeight - cy, fy);
        SetFromVectors(d, p, right, up, nearDist, farDist, fov, aspect);
    }
    /**
     * [Frustum::SetFromVectors 根据位姿和内参构造视锥，主要是确定锥体的六个面]
     * @param forward  [x]
     * @param pos      [相机坐标]
     * @param rightVec [y]
     * @param up       [z]
     * @param nearDist [最近距离]
     * @param farDist  [最远距离]
     * @param fov      [广角]
     * @param aspect   [宽高比]
     * 参考：http://www.lighthouse3d.com/tutorials/view-frustum-culling/geometric-approach-extracting-the-planes/
     */
    void Frustum::SetFromVectors(const Vec3& forward, const Vec3& pos, const Vec3& rightVec, const Vec3& up, float nearDist, float farDist, float fov, float aspect)
    {
        //! 计算near和far平面的宽和高
        float angleTangent = tan(fov / 2);
        float heightFar = angleTangent * farDist;
        float widthFar = heightFar * aspect;
        float heightNear = angleTangent * nearDist;
        float widthNear = heightNear * aspect;

        //! 计算锥体的8个顶点
        Vec3 farCenter = pos + forward * farDist;
        Vec3 farTopLeft = farCenter + (up * heightFar) - (rightVec * widthFar);
        Vec3 farTopRight = farCenter + (up * heightFar) + (rightVec * widthFar);
        Vec3 farBotLeft = farCenter - (up * heightFar) - (rightVec * widthFar);
        Vec3 farBotRight = farCenter - (up * heightFar) + (rightVec * widthFar);

        Vec3 nearCenter = pos + forward * nearDist;
        Vec3 nearTopLeft = nearCenter + (up * heightNear) - (rightVec * widthNear);
        Vec3 nearTopRight = nearCenter + (up * heightNear) + (rightVec * widthNear);
        Vec3 nearBotLeft = nearCenter - (up * heightNear) - (rightVec * widthNear);
        Vec3 nearBotRight = nearCenter - (up * heightNear) + (rightVec * widthNear);

        //! 由上面计算出的8个点得到椎体的6个面
        near = Plane(nearBotLeft, nearTopLeft, nearBotRight);
        far = Plane(farTopRight, farTopLeft, farBotRight);
        left = Plane(farTopLeft, nearTopLeft, farBotLeft);
        right = Plane(nearTopRight, farTopRight, nearBotRight);
        top = Plane(nearTopLeft, farTopLeft, nearTopRight);
        bottom = Plane(nearBotRight, farBotLeft, nearBotLeft);

        corners[0] = farTopLeft.cast<float>();
        corners[1] = farTopRight.cast<float>();
        corners[2] = farBotLeft.cast<float>();
        corners[3] = farBotRight.cast<float>();
        corners[4] = nearBotRight.cast<float>();
        corners[5] = nearTopLeft.cast<float>();
        corners[6] = nearTopRight.cast<float>();
        corners[7] = nearBotLeft.cast<float>();

        //! 确定锥体的12条边界线
        // Far face lines.
        lines[0] = corners[0];
        lines[1] = corners[1];
        lines[2] = corners[3];
        lines[3] = corners[2];
        lines[4] = corners[1];
        lines[5] = corners[3];
        lines[6] = corners[2];
        lines[7] = corners[0];

        // Near face lines.
        lines[8] = corners[4];
        lines[9] = corners[7];
        lines[10] = corners[6];
        lines[11] = corners[5];
        lines[12] = corners[5];
        lines[13] = corners[7];
        lines[14] = corners[6];
        lines[15] = corners[4];

        // Connecting lines.
        lines[16] = corners[0];
        lines[17] = corners[5];
        lines[18] = corners[1];
        lines[19] = corners[6];
        lines[20] = corners[2];
        lines[21] = corners[7];
        lines[22] = corners[3];
        lines[23] = corners[4];
    }


} // namespace chisel 
