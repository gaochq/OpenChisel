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

#ifndef MARCHINGCUBES_H_
#define MARCHINGCUBES_H_

#include <open_chisel/geometry/Geometry.h>
#include <open_chisel/mesh/Mesh.h>

namespace chisel
{

    typedef std::vector<Mat3x3, Eigen::aligned_allocator<Mat3x3> > TriangleVector;
    class MarchingCubes
    //! 这部分内容参考：论文，Lorensen W E, Cline H E. Marching cubes: A high resolution 3D surface construction algorithm
    //! 博客：http://www.cnblogs.com/shushen/p/5542131.html
    //!       该方法的缺陷：http://users.polytech.unice.fr/~lingrand/MarchingCubes/algo.html
    {
        public:
            static int triangleTable[256][16];
            static int edgeIndexPairs[12][2];

            MarchingCubes();
            virtual ~MarchingCubes();


            static void MeshCube(const Eigen::Matrix<float, 3, 8>& vertex_coordinates, const Eigen::Matrix<float, 8, 1>& vertexSDF, TriangleVector* triangles)
            {
                assert(triangles != nullptr);

                const int index = CalculateVertexConfiguration(vertexSDF);

                Eigen::Matrix<float, 3, 12> edgeCoords;
                InterpolateEdgeVertices(vertex_coordinates, vertexSDF, &edgeCoords);

                const int* table_row = triangleTable[index];

                int edgeIDX = 0;
                int tableCol = 0;
                while ((edgeIDX = table_row[tableCol]) != -1)
                {
                    Eigen::Matrix3f triangle;
                    triangle.col(0) = edgeCoords.col(edgeIDX);
                    edgeIDX = table_row[tableCol + 1];
                    triangle.col(1) = edgeCoords.col(edgeIDX);
                    edgeIDX = table_row[tableCol + 2];
                    triangle.col(2) = edgeCoords.col(edgeIDX);
                    triangles->push_back(triangle);
                    tableCol += 3;
                }
            }

            /**
             * [MeshCube 生成或重新计算triangle mesh segment]
             * @param vertexCoords [以某个voxel为中心的8个voxel的中心坐标]
             * @param vertexSDF    [以某个voxel为中心的8个voxel的包含的距离值]
             * @param nextIDX      [下一片mesh的ID号]
             * @param mesh         [要计算的triangle mesh segment]
             */
            static void MeshCube(const Eigen::Matrix<float, 3, 8>& vertexCoords, const Eigen::Matrix<float, 8, 1>& vertexSDF, VertIndex* nextIDX, Mesh* mesh)
            {
                assert(nextIDX != nullptr);
                assert(mesh != nullptr);

                //! 得到8个距离符号值的编码
                const int index = CalculateVertexConfiguration(vertexSDF);

                Eigen::Matrix<float, 3, 12> edge_vertex_coordinates;
                InterpolateEdgeVertices(vertexCoords, vertexSDF, &edge_vertex_coordinates);

                //! 根据8个相邻voxel的距离符号，查询属于哪一种cube configuration，参见MarchingCubes.cpp
                const int* table_row = triangleTable[index];

                //! 参见论文III-I部分
                int table_col = 0;
                while (table_row[table_col] != -1)
                {
                    //! emplace_back的性能要优于push_back，但是emplace_back对构造函数有特殊要求。
                    //! 这个地方table_row的非“-1”的值，应该和edge_vertex_coordinates的值能对应上的。
                    mesh->vertices.emplace_back(edge_vertex_coordinates.col(table_row[table_col + 2]));
                    mesh->vertices.emplace_back(edge_vertex_coordinates.col(table_row[table_col + 1]));
                    mesh->vertices.emplace_back(edge_vertex_coordinates.col(table_row[table_col]));
                    
                    mesh->indices.push_back(*nextIDX);
                    mesh->indices.push_back((*nextIDX) + 1);
                    mesh->indices.push_back((*nextIDX) + 2);

                    const Eigen::Vector3f& p0 = mesh->vertices[*nextIDX];
                    const Eigen::Vector3f& p1 = mesh->vertices[*nextIDX + 1];
                    const Eigen::Vector3f& p2 = mesh->vertices[*nextIDX + 2];

                    //! 计算由三个顶点构成面的法线
                    Eigen::Vector3f px = (p1 - p0);
                    Eigen::Vector3f py = (p2 - p0);
                    Eigen::Vector3f n = px.cross(py).normalized();

                    //！ 存入mesh面片的法向量,法向量和mesh片顶点是对应的
                    mesh->normals.push_back(n);
                    mesh->normals.push_back(n);
                    mesh->normals.push_back(n);
                    *nextIDX += 3;
                    table_col += 3;
                }
            }

            /**
             * [CalculateVertexConfiguration 将8个voxel的距离值符号用0-256代表，看做将8个距离值符号顺序编码]
             * @param  vertexSDF [输入的8个距离值]
             * @return           [编码输出]
             */
            static int CalculateVertexConfiguration(const Eigen::Matrix<float, 8, 1>& vertexSDF)
            {
                return  (vertexSDF(0) < 0 ? (1<<0) : 0) |
                        (vertexSDF(1) < 0 ? (1<<1) : 0) |
                        (vertexSDF(2) < 0 ? (1<<2) : 0) |
                        (vertexSDF(3) < 0 ? (1<<3) : 0) |
                        (vertexSDF(4) < 0 ? (1<<4) : 0) |
                        (vertexSDF(5) < 0 ? (1<<5) : 0) |
                        (vertexSDF(6) < 0 ? (1<<6) : 0) |
                        (vertexSDF(7) < 0 ? (1<<7) : 0);
            }

            /**
             * [InterpolateEdgeVertices description]
             * @param vertexCoords [相邻的8个voxel的中心坐标]
             * @param vertSDF      [相邻的8个voxel包含的距离值]
             * @param edgeCoords   [要生成的12条边]
             */
            static void InterpolateEdgeVertices(const Eigen::Matrix<float, 3, 8>& vertexCoords, const Eigen::Matrix<float, 8, 1>& vertSDF, Eigen::Matrix<float, 3, 12>* edgeCoords)
            {
                assert(edgeCoords != nullptr);
                for (std::size_t i = 0; i < 12; ++i)
                {
                    const int* pairs = edgeIndexPairs[i];
                    const int edge0 = pairs[0];
                    const int edge1 = pairs[1];
                    // Only interpolate along edges where there is a zero crossing.
                    //! 如果这两个voxel的距离符号刚好相反
                    if ((vertSDF(edge0) < 0 && vertSDF(edge1) >= 0) || (vertSDF(edge0) >= 0 && vertSDF(edge1) < 0))
                        edgeCoords->col(i) = InterpolateVertex(vertexCoords.col(edge0), vertexCoords.col(edge1), vertSDF(edge0), vertSDF(edge1));
                }
            }

            // Performs linear interpolation on two cube corners to find the approximate
            // zero crossing (surface) value.
            /**
             * [InterpolateVertex 由两个voxel(顶点)的坐标和距离值计算连接这两个顶点(voxel)边的坐标]
             * @param  vertex1 [顶点1]
             * @param  vertex2 [顶点2]
             * @param  sdf1    [距离值1]
             * @param  sdf2    [距离值2]
             * @return         [边的坐标]
             */
            static inline Vec3 InterpolateVertex(const Vec3& vertex1, const Vec3& vertex2, const float& sdf1, const float& sdf2)
            {
                const float minDiff = 1e-6;
                const float sdfDiff = sdf1 - sdf2;

                //! 两个voxel距离值非常接近，但是符号不一致
                if (fabs(sdfDiff) < minDiff)
                {
                    return Vec3(vertex1 + 0.5 * vertex2);
                }
                //! 如果两个voxel距离值相差较大
                const float t = sdf1 / sdfDiff;
                return Vec3(vertex1 + t * (vertex2 - vertex1));
            }
    };

} // namespace chisel 

#endif // MARCHINGCUBES_H_ 
