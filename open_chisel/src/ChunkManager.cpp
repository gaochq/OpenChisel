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

#include <assert.h>
#include <open_chisel/threading/Threading.h>
#include <open_chisel/ChunkManager.h>
#include <open_chisel/geometry/Frustum.h>
#include <open_chisel/geometry/AABB.h>
#include <open_chisel/marching_cubes/MarchingCubes.h>
#include <open_chisel/geometry/Raycast.h>
#include <open_chisel/ProjectionIntegrator.h>
#include <open_chisel/truncation/Truncator.h>
#include <iostream>

namespace chisel
{

    ChunkManager::ChunkManager() :
            chunkSize(16, 16, 16), voxelResolutionMeters(0.03)
    {
        CacheCentroids();
    }

    ChunkManager::~ChunkManager()
    {

    }

    //! 初始化Chunk的大小、分辨率及颜色渲染标志位
    ChunkManager::ChunkManager(const Eigen::Vector3i& size, float res, bool color) :
            chunkSize(size), voxelResolutionMeters(res), useColor(color)
    {
        CacheCentroids();
    }

    /**
     * [ChunkManager::CacheCentroids 计算每个chunk的中心坐标
     *     统计的是chunk中个voxel的中心坐标
     * ]
     */
    void ChunkManager::CacheCentroids()
    {
        halfVoxel = Vec3(voxelResolutionMeters, voxelResolutionMeters, voxelResolutionMeters) * 0.5f;
        centroids.resize(static_cast<size_t>(chunkSize(0) * chunkSize(1) * chunkSize(2)));
        int i = 0;
        for (int z = 0; z < chunkSize(2); z++)
        {
            for(int y = 0; y < chunkSize(1); y++)
            {
                for(int x = 0; x < chunkSize(0); x++)
                {
                    centroids[i] = Vec3(x, y, z) * voxelResolutionMeters + halfVoxel;
                    i++;
                }
            }
        }

        //! 假设voxel的左下角的顶点坐标为(0,0,0),则矩阵的每一列表示一个顶点
        cubeIndexOffsets << 0, 1, 1, 0, 0, 1, 1, 0,
                            0, 0, 1, 1, 0, 0, 1, 1,
                            0, 0, 0, 0, 1, 1, 1, 1;
    }

    /**
     * [ChunkManager::GetChunkIDsIntersecting 通过AABB box的大小，计算chunk的ID号]
     * @param box       [description]
     * @param chunkList [description]
     */
    void ChunkManager::GetChunkIDsIntersecting(const AABB& box, ChunkIDList* chunkList)
    {
        assert(chunkList != nullptr);

        ChunkID minID = GetIDAt(box.min);
        ChunkID maxID = GetIDAt(box.max) + Eigen::Vector3i(1, 1, 1);

        for (int x = minID(0); x < maxID(0); x++)
        {
            for (int y = minID(1); y < maxID(1); y++)
            {
                for (int z = minID(2); z < maxID(2); z++)
                {
                    chunkList->push_back(ChunkID(x, y, z));
                }
            }
        }
    }


    /**
     * [ChunkManager::RecomputeMesh description]
     * @param chunkID [要计算chunk的ID号,单个chunk]
     * @param mutex   [description]
     */
    void ChunkManager::RecomputeMesh(const ChunkID& chunkID, std::mutex& mutex)
    {
        mutex.lock();

        //! 如果此ID号Chunk不存在，则直接返回
        if (!HasChunk(chunkID))
        {
            mutex.unlock();
            return;
        }

        //! 如果该ID对应的chunk存在，则读取其对应的mesh
        MeshPtr mesh;
        if (!HasMesh(chunkID))
        {
            mesh = std::allocate_shared<Mesh>(Eigen::aligned_allocator<Mesh>());
        }
        else
        {
            mesh = GetMesh(chunkID);
        }

        ChunkPtr chunk = GetChunk(chunkID);
        mutex.unlock();

        //! 生成或重新计算chunk的mesh segment
        GenerateMesh(chunk, mesh.get());

        //! 按照顶点渲染mesh
        if(useColor)
        {
            ColorizeMesh(mesh.get());
        }

        //! 计算mesh的Normal
        ComputeNormalsFromGradients(mesh.get());

        mutex.lock();
        if(!mesh->vertices.empty())
            allMeshes[chunkID] = mesh;
        mutex.unlock();
    }

    /**
     * [ChunkManager::RecomputeMeshes 重新计算mesh]
     * @param chunkMeshes [已经更新过的chunk]
     */
    void ChunkManager::RecomputeMeshes(const ChunkSet& chunkMeshes)
    {
        //! 如果chunkMeshes为空，则直接返回即可
        if (chunkMeshes.empty())
        {
            return;
        }

        //! 根据value判断对应的chunk是否已更新，如果更新则重新计算其对应的mesh即可
        std::mutex mutex;
        for (const std::pair<ChunkID, bool>& chunk : chunkMeshes)
        //parallel_for(chunks.begin(), chunks.end(), [this, &mutex](const ChunkID& chunkID)
        {
            //! 这个地方查询的时候就利用了hash结构的特点，查询的时间复杂度为N(1).
            if (chunk.second)
              this->RecomputeMesh(ChunkID(chunk.first), mutex);
        }

    }

    //! 根据ID号，建立新的chunk,并将其插入到ChunkMap之中
    void ChunkManager::CreateChunk(const ChunkID& id)
    {
        AddChunk(std::allocate_shared<Chunk>(Eigen::aligned_allocator<Chunk>(), id, chunkSize, voxelResolutionMeters, useColor));
    }

    void ChunkManager::Reset()
    {
        allMeshes.clear();
        chunks.clear();
    }

    /**
     * [ChunkManager::GetChunkIDsIntersecting 获取在视椎体中所有的Chunk]
     * @param frustum   [输入的视锥体]
     * @param chunkList [输出的Chunklist]
     */
    void ChunkManager::GetChunkIDsIntersecting(const Frustum& frustum, ChunkIDList* chunkList)
    {
        assert(chunkList != nullptr);

        //! 求取视锥体的最大最小坐标
        AABB frustumAABB;
        frustum.ComputeBoundingBox(&frustumAABB);

        ChunkID minID = GetIDAt(frustumAABB.min);
        ChunkID maxID = GetIDAt(frustumAABB.max) + Eigen::Vector3i(1, 1, 1);

        //printf("FrustumAABB: %f %f %f %f %f %f\n", frustumAABB.min.x(), frustumAABB.min.y(), frustumAABB.min.z(), frustumAABB.max.x(), frustumAABB.max.y(), frustumAABB.max.z());
        //printf("Frustum min: %d %d %d max: %d %d %d\n", minID.x(), minID.y(), minID.z(), maxID.x(), maxID.y(), maxID.z());
        for (int x = minID(0) - 1; x <= maxID(0) + 1; x++)
        {
            for (int y = minID(1) - 1; y <= maxID(1) + 1; y++)
            {
                for (int z = minID(2) - 1; z <= maxID(2) + 1; z++)
                {
                    Vec3 min = Vec3(x * chunkSize(0), y * chunkSize(1), z * chunkSize(2)) * voxelResolutionMeters;
                    Vec3 max = min + chunkSize.cast<float>() * voxelResolutionMeters;
                    AABB chunkBox(min, max);

                    //!  判断该chunk是否被视锥体包含
                    if(frustum.Intersects(chunkBox))
                    {
                        chunkList->push_back(ChunkID(x, y, z));
                    }
                }
            }
        }

        //printf("%lu chunks intersect frustum\n", chunkList->size());
    }

    void ChunkManager::GetChunkIDsIntersecting
    (
            const PointCloud& cloud,
            const Transform& cameraTransform,
            const ProjectionIntegrator& integrator,
            float maxDist,
            ChunkPointMap* chunkList
    )
    {
        assert(!!chunkList);
        chunkList->clear();
        const float roundX = 1.0f / (chunkSize.x() * voxelResolutionMeters);
        const float roundY = 1.0f / (chunkSize.y() * voxelResolutionMeters);
        const float roundZ = 1.0f / (chunkSize.z() * voxelResolutionMeters);
        const TruncatorPtr& truncator = integrator.GetTruncator();

        Point3 minVal(-std::numeric_limits<int>::max(), -std::numeric_limits<int>::max(), -std::numeric_limits<int>::max());
        Point3 maxVal(std::numeric_limits<int>::max(), std::numeric_limits<int>::max(), std::numeric_limits<int>::max());
        size_t numPoints = cloud.GetPoints().size();
        Vec3 start = cameraTransform.translation();
        for (size_t i = 0; i < numPoints; i++)
        {
            const Vec3& point = cloud.GetPoints().at(i);
            Vec3 end = cameraTransform * point;
            float truncation = truncator->GetTruncationDistance(point.z());
            Vec3 dir = (end - start).normalized();
            Vec3 truncEnd = end + dir * truncation;
            Vec3 truncStart = end - dir * truncation;
            Vec3 startInt = Vec3(truncStart.x() * roundX , truncStart.y() * roundY, truncStart.z() * roundZ);
            Vec3 endInt = Vec3(truncEnd.x() * roundX, truncEnd.y() * roundY, truncEnd.z() * roundZ);

            Point3List intersectingChunks;
            Raycast(startInt, endInt, minVal, maxVal, &intersectingChunks);

            for (const Point3& id : intersectingChunks)
            {
                if(chunkList->find(id) == chunkList->end())
                    (*chunkList)[id] = std::vector<size_t>();

                (*chunkList)[id].push_back(i);
            }
        }

    }

    /**
     * [ChunkManager::ExtractInsideVoxelMesh 提取voxel内部的mesh]
     * @param chunk         [包含要提取voxel的chunk]
     * @param index         [voxel对应的ID号]
     * @param coords        [voxel对应的中心坐标]
     * @param nextMeshIndex [下一块mesh的ID]
     * @param mesh          [要生成的triangle mesh segment]
     */
    void ChunkManager::ExtractInsideVoxelMesh(const ChunkPtr& chunk, const Eigen::Vector3i& index, const Vec3& coords, VertIndex* nextMeshIndex, Mesh* mesh)
    {
        assert(mesh != nullptr);
        //! voxel的8个顶点距voxel左下角顶点的距离
        Eigen::Matrix<float, 3, 8> cubeCoordOffsets = cubeIndexOffsets.cast<float>() * voxelResolutionMeters;
        Eigen::Matrix<float, 3, 8> cornerCoords;
        Eigen::Matrix<float, 8, 1> cornerSDF;
        bool allNeighborsObserved = true;

        //! 一次判断该voxel的8个顶点都是否可观测(权重是否大于0)
        for (int i = 0; i < 8; ++i)
        {
            Eigen::Vector3i corner_index = index + cubeIndexOffsets.col(i);
            const DistVoxel& thisVoxel = chunk->GetDistVoxel(corner_index.x(), corner_index.y(), corner_index.z());

            // Do not extract a mesh here if one of the corner is unobserved and
            // outside the truncation region.
            //! 如果这个顶点出的voxel的权重小于0，说明这个顶点不可观测，说明该voxel不能生成mesh，直接返回 
            if (thisVoxel.GetWeight() <= 1e-15)
            {
                allNeighborsObserved = false;
                break;
            }

            //! 获取该voxel8个voxel的中心坐标和距离值
            cornerCoords.col(i) = coords + cubeCoordOffsets.col(i);
            cornerSDF(i) = thisVoxel.GetSDF();
        }
        
        //! 如果所有的8个顶点出的voxel都是可观测的
        if (allNeighborsObserved)
        {
            MarchingCubes::MeshCube(cornerCoords, cornerSDF, nextMeshIndex, mesh);
        }
    }

    /**
     * [ChunkManager::ExtractBorderVoxelMesh description]
     * @param chunk         [要计算mesh segment的chunk]
     * @param index         [voxel对应的ID号]
     * @param coordinates   [voxel的中心坐标]
     * @param nextMeshIndex [下一篇mesh的ID号]
     * @param mesh          [要计算的mesh segment]
     */
    void ChunkManager::ExtractBorderVoxelMesh(const ChunkPtr& chunk, const Eigen::Vector3i& index, const Eigen::Vector3f& coordinates, VertIndex* nextMeshIndex, Mesh* mesh)
    {
        //! voxel的8个顶点距voxel左下角顶点的距离
        const Eigen::Matrix<float, 3, 8> cubeCoordOffsets = cubeIndexOffsets.cast<float>() * voxelResolutionMeters;
        Eigen::Matrix<float, 3, 8> cornerCoords;
        Eigen::Matrix<float, 8, 1> cornerSDF;
        bool allNeighborsObserved = true;


        for (int i = 0; i < 8; ++i)
        {
            //! 获取8个相邻的voxel的ID号
            Eigen::Vector3i cornerIDX = index + cubeIndexOffsets.col(i);

            //！ 如果该voxel包含在chunk之内
            if (chunk->IsCoordValid(cornerIDX.x(), cornerIDX.y(), cornerIDX.z()))
            {
                //! 获取该ID号，对应的voxel
                const DistVoxel& thisVoxel = chunk->GetDistVoxel(cornerIDX.x(), cornerIDX.y(), cornerIDX.z());
                // Do not extract a mesh here if one of the corners is unobserved
                // and outside the truncation region.
                //! 如果该voxel观测权重为0，则直接跳出即可 
                if (thisVoxel.GetWeight() <= 1e-15)
                {
                    allNeighborsObserved = false;
                    break;
                }

                //! 获取相邻8个voxel的坐标和距离值
                cornerCoords.col(i) = coordinates + cubeCoordOffsets.col(i);
                cornerSDF(i) = thisVoxel.GetSDF();
            }
            //! 如果该voxel不在chunk之内，因为该中心voxel位于chunk的某个平面上
            else
            {
                Eigen::Vector3i chunkOffset = Eigen::Vector3i::Zero();

                //! 重新计算不在当前chunk的voxel
                for (int j = 0; j < 3; j++)
                {
                    //! 如果voxel的ID号小于0，说明其是上一个chunk的voxel
                    if (cornerIDX(j) < 0)
                    {
                        chunkOffset(j) = -1;
                        cornerIDX(j) = chunkSize(j) - 1;
                    }
                    //! 如果voxel的ID号大于voxel总数，说明其是上一个chunk的voxel
                    else if(cornerIDX(j) >= chunkSize(j))
                    {
                        chunkOffset(j) = 1;
                        cornerIDX(j) = 0;
                    }
                }

                //! 获取相邻包含上面voxel的chunk的ID号
                ChunkID neighborID = chunkOffset + chunk->GetID();

                //! 如果该chunk存在
                if (HasChunk(neighborID))
                {
                    //! 如果新得到的chunk不包含该voxel，则直接跳出
                    const ChunkPtr& neighborChunk = GetChunk(neighborID);
                    if(!neighborChunk->IsCoordValid(cornerIDX.x(), cornerIDX.y(), cornerIDX.z()))
                    {
                        allNeighborsObserved = false;
                        break;
                    }

                    //! 剩下的与上面相同
                    const DistVoxel& thisVoxel = neighborChunk->GetDistVoxel(cornerIDX.x(), cornerIDX.y(), cornerIDX.z());
                    // Do not extract a mesh here if one of the corners is unobserved
                    // and outside the truncation region.
                    if (thisVoxel.GetWeight() <= 1e-15)
                    {
                        allNeighborsObserved = false;
                        break;
                    }
                    cornerCoords.col(i) = coordinates + cubeCoordOffsets.col(i);
                    cornerSDF(i) = thisVoxel.GetSDF();
                }
                //! 如果该chunk不存在，则直接跳出即可
                else
                {
                    allNeighborsObserved = false;
                    break;
                }

            }

        }

        if (allNeighborsObserved)
        {
            MarchingCubes::MeshCube(cornerCoords, cornerSDF, nextMeshIndex, mesh);
        }
    }

    /**
     * [ChunkManager::GenerateMesh 由chunk生成mesh]
     * @param chunk [输入的chunk]
     * @param mesh  [该chunk对应的mesh segment]
     */
    void ChunkManager::GenerateMesh(const ChunkPtr& chunk, Mesh* mesh)
    {
        assert(mesh != nullptr);

        mesh->Clear();
        const int maxX = chunkSize(0);
        const int maxY = chunkSize(1);
        const int maxZ = chunkSize(2);


        Eigen::Vector3i index;
        VoxelID i = 0;
        VertIndex nextIndex = 0;

        // For voxels not bordering the outside, we can use a more efficient function.
        //! 提取chunk内部所有的voxel，除了轴向坐标最大的几个面，然后计算chunk对应的mesh segment
        for (index.z() = 0; index.z() < maxZ - 1; index.z()++)
        {
            for (index.y() = 0; index.y() < maxY - 1; index.y()++)
            {
                for (index.x() = 0; index.x() < maxX - 1; index.x()++)
                {
                    //! 获取voxel的ID号
                    i = chunk->GetVoxelID(index.x(), index.y(), index.z());
                    ExtractInsideVoxelMesh(chunk, index, centroids.at(i) + chunk->GetOrigin(), &nextIndex, mesh);
                }
            }
        }

        //! 这个地方为什么要计算三个轴向坐标最大的面
        // Max X plane (takes care of max-Y corner as well).
        //! 由x轴方向最大的面计算chunk的mesh segment
        i = 0;
        index.x() = maxX - 1;
        for (index.z() = 0; index.z() < maxZ - 1; index.z()++)
        {
            for (index.y() = 0; index.y() < maxY; index.y()++)
            {
                i = chunk->GetVoxelID(index.x(), index.y(), index.z());
                ExtractBorderVoxelMesh(chunk, index, centroids.at(i) + chunk->GetOrigin(), &nextIndex, mesh);
            }
        }

        // Max Y plane.
        i = 0;
        index.y() = maxY - 1;
        for (index.z() = 0; index.z() < maxZ - 1; index.z()++)
        {
            for (index.x() = 0; index.x() < maxX - 1; index.x()++)
            {
                i = chunk->GetVoxelID(index.x(), index.y(), index.z());
                ExtractBorderVoxelMesh(chunk, index, centroids.at(i) + chunk->GetOrigin(), &nextIndex, mesh);
            }
        }

        // Max Z plane (also takes care of corners).
        i = 0;
        index.z() = maxZ - 1;
        for (index.y() = 0; index.y() < maxY; index.y()++)
        {
            for (index.x() = 0; index.x() < maxX; index.x()++)
            {
                i = chunk->GetVoxelID(index.x(), index.y(), index.z());
                ExtractBorderVoxelMesh(chunk, index, centroids.at(i) + chunk->GetOrigin(), &nextIndex, mesh);
            }
        }

        //printf("Generated a new mesh with %lu verts, %lu norm, and %lu idx\n", mesh->vertices.size(), mesh->normals.size(), mesh->indices.size());

        assert(mesh->vertices.size() == mesh->normals.size());
        assert(mesh->vertices.size() == mesh->indices.size());
    }

    /**
     * [ChunkManager::GetSDFAndGradient 求取该顶点处的截断距离值和距离值梯度]
     * @param  pos  [顶点坐标]
     * @param  dist [距离]
     * @param  grad [梯度]
     * @return      [description]
     */
    bool ChunkManager::GetSDFAndGradient(const Eigen::Vector3f& pos, double* dist, Eigen::Vector3f* grad)
    {
        //! 求取顶点坐标四舍五入后所在voxel
        Eigen::Vector3f posf = Eigen::Vector3f(std::floor(pos.x() / voxelResolutionMeters) * voxelResolutionMeters + voxelResolutionMeters / 2.0f,
                std::floor(pos.y() / voxelResolutionMeters) * voxelResolutionMeters + voxelResolutionMeters / 2.0f,
                std::floor(pos.z() / voxelResolutionMeters) * voxelResolutionMeters + voxelResolutionMeters / 2.0f);

        if (!GetSDF(posf, dist)) return false;
        double ddxplus, ddyplus, ddzplus = 0.0;
        double ddxminus, ddyminus, ddzminus = 0.0;
        //! 求取该voxel的相邻的坐标差值为分辨率的voxel的截断距离值
        if (!GetSDF(posf + Eigen::Vector3f(voxelResolutionMeters, 0, 0), &ddxplus)) return false;
        if (!GetSDF(posf + Eigen::Vector3f(0, voxelResolutionMeters, 0), &ddyplus)) return false;
        if (!GetSDF(posf + Eigen::Vector3f(0, 0, voxelResolutionMeters), &ddzplus)) return false;
        if (!GetSDF(posf - Eigen::Vector3f(voxelResolutionMeters, 0, 0), &ddxminus)) return false;
        if (!GetSDF(posf - Eigen::Vector3f(0, voxelResolutionMeters, 0), &ddyminus)) return false;
        if (!GetSDF(posf - Eigen::Vector3f(0, 0, voxelResolutionMeters), &ddzminus)) return false;

        //! 求取距离的梯度(和求取图像梯度的思路一致)，并做归一化
        *grad = Eigen::Vector3f(ddxplus - ddxminus, ddyplus - ddyminus, ddzplus - ddzminus);
        grad->normalize();
        return true;
    }

    /**
     * [ChunkManager::GetSDF 求取voxel处的阶段距离值]
     * @param  posf [voxel所在的坐标]
     * @param  dist [得到的距离]
     * @return      [description]
     */
    bool ChunkManager::GetSDF(const Eigen::Vector3f& posf, double* dist)
    {
        chisel::ChunkPtr chunk = GetChunkAt(posf);
        if(chunk)
        {
            Eigen::Vector3f relativePos = posf - chunk->GetOrigin();
            Eigen::Vector3i coords = chunk->GetVoxelCoords(relativePos);
            chisel::VoxelID id = chunk->GetVoxelID(coords);
            if(id >= 0 && id < chunk->GetTotalNumVoxels())
            {
                const chisel::DistVoxel& voxel = chunk->GetDistVoxel(id);
                if(voxel.GetWeight() > 1e-12)
                {
                    *dist = voxel.GetSDF();
                    return true;
                }
            }
            return false;
        }
        else
        {
            return false;
        }
    }

    /**
     * [ChunkManager::InterpolateColor 为mesh的顶点渲染颜色]
     * @param  colorPos [顶点坐标]
     * @return          [description]
     */
    Vec3 ChunkManager::InterpolateColor(const Vec3& colorPos)
    {
        //! 根据顶点的坐标，计算到顶点坐标的voxel个数
        const float& x = colorPos(0);
        const float& y = colorPos(1);
        const float& z = colorPos(2);
        //! std::floor,四舍五入取整数
        const int x_0 = static_cast<int>(std::floor(x / voxelResolutionMeters));
        const int y_0 = static_cast<int>(std::floor(y / voxelResolutionMeters));
        const int z_0 = static_cast<int>(std::floor(z / voxelResolutionMeters));
        const int x_1 = x_0 + 1;
        const int y_1 = y_0 + 1;
        const int z_1 = z_0 + 1;

        //! 该voxel右上角8领域内的共8个voxel
        const ColorVoxel* v_000 = GetColorVoxel(Vec3(x_0, y_0, z_0));
        const ColorVoxel* v_001 = GetColorVoxel(Vec3(x_0, y_0, z_1));
        const ColorVoxel* v_011 = GetColorVoxel(Vec3(x_0, y_1, z_1));
        const ColorVoxel* v_111 = GetColorVoxel(Vec3(x_1, y_1, z_1));
        const ColorVoxel* v_110 = GetColorVoxel(Vec3(x_1, y_1, z_0));
        const ColorVoxel* v_100 = GetColorVoxel(Vec3(x_1, y_0, z_0));
        const ColorVoxel* v_010 = GetColorVoxel(Vec3(x_0, y_1, z_0));
        const ColorVoxel* v_101 = GetColorVoxel(Vec3(x_1, y_0, z_1));

        //! 如果有一个ColorVoxel不存在，则由chunk出发，获取颜色
        if(!v_000 || !v_001 || !v_011 || !v_111 || !v_110 || !v_100 || !v_010 || !v_101)
        {
            const ChunkID& chunkID = GetIDAt(colorPos);

            if(!HasChunk(chunkID))
            {
                return Vec3(0, 0, 0);
            }
            else
            {
                const ChunkPtr& chunk = GetChunk(chunkID);
                return chunk->GetColorAt(colorPos);
            }
        }

        //! 坐标减去voxel的个数是什么鬼
        float xd = (x - x_0) / (x_1 - x_0);
        float yd = (y - y_0) / (y_1 - y_0);
        float zd = (z - z_0) / (z_1 - z_0);

        //! 这个权重关系看不懂啊。。。
        float red, green, blue = 0.0f;
        {
            float c_00 = v_000->GetRed() * (1 - xd) + v_100->GetRed() * xd;
            float c_10 = v_010->GetRed() * (1 - xd) + v_110->GetRed() * xd;
            float c_01 = v_001->GetRed() * (1 - xd) + v_101->GetRed() * xd;
            float c_11 = v_011->GetRed() * (1 - xd) + v_111->GetRed() * xd;
            float c_0 = c_00 * (1 - yd) + c_10 * yd;
            float c_1 = c_01 * (1 - yd) + c_11 * yd;
            float c = c_0 * (1 - zd) + c_1 * zd;
            red = c / 255.0f;
        }
        {
            float c_00 = v_000->GetGreen() * (1 - xd) + v_100->GetGreen() * xd;
            float c_10 = v_010->GetGreen() * (1 - xd) + v_110->GetGreen() * xd;
            float c_01 = v_001->GetGreen() * (1 - xd) + v_101->GetGreen() * xd;
            float c_11 = v_011->GetGreen() * (1 - xd) + v_111->GetGreen() * xd;
            float c_0 = c_00 * (1 - yd) + c_10 * yd;
            float c_1 = c_01 * (1 - yd) + c_11 * yd;
            float c = c_0 * (1 - zd) + c_1 * zd;
            green = c / 255.0f;
        }
        {
            float c_00 = v_000->GetBlue() * (1 - xd) + v_100->GetBlue()  * xd;
            float c_10 = v_010->GetBlue() * (1 - xd) + v_110->GetBlue()  * xd;
            float c_01 = v_001->GetBlue() * (1 - xd) + v_101->GetBlue()  * xd;
            float c_11 = v_011->GetBlue() * (1 - xd) + v_111->GetBlue()  * xd;
            float c_0 = c_00 * (1 - yd) + c_10 * yd;
            float c_1 = c_01 * (1 - yd) + c_11 * yd;
            float c = c_0 * (1 - zd) + c_1 * zd;
            blue = c / 255.0f;
         }

        return Vec3(red, green, blue);
    }

    const DistVoxel* ChunkManager::GetDistanceVoxel(const Vec3& pos)
    {
        ChunkPtr chunk = GetChunkAt(pos);

        if(chunk.get())
        {
            Vec3 rel = (pos - chunk->GetOrigin());
            return &(chunk->GetDistVoxel(chunk->GetVoxelID(rel)));
        }
        else return nullptr;
    }

    /**
     * [ChunkManager::GetColorVoxel 获取voxel的颜色]
     * @param  pos [voxel的坐标]
     * @return     [description]
     */
    const ColorVoxel* ChunkManager::GetColorVoxel(const Vec3& pos)
    {
        ChunkPtr chunk = GetChunkAt(pos);

        if(chunk.get())
        {
            //! 获取该voxel的ID
            Vec3 rel = (pos - chunk->GetOrigin());
            const VoxelID& id = chunk->GetVoxelID(rel);
            if (id >= 0 && id < chunk->GetTotalNumVoxels())
            {
                return &(chunk->GetColorVoxel(id));
            }
            else
            {
                return nullptr;
            }
        }
        else return nullptr;
    }

    /**
     * [ChunkManager::ComputeNormalsFromGradients 通过mesh的顶点计算mesh的normal]
     * @param mesh [输入的mesh片]
     */
    void ChunkManager::ComputeNormalsFromGradients(Mesh* mesh)
    {
        assert(mesh != nullptr);
        double dist;
        Vec3 grad;
        for (size_t i = 0; i < mesh->vertices.size(); i++)
        {
            const Vec3& vertex = mesh->vertices.at(i);
            if(GetSDFAndGradient(vertex, &dist, &grad))
            {
                //! grad已经做了归一化，那么这个地方的mag=1？
                float mag = grad.norm();
                if(mag> 1e-12)
                {
                    mesh->normals[i] = grad * (1.0f / mag);
                }
            }
        }
    }

    /**
     * [ChunkManager::ColorizeMesh 为mesh渲染颜色]
     * @param mesh [要渲染的mesh]
     */
    void ChunkManager::ColorizeMesh(Mesh* mesh)
    {
        assert(mesh != nullptr);

        //! 按照顶点渲染
        mesh->colors.clear();
        mesh->colors.resize(mesh->vertices.size());
        for (size_t i = 0; i < mesh->vertices.size(); i++)
        {

            const Vec3& vertex = mesh->vertices.at(i);
            mesh->colors[i] = InterpolateColor(vertex);
        }
    }


    void ChunkManager::PrintMemoryStatistics()
    {
        float bigFloat = std::numeric_limits<float>::max();

        chisel::AABB totalBounds;
        totalBounds.min = chisel::Vec3(bigFloat, bigFloat, bigFloat);
        totalBounds.max = chisel::Vec3(-bigFloat, -bigFloat, -bigFloat);

        ChunkStatistics stats;
        stats.numKnownInside = 0;
        stats.numKnownOutside = 0;
        stats.numUnknown = 0;
        stats.totalWeight = 0.0f;
        for (const std::pair<ChunkID, ChunkPtr>& chunk : chunks)
        {
            AABB bounds = chunk.second->ComputeBoundingBox();
            for (int i = 0; i < 3; i++)
            {
                totalBounds.min(i) = std::min(totalBounds.min(i), bounds.min(i));
                totalBounds.max(i) = std::max(totalBounds.max(i), bounds.max(i));
            }

            chunk.second->ComputeStatistics(&stats);
        }


        Vec3 ext = totalBounds.GetExtents();
        Vec3 numVoxels = ext * 2 / voxelResolutionMeters;
        float totalNum = numVoxels(0) * numVoxels(1) * numVoxels(2);

        float maxMemory = totalNum * sizeof(DistVoxel) / 1000000.0f;

        size_t currentNum = chunks.size() * (chunkSize(0) * chunkSize(1) * chunkSize(2));
        float currentMemory = currentNum * sizeof(DistVoxel) / 1000000.0f;

        printf("Num Unknown: %lu, Num KnownIn: %lu, Num KnownOut: %lu Weight: %f\n", stats.numUnknown, stats.numKnownInside, stats.numKnownOutside, stats.totalWeight);
        printf("Bounds: %f %f %f %f %f %f\n", totalBounds.min.x(), totalBounds.min.y(), totalBounds.min.z(), totalBounds.max.x(), totalBounds.max.y(), totalBounds.max.z());
        printf("Theoretical max (MB): %f, Current (MB): %f\n", maxMemory, currentMemory);

    }

} // namespace chisel 
