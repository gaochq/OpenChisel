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

#ifndef CHISEL_H_
#define CHISEL_H_

#include <open_chisel/threading/Threading.h>
#include <open_chisel/ChunkManager.h>
#include <open_chisel/ProjectionIntegrator.h>
#include <open_chisel/geometry/Geometry.h>
#include <open_chisel/camera/PinholeCamera.h>
#include <open_chisel/camera/DepthImage.h>
#include <open_chisel/geometry/Frustum.h>
#include <open_chisel/pointcloud/PointCloud.h>

namespace chisel
{

    class Chisel
    {
        public:
            Chisel();
            Chisel(const Eigen::Vector3i& chunkSize, float voxelResolution, bool useColor);
            virtual ~Chisel();

            inline const ChunkManager& GetChunkManager() const { return chunkManager; }
            inline ChunkManager& GetMutableChunkManager() { return chunkManager; }
            inline void SetChunkManager(const ChunkManager& manager) { chunkManager = manager; }

            void IntegratePointCloud(const ProjectionIntegrator& integrator,
                                     const PointCloud& cloud,
                                     const Transform& extrinsic,
                                     float maxDist);

            /**
             * [IntegrateDepthScan 根据当前时刻得到的深度图和相机位姿，对视锥体中的voxel进行更新]
             * @param integrator [description]
             * @param depthImage [上一帧深度图的信息]
             * @param extrinsic  [相机位姿]
             * @param camera     [相机模型(内参)]
             */
            template <class DataType> void IntegrateDepthScan(const ProjectionIntegrator& integrator,
                                                              const std::shared_ptr<const DepthImage<DataType> >& depthImage,
                                                              const Transform& extrinsic,
                                                              const PinholeCamera& camera)
            {
                    printf("CHISEL: Integrating a scan\n");

                    //! Step1: 获取深度图信息
                    DataType minimum, maximum, mean;
                    depthImage->GetStats(minimum, maximum, mean);

                    //! Step2: 构造视锥体
                    Frustum frustum;
                    PinholeCamera cameraCopy = camera;
                    cameraCopy.SetNearPlane(static_cast<float>(minimum));
                    cameraCopy.SetFarPlane(static_cast<float>(maximum));

                    cameraCopy.SetupFrustum(extrinsic, &frustum);

                    //! Step3: 获取在视锥体内部和视锥体相交的Chunks
                    ChunkIDList chunksIntersecting;
                    chunkManager.GetChunkIDsIntersecting(frustum, &chunksIntersecting);

                    //! Step4: 综合处理视锥体中的各个Chunk
                    std::mutex mutex;
                    ChunkIDList garbageChunks;
                    for(const ChunkID& chunkID : chunksIntersecting)
                    //parallel_for(chunksIntersecting.begin(), chunksIntersecting.end(), [&](const ChunkID& chunkID)
                    {
                        bool chunkNew = false;

                        //! Step4.1 如果该ID对应的Chunk不存在，则创建新的chunk; 若存在则直接提取即可
                        mutex.lock();
                        if (!chunkManager.HasChunk(chunkID))
                        {
                           chunkNew = true;
                           chunkManager.CreateChunk(chunkID);
                        }

                        ChunkPtr chunk = chunkManager.GetChunk(chunkID);
                        mutex.unlock();

                        //! Step4.2 根据深度图和相机位姿更新chunk中的所有voxel，并判断该chunk是否被更新
                        bool needsUpdate = integrator.Integrate(depthImage, camera, extrinsic, chunk.get());

                        //! 若上面的chunk被更新，则将包括其和其对应的上下左右等27个chunk对应的mesh置为需要更新
                        mutex.lock();
                        if (needsUpdate)
                        {
                            for (int dx = -1; dx <= 1; dx++)
                            {
                                for (int dy = -1; dy <= 1; dy++)
                                {
                                    for (int dz = -1; dz <= 1; dz++)
                                    {
                                        meshesToUpdate[chunkID + ChunkID(dx, dy, dz)] = true;
                                    }
                                }
                            }
                        }

                        //! 如果该chunk不存在，则将其加入到garbageChunks中
                        else if(chunkNew)
                        {
                            garbageChunks.push_back(chunkID);
                        }
                        mutex.unlock();
                    }
                    //);
                    printf("CHISEL: Done with scan\n");
                    GarbageCollect(garbageChunks);
                    //chunkManager.PrintMemoryStatistics();
            }

            /**
             * [IntegrateDepthScanColor 对视锥体中的voxel进行更新，并加以颜色渲染]
             * @param integrator     [description]
             * @param depthImage     [description]
             * @param depthExtrinsic [description]
             * @param depthCamera    [description]
             * @param colorImage     [description]
             * @param colorExtrinsic [description]
             * @param colorCamera    [description]
             */
            template <class DataType, class ColorType> void IntegrateDepthScanColor(const ProjectionIntegrator& integrator, const std::shared_ptr<const DepthImage<DataType> >& depthImage,  
                                                                                    const Transform& depthExtrinsic, const PinholeCamera& depthCamera, 
                                                                                    const std::shared_ptr<const ColorImage<ColorType> >& colorImage, 
                                                                                    const Transform& colorExtrinsic, const PinholeCamera& colorCamera)
            {
                    //! Step1: 构造视锥体
                    Frustum frustum;
                    depthCamera.SetupFrustum(depthExtrinsic, &frustum);

                    //! Step2: 获取在视锥体内部和视锥体相交的Chunks
                    ChunkIDList chunksIntersecting;
                    chunkManager.GetChunkIDsIntersecting(frustum, &chunksIntersecting);

                    std::mutex mutex;
                    ChunkIDList garbageChunks;
                    //for ( const ChunkID& chunkID : chunksIntersecting)
                    ////! Step3: 综合处理视锥体中的各个Chunk
                    parallel_for(chunksIntersecting.begin(), chunksIntersecting.end(), [&](const ChunkID& chunkID)
                    {

                        mutex.lock();
                        //! Step3.1 如果该ID对应的Chunk不存在，则创建新的chunk; 若存在则直接提取即可
                        bool chunkNew = false;
                        if (!chunkManager.HasChunk(chunkID))
                        {
                           chunkNew = true;
                           chunkManager.CreateChunk(chunkID);
                        }

                        ChunkPtr chunk = chunkManager.GetChunk(chunkID);
                        mutex.unlock();

                        //! Step4.2 根据深度图、彩色图以及相机位姿更新chunk中的所有voxel，并判断该chunk是否被更新
                        bool needsUpdate = integrator.IntegrateColor(depthImage, depthCamera, depthExtrinsic, colorImage, colorCamera, colorExtrinsic, chunk.get());

                        mutex.lock();
                        if (needsUpdate)
                        {
                            for (int dx = -1; dx <= 1; dx++)
                            {
                                for (int dy = -1; dy <= 1; dy++)
                                {
                                    for (int dz = -1; dz <= 1; dz++)
                                    {
                                        meshesToUpdate[chunkID + ChunkID(dx, dy, dz)] = true;
                                    }
                                }
                            }
                        }
                        else if(chunkNew)
                        {
                            garbageChunks.push_back(chunkID);
                        }
                        mutex.unlock();
                    }
                    );

                    GarbageCollect(garbageChunks);
                    //chunkManager.PrintMemoryStatistics();
            }

            void GarbageCollect(const ChunkIDList& chunks);
            void UpdateMeshes();

            bool SaveAllMeshesToPLY(const std::string& filename);
            void Reset();

            const ChunkSet& GetMeshesToUpdate() const { return meshesToUpdate; }

        protected:
            ChunkManager chunkManager;
            ChunkSet meshesToUpdate;

    };
    typedef std::shared_ptr<Chisel> ChiselPtr;
    typedef std::shared_ptr<const Chisel> ChiselConstPtr;

} // namespace chisel 

#endif // CHISEL_H_ 
