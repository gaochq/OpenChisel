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

#ifndef PROJECTIONINTEGRATOR_H_
#define PROJECTIONINTEGRATOR_H_

#include <open_chisel/pointcloud/PointCloud.h>
#include <open_chisel/geometry/Geometry.h>
#include <open_chisel/geometry/Frustum.h>
#include <open_chisel/geometry/AABB.h>
#include <open_chisel/camera/PinholeCamera.h>
#include <open_chisel/camera/DepthImage.h>
#include <open_chisel/camera/ColorImage.h>
#include <open_chisel/Chunk.h>

#include <open_chisel/truncation/Truncator.h>
#include <open_chisel/weighting/Weighter.h>

namespace chisel
{

    class ProjectionIntegrator
    {
        public:
            ProjectionIntegrator();
            ProjectionIntegrator(const TruncatorPtr& t,
                                 const WeighterPtr& w,
                                 float carvingDist,
                                 bool enableCarving,
                                 const Vec3List& centroids);

            virtual ~ProjectionIntegrator();

            bool Integrate(const PointCloud& cloud, const Transform& cameraPose, Chunk* chunk, const std::vector<size_t>& idx) const;
            bool IntegratePointCloud(const PointCloud& cloud, const Transform& cameraPose, Chunk* chunk,  const std::vector<size_t>& idx) const;
            bool IntegrateColorPointCloud(const PointCloud& cloud, const Transform& cameraPose, Chunk* chunk,  const std::vector<size_t>& idx) const;

            /**
             * [Integrate ]
             * @param  depthImage [深度图]
             * @param  camera     [相机模型]
             * @param  cameraPose [相机位姿]
             * @param  chunk      [需要判断的Chubk]
             * @return            [只要chunk中有一个voxel被更新了，则chunk已被更新，返回真值]
             */
            template<class DataType> bool Integrate(const std::shared_ptr<const DepthImage<DataType> >& depthImage,
                                                    const PinholeCamera& camera,
                                                    const Transform& cameraPose, Chunk* chunk) const
            {
                assert(chunk != nullptr);
                //! Step1：获取Chunk中包含的voxels个数、分辨率以及Chunk的初始坐标
                Eigen::Vector3i numVoxels = chunk->GetNumVoxels();
                float resolution = chunk->GetVoxelResolutionMeters();
                Vec3 origin = chunk->GetOrigin();

                //! Step2: 根据论文算法1求取截断距离函数
                float diag = 2.0 * sqrt(3.0f) * resolution;
                Vec3 voxelCenter;
                bool updated = false;
                for (size_t i = 0; i < centroids.size(); i++)
                {
                    //！Step2.1：将voxel从世界坐标系转到相机坐标系，再转到相机平面上，得到像素点cameraPos，并判断该像素点是否有效
                    voxelCenter = centroids[i] + origin;
                    Vec3 voxelCenterInCamera = cameraPose.linear().transpose() * (voxelCenter - cameraPose.translation());
                    Vec3 cameraPos = camera.ProjectPoint(voxelCenterInCamera);

                    if (!camera.IsPointOnImage(cameraPos) || voxelCenterInCamera.z() < 0)
                        continue;

                    //! Step2.2：求取该像素点的深度
                    float voxelDist = voxelCenterInCamera.z();
                    float depth = depthImage->DepthAt((int)cameraPos(1), (int)cameraPos(0)); //depthImage->BilinearInterpolateDepth(cameraPos(0), cameraPos(1));

                    if(std::isnan(depth))
                    {
                        continue;
                    }

                    //！Step2.3：求取该voxel到物体表面的距离(由截断距离的二次型计算)
                    //!          对应论文算法1中第5步
                    float truncation = truncator->GetTruncationDistance(depth);

                    //! Step2.4: 求取截断函数距离值，对应论文算法1中的u
                    float surfaceDist = depth - voxelDist;

                    //! Step2.5: 判断该voxel是否处于hit region
                    if (fabs(surfaceDist) < truncation + diag)
                    {   
                        //! 由Chunk中voxel的索引获取voxel
                        DistVoxel& voxel = chunk->GetDistVoxelMutable(i);
                        voxel.Integrate(surfaceDist, 1.0f);
                        updated = true;
                    }

                    //! Step2.6: 判断该voxel是否处于space carving region
                    //! 对应论文算法1的第7步-->第10步
                    //! carvingDist = 0.05，那么当resolution的大小和carvingDist相等时，这部分就是bug。。。
                    else if (enableVoxelCarving && surfaceDist > truncation + carvingDist)
                    {
                        DistVoxel& voxel = chunk->GetDistVoxelMutable(i);
                        if (voxel.GetWeight() > 0 && voxel.GetSDF() < 1e-5)
                        {
                            voxel.Carve();
                            updated = true;
                        }
                    }


                }
                return updated;
            }
            template<class DataType, class ColorType> bool IntegrateColor(const std::shared_ptr<const DepthImage<DataType> >& depthImage, const PinholeCamera& depthCamera, const Transform& depthCameraPose, const std::shared_ptr<const ColorImage<ColorType> >& colorImage, const PinholeCamera& colorCamera, const Transform& colorCameraPose, Chunk* chunk) const
            {
                    assert(chunk != nullptr);

                    //! Step1：获取Chunk中包含的voxels个数、分辨率以及Chunk的初始坐标
                    float resolution = chunk->GetVoxelResolutionMeters();
                    Vec3 origin = chunk->GetOrigin();
                    float resolutionDiagonal = 2.0 * sqrt(3.0f) * resolution;
                    bool updated = false;
                    //std::vector<size_t> indexes;
                    //indexes.resize(centroids.size());
                    //for (size_t i = 0; i < centroids.size(); i++)
                    //{
                    //    indexes[i] = i;
                    //}

                    //! Step2: 根据论文算法1求取截断距离函数
                    for (size_t i = 0; i < centroids.size(); i++)
                    //parallel_for(indexes.begin(), indexes.end(), [&](const size_t& i)
                    {
                        //！Step2.1：将voxel从世界坐标系转到相机坐标系，再转到相机平面上，得到像素点cameraPos，并判断该像素点是否有效
                        Color<ColorType> color;
                        Vec3 voxelCenter = centroids[i] + origin;
                        Vec3 voxelCenterInCamera = depthCameraPose.linear().transpose() * (voxelCenter - depthCameraPose.translation());
                        Vec3 cameraPos = depthCamera.ProjectPoint(voxelCenterInCamera);

                        if (!depthCamera.IsPointOnImage(cameraPos) || voxelCenterInCamera.z() < 0)
                        {
                            continue;
                        }

                        //! Step2.2：求取该像素点的深度
                        float voxelDist = voxelCenterInCamera.z();
                        float depth = depthImage->DepthAt((int)cameraPos(1), (int)cameraPos(0)); //depthImage->BilinearInterpolateDepth(cameraPos(0), cameraPos(1));

                        if(std::isnan(depth))
                        {
                            continue;
                        }

                        //！Step2.3：求取该voxel到物体表面的距离(由截断距离的二次型计算)
                        //!           对应论文算法1中第5步
                        float truncation = truncator->GetTruncationDistance(depth);

                        //! Step2.4: 求取截断函数距离值，对应论文算法1中的u
                        float surfaceDist = depth - voxelDist;

                        //! Step2.5: 判断该voxel是否处于hit region
                        if (std::abs(surfaceDist) < truncation + resolutionDiagonal)
                        {
                            //! 将voxel投影到彩色相机平面上
                            Vec3 voxelCenterInColorCamera = colorCameraPose.linear().transpose() * (voxelCenter - colorCameraPose.translation());
                            Vec3 colorCameraPos = colorCamera.ProjectPoint(voxelCenterInColorCamera);

                            //! 如果该像素点有效
                            if(colorCamera.IsPointOnImage(colorCameraPos))
                            {
                                ColorVoxel& colorVoxel = chunk->GetColorVoxelMutable(i);

                                if (colorVoxel.GetWeight() < 5)
                                {
                                    int r = static_cast<int>(colorCameraPos(1));
                                    int c = static_cast<int>(colorCameraPos(0));
                                    //！获取(r，c)处像素点的颜色
                                    colorImage->At(r, c, &color);
                                    
                                    //! 按照论文III.C 更新voxe颜色
                                    colorVoxel.Integrate(color.red, color.green, color.blue, 1);
                                }
                            }

                            //! 按照算法1更新voxel的距离和权重
                            DistVoxel& voxel = chunk->GetDistVoxelMutable(i);
                            voxel.Integrate(surfaceDist, weighter->GetWeight(surfaceDist, truncation));

                            updated = true;
                        }

                        //! Step2.6: 判断该voxel是否处于space carving region
                        //! 对应论文算法1的第7步-->第10步
                        else if (enableVoxelCarving && surfaceDist > truncation + carvingDist)
                        {
                            DistVoxel& voxel = chunk->GetDistVoxelMutable(i);
                            if (voxel.GetWeight() > 0 && voxel.GetSDF() < 1e-5)
                            {
                                voxel.Carve();
                                updated = true;
                            }
                        }


                    }
                    //);

                    return updated;
            }

            inline const TruncatorPtr& GetTruncator() const { return truncator; }
            inline void SetTruncator(const TruncatorPtr& value) { truncator = value; }
            inline const WeighterPtr& GetWeighter() const { return weighter; }
            inline void SetWeighter(const WeighterPtr& value) { weighter = value; }

            inline float GetCarvingDist() const { return carvingDist; }
            inline bool IsCarvingEnabled() const { return enableVoxelCarving; }
            inline void SetCarvingDist(float dist) { carvingDist = dist; }
            inline void SetCarvingEnabled(bool enabled) { enableVoxelCarving = enabled; }

            inline void SetCentroids(const Vec3List& c) { centroids = c; }

        protected:
            TruncatorPtr truncator;
            WeighterPtr weighter;
            float carvingDist;
            bool enableVoxelCarving;
            Vec3List centroids;
    };

} // namespace chisel 

#endif // PROJECTIONINTEGRATOR_H_ 
