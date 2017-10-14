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

#ifndef DISTVOXEL_H_
#define DISTVOXEL_H_

#include <limits>
#include <stdint.h>

#include <open_chisel/FixedPointFloat.h>

namespace chisel
{

    class DistVoxel
    {
        public:
            DistVoxel();
            virtual ~DistVoxel();

            inline float GetSDF() const
            {
                return sdf;
            }

            inline void SetSDF(const float& distance)
            {
                sdf = distance;
            }

            inline float GetWeight() const { return weight; }
            inline void SetWeight(const float& w) { weight = w; }

            //! 更新voxel的符号距离函数值和权重，对应论文算法1中的第12和13步
            inline void Integrate(const float& distUpdate, const float& weightUpdate)
            {
                float oldSDF = GetSDF();
                float oldWeight = GetWeight();
                //! 更新SDF，对应算法12步
                float newDist = (oldWeight * oldSDF + weightUpdate * distUpdate) / (weightUpdate + oldWeight);
                SetSDF(newDist);
                //! 更新权重，对应算法13步
                SetWeight(oldWeight + weightUpdate+Observe_Num);
                Observe_Num ++;
                if(Observe_Num>=30)
                    Observe_Num = 30;

                if(Observe_Num==1)
                    Static_sdf = sdf;

                if(weight>0 && sdf>Static_sdf)
                    Static_sdf = sdf;
            }

            //! 重置voxel
            inline void Carve()
            {
                //Reset();
                Integrate(0.0, 1.5);
            }

            inline void Reset()
            {
                sdf = 99999;
                weight = 0;
            }
            /*
            inline void Set_Dyanmic()
            {
                Dynmiac_state = true;
            }

            inline void Reset_Dynamic()
            {
                Dynmiac_state = false;
            }
            */
            inline bool Set_State(bool state)
            {
                Dynmiac_state = state;
            }

            inline bool Get_State()
            {
                return Dynmiac_state;
            }

            inline float Get_ObserveNum()
            {
                return weight;
            }

            inline float Get_StaticSdf()
            {
                return Static_sdf;
            }

            inline void Set_Intesity(const uint8_t& newRed, const uint8_t& newGreen, const uint8_t& newBlue)
            {
                double Intensity_diff;
                Intensity_current = static_cast<double>((newRed*30 + newGreen*59 + newBlue*11)/100);
                if(Intensity_last < 0)
                    Intensity_last = Intensity_current;

                if(fabs(Intensity_current - Intensity_last)>10)
                    Dynmiac_state = true;
                Intensity_last = Intensity_current;
            }
        protected:
            float sdf;
            float weight;
            bool Dynmiac_state;
            float Observe_Num;
            float Static_sdf;
            double Intensity_last;
            double Intensity_current;
    };

} // namespace chisel 

#endif // DISTVOXEL_H_ 
