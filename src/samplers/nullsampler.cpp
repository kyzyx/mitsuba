/*
    This file is part of Mitsuba, a physically based rendering system.

    Copyright (c) 2007-2014 by Wenzel Jakob and others.

    Mitsuba is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License Version 3
    as published by the Free Software Foundation.

    Mitsuba is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include <mitsuba/render/sampler.h>

MTS_NAMESPACE_BEGIN

class NullSampler : public Sampler {
public:
    NullSampler() : Sampler(Properties()) {
        m_sampleCount = 1;
    }
    NullSampler(const Properties &props) : Sampler(props) {
        m_sampleCount = 1;
    }
    NullSampler(Stream *stream, InstanceManager *manager)
     : Sampler(stream, manager) {
        m_sampleCount = 1;
     }

    void generate(const Point2i &) {
        for (size_t i=0; i<m_req1D.size(); i++) {
            for (size_t j=0; j<m_req1D[i]; j++) {
                m_sampleArrays1D[i][j] = 0.5;
            }
        }
        for (size_t i=0; i<m_req2D.size(); i++) {
            for (size_t j=0; j<m_req2D[i]; j++) {
                m_sampleArrays2D[i][j] = next2D();
            }
        }
        m_sampleIndex = 0;
        m_dimension1DArray = m_dimension2DArray = 0;
    }
    ref<Sampler> clone() {
        ref<NullSampler> sampler = new NullSampler();
        sampler->m_sampleCount = m_sampleCount;

        for (size_t i=0; i<m_req1D.size(); ++i)
            sampler->request1DArray(m_req1D[i]);
        for (size_t i=0; i<m_req2D.size(); ++i)
            sampler->request2DArray(m_req2D[i]);
        return sampler.get();
    }
    void setSampleIndex(size_t sampleIndex) {
        m_sampleIndex = sampleIndex;
        m_dimension1DArray = m_dimension2DArray = 0;
    }

    void advance() {
        m_sampleIndex++;
        m_dimension1DArray = m_dimension2DArray = 0;
    }
    Float next1D() {
        return 0.5;
    }

    Point2 next2D() {
            return Point2(0.5,0.5);
    }

    std::string toString() const {
        return "NullSampler[]";
    }

    MTS_DECLARE_CLASS()
};

MTS_IMPLEMENT_CLASS_S(NullSampler, false, Sampler)
MTS_EXPORT_PLUGIN(NullSampler, "Null sampler");
MTS_NAMESPACE_END
