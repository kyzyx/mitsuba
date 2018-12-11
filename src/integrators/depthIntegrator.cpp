#include <mitsuba/render/scene.h>

MTS_NAMESPACE_BEGIN

class MyIntegrator : public SamplingIntegrator {
public:
  MTS_DECLARE_CLASS()

  // Initialize the integrator with the specified properties
  MyIntegrator(const Properties &props) : SamplingIntegrator(props) {
    Spectrum defaultColor;
    defaultColor.fromLinearRGB(1.0f, 1.0f, 1.0f);
    m_color = props.getSpectrum("color", defaultColor);
  }

  // Unserialize from a binary data stream
  MyIntegrator(Stream *stream, InstanceManager *manager) : SamplingIntegrator(stream, manager) {
    m_color = Spectrum(stream);
    // _maxDist = stream->readFloat();
    m_maxDist = 100.0f;
  }

  // Serialize to a binary data stream
  void serialize(Stream *stream, InstanceManager *manager) const {
    SamplingIntegrator::serialize(stream, manager);
    m_color.serialize(stream);
    stream->writeFloat(m_maxDist);
  }

  // Query for an unbiased estimate of the radiance along <tt>r</tt>
  Spectrum Li(const RayDifferential &r, RadianceQueryRecord &rRec) const {
    if (rRec.rayIntersect(r)) {
      Float distance = rRec.its.t;
      return Spectrum(distance) * m_color;
    }
    return Spectrum(0.0f);
  }

  // Preprocess function -- called on the initiating machine
  bool preprocess(const Scene *scene, RenderQueue *queue, const RenderJob *job, int sceneResID, int cameraResID, int samplerResID) {
    SamplingIntegrator::preprocess(scene, queue, job, sceneResID, cameraResID, samplerResID);

    const AABB &sceneAABB = scene->getAABB();
    /* Find the camera position at t = 0 seconds */
    Point cameraPosition = scene->getSensor()->getWorldTransform()->eval(0).transformAffine(Point(0.0f));
    m_maxDist = - std::numeric_limits<Float>::infinity();

    for (int i=0; i<8; ++i)
      m_maxDist = std::max(m_maxDist, (cameraPosition - sceneAABB.getCorner(i)).length());

    m_maxDist = 100.0f;

    return true;
  }

private:
  Spectrum m_color;
  Float m_maxDist;
};

MTS_IMPLEMENT_CLASS_S(MyIntegrator, false, SamplingIntegrator)
MTS_EXPORT_PLUGIN(MyIntegrator, "A contived integrator");
MTS_NAMESPACE_END
