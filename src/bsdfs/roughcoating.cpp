/*
    This file is part of Mitsuba, a physically based rendering system.

    Copyright (c) 2007-2011 by Wenzel Jakob and others.

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

#include <mitsuba/render/bsdf.h>
#include <mitsuba/hw/basicshader.h>
#include "microfacet.h"
#include "ior.h"

MTS_NAMESPACE_BEGIN

#define TRANSMITTANCE_PRECOMP_NODES 200

/*!\plugin{roughcoating}{Rough coating material}
 * \order{10}
 * \icon{bsdf_roughcoating}
 * \parameters{
 *     \parameter{distribution}{\String}{
 *          Specifies the type of microfacet normal distribution 
 *          used to model the surface roughness.
 *       \begin{enumerate}[(i)]
 *           \item \code{beckmann}: Physically-based distribution derived from
 *               Gaussian random surfaces. This is the default.
 *           \item \code{ggx}: New distribution proposed by
 *              Walter et al. \cite{Walter07Microfacet}, which is meant to better handle 
 *              the long tails observed in measurements of ground surfaces. 
 *              Renderings with this distribution may converge slowly.
 *           \item \code{phong}: Classical $\cos^p\theta$ distribution.
 *              Due to the underlying microfacet theory, 
 *              the use of this distribution here leads to more realistic 
 *              behavior than the separately available \pluginref{phong} plugin.
 *              \vspace{-4mm}
 *       \end{enumerate}
 *     }
 *     \parameter{alpha}{\Float}{
 *         Specifies the roughness of the unresolved surface micro-geometry. 
 *         When the Beckmann distribution is used, this parameter is equal to the 
 *         \emph{root mean square} (RMS) slope of the microfacets. 
 *         \default{0.1}. 
 *     }
 *     \parameter{intIOR}{\Float\Or\String}{Interior index of refraction specified
 *      numerically or using a known material name. \default{\texttt{bk7} / 1.5046}}
 *     \parameter{extIOR}{\Float\Or\String}{Exterior index of refraction specified
 *      numerically or using a known material name. \default{\texttt{air} / 1.000277}}
 *     \parameter{sigmaA}{\Spectrum\Or\Texture}{The absorption coefficient of the 
 *      coating layer. \default{0, i.e. there is no absorption}}
 *     \parameter{\Unnamed}{\BSDF}{A nested BSDF model that should be coated.}
 * }
 *
 */
class RoughCoating : public BSDF {
public:
	/// \sa refractTo()
	enum EDestination {
		EInterior = 0,
		EExterior = 1
	};

	RoughCoating(const Properties &props) : BSDF(props) {
		/* Specifies the internal index of refraction at the interface */
		m_intIOR = lookupIOR(props, "intIOR", "bk7");

		/* Specifies the external index of refraction at the interface */
		m_extIOR = lookupIOR(props, "extIOR", "air");

		/* Specifies the absorption within the layer */
		m_sigmaA = new ConstantSpectrumTexture(
			props.getSpectrum("sigmaA", Spectrum(0.0f)));

		if (m_intIOR < 0 || m_extIOR < 0 || m_intIOR == m_extIOR)
			Log(EError, "The interior and exterior indices of "
				"refraction must be positive and differ!");

		m_distribution = MicrofacetDistribution(
			props.getString("distribution", "beckmann")
		);

		if (m_distribution.isAnisotropic())
			Log(EError, "The 'roughplastic' plugin currently does not support "
				"anisotropic microfacet distributions!");

		m_alpha = m_distribution.transformRoughness(
			props.getFloat("alpha", 0.1f));

		m_specularSamplingWeight = 0.0f;
	}

	RoughCoating(Stream *stream, InstanceManager *manager) 
	 : BSDF(stream, manager) {
		m_distribution = MicrofacetDistribution(
			(MicrofacetDistribution::EType) stream->readUInt()
		);
		m_nested = static_cast<BSDF *>(manager->getInstance(stream));
		m_sigmaA = static_cast<Texture *>(manager->getInstance(stream));
		m_roughTransmittance = static_cast<CubicSpline *>(manager->getInstance(stream));
		m_alpha = stream->readFloat();
		m_intIOR = stream->readFloat();
		m_extIOR = stream->readFloat();
		m_thickness = stream->readFloat();

		configure();
	}

	void configure() {
		unsigned int extraFlags = 0;
		if (!m_sigmaA->isConstant())
			extraFlags |= ESpatiallyVarying;

		m_components.clear();
		for (int i=0; i<m_nested->getComponentCount(); ++i) 
			m_components.push_back(m_nested->getType(i) | extraFlags);

		m_components.push_back(EGlossyReflection | EFrontSide | EBackSide);

		m_usesRayDifferentials = m_nested->usesRayDifferentials()
			|| m_sigmaA->usesRayDifferentials();

		/* Compute weights that further steer samples towards
		   the specular or nested components */
		Float avgAbsorption = (m_sigmaA->getAverage()
			 *(-2*m_thickness)).exp().average();

		m_specularSamplingWeight = 1.0f / (avgAbsorption + 1.0f);

		/* Precompute the rough transmittance through the interface */
		m_roughTransmittance = m_distribution.computeRoughTransmittance(
				m_extIOR, m_intIOR, m_alpha, TRANSMITTANCE_PRECOMP_NODES);

		BSDF::configure();
	}

	/// Helper function: reflect \c wi with respect to a given surface normal
	inline Vector reflect(const Vector &wi, const Normal &m) const {
		return 2 * dot(wi, m) * Vector(m) - wi;
	}

	inline Float signum(Float value) const {
		return (value < 0) ? -1.0f : 1.0f;
	}

	/// Refraction in local coordinates 
	Vector refractTo(EDestination dest,
			const Vector &wi) const {
		Float etaI, etaT;
		if (dest == EInterior) {
			etaI = m_extIOR;
			etaT = m_intIOR;
		} else {
			etaI = m_intIOR;
			etaT = m_extIOR;
		}

		Float cosThetaI = Frame::cosTheta(wi);
		bool entering = cosThetaI > 0.0f;

		/* Using Snell's law, calculate the squared sine of the
		   angle between the normal and the transmitted ray */
		Float eta = etaI / etaT,
			  sinThetaTSqr = eta*eta * Frame::sinTheta2(wi);

		if (sinThetaTSqr >= 1.0f) {
			/* Total internal reflection */
			return Vector(0.0f);
		} else {
			Float cosThetaT = std::sqrt(1.0f - sinThetaTSqr);

			/* Retain the directionality of the vector */
			return Vector(eta*wi.x, eta*wi.y, 
				entering ? cosThetaT : -cosThetaT);
		}
	}

	Spectrum eval(const BSDFQueryRecord &bRec, EMeasure measure) const {
		bool hasNested = (bRec.typeMask & m_nested->getType() & BSDF::EAll)
			&& (bRec.component == -1 || bRec.component < (int) m_components.size()-1);
		bool hasSpecular = (bRec.typeMask & EGlossyReflection)
			&& (bRec.component == -1 || bRec.component == (int) m_components.size()-1)
			&& measure == ESolidAngle;
			
		Spectrum result(0.0f);
		if (hasSpecular && Frame::cosTheta(bRec.wo) * Frame::cosTheta(bRec.wi) > 0) {
			/* Calculate the reflection half-vector */
			const Vector H = normalize(bRec.wo+bRec.wi)
				* signum(Frame::cosTheta(bRec.wo));

			/* Evaluate the microsurface normal distribution */
			const Float D = m_distribution.eval(H, m_alpha);

			/* Fresnel term */
			const Float F = fresnel(absDot(bRec.wi, H), m_extIOR, m_intIOR);

			/* Smith's shadow-masking function */
			const Float G = m_distribution.G(bRec.wi, bRec.wo, H, m_alpha);

			/* Calculate the specular reflection component */
			Float value = F * D * G / 
				(4.0f * std::abs(Frame::cosTheta(bRec.wi)));

			result += Spectrum(value);
		}

		if (hasNested) {
			BSDFQueryRecord bRecInt(bRec);
			bRecInt.wi = refractTo(EInterior, bRec.wi);
			bRecInt.wo = refractTo(EInterior, bRec.wo);

			Spectrum nestedResult = m_nested->eval(bRecInt, measure) *
				m_roughTransmittance->eval(std::abs(Frame::cosTheta(bRec.wi))) *
				m_roughTransmittance->eval(std::abs(Frame::cosTheta(bRec.wo)));

			Spectrum sigmaA = m_sigmaA->getValue(bRec.its) * m_thickness;
			if (!sigmaA.isZero()) 
				nestedResult *= (-sigmaA *
					(1/std::abs(Frame::cosTheta(bRecInt.wi)) +
					 1/std::abs(Frame::cosTheta(bRecInt.wo)))).exp();

			if (measure == ESolidAngle) {
				Float eta = m_extIOR / m_intIOR;
				/* Solid angle compression & irradiance conversion factors */
				nestedResult *= eta * eta * 
					  Frame::cosTheta(bRec.wi) * Frame::cosTheta(bRec.wo)
				   / (Frame::cosTheta(bRecInt.wi) * Frame::cosTheta(bRecInt.wo));
			}

			result += nestedResult;
		}

		return result;
	}

	Float pdf(const BSDFQueryRecord &bRec, EMeasure measure) const {
		bool hasNested = (bRec.typeMask & m_nested->getType() & BSDF::EAll)
			&& (bRec.component == -1 || bRec.component < (int) m_components.size()-1);
		bool hasSpecular = (bRec.typeMask & EGlossyReflection)
			&& (bRec.component == -1 || bRec.component == (int) m_components.size()-1)
			&& measure == ESolidAngle;

		/* Calculate the reflection half-vector */
		const Vector H = normalize(bRec.wo+bRec.wi)
				* signum(Frame::cosTheta(bRec.wo));

		Float probNested, probSpecular;
		if (hasSpecular && hasNested) {
			/* Find the probability of sampling the specular component */
			probSpecular = 1-m_roughTransmittance->eval(
				std::abs(Frame::cosTheta(bRec.wi)));

			/* Reallocate samples */
			probSpecular = (probSpecular*m_specularSamplingWeight) /
				(probSpecular*m_specularSamplingWeight + 
				(1-probSpecular) * (1-m_specularSamplingWeight));

			probNested = 1 - probSpecular;
		} else {
			probNested = probSpecular = 1.0f;
		}

		Float result = 0.0f;
		if (hasSpecular && Frame::cosTheta(bRec.wo) * Frame::cosTheta(bRec.wi) > 0) {
			/* Jacobian of the half-direction transform */
			const Float dwh_dwo = 1.0f / (4.0f * absDot(bRec.wo, H));

			/* Evaluate the microsurface normal distribution */
			const Float prob = m_distribution.pdf(H, m_alpha);

			result = prob * dwh_dwo * probSpecular;
		}

		if (hasNested) {
			BSDFQueryRecord bRecInt(bRec);
			bRecInt.wi = refractTo(EInterior, bRec.wi);
			bRecInt.wo = refractTo(EInterior, bRec.wo);

			Float prob = m_nested->pdf(bRecInt, measure);

			if (measure == ESolidAngle) {
				Float eta = m_extIOR / m_intIOR;
				prob *= eta * eta * Frame::cosTheta(bRec.wo)
			          / Frame::cosTheta(bRecInt.wo);
			}

			result += prob * probNested;
		}

		return result;
	}

	inline Spectrum sample(BSDFQueryRecord &bRec, Float &_pdf, const Point2 &_sample) const {
		bool hasNested = (bRec.typeMask & m_nested->getType() & BSDF::EAll)
			&& (bRec.component == -1 || bRec.component < (int) m_components.size()-1);
		bool hasSpecular = (bRec.typeMask & EGlossyReflection)
			&& (bRec.component == -1 || bRec.component == (int) m_components.size()-1);

		bool choseSpecular = hasSpecular;
		Point2 sample(_sample);

		Float probSpecular;
		if (hasSpecular && hasNested) {
			/* Find the probability of sampling the diffuse component */
			probSpecular = 1 - m_roughTransmittance->eval(std::abs(Frame::cosTheta(bRec.wi)));

			/* Reallocate samples */
			probSpecular = (probSpecular*m_specularSamplingWeight) /
				(probSpecular*m_specularSamplingWeight + 
				(1-probSpecular) * (1-m_specularSamplingWeight));

			if (sample.x <= probSpecular) {
				sample.x /= probSpecular;
			} else {
				sample.x = (sample.x - probSpecular) / (1 - probSpecular);
				choseSpecular = false;
			}
		}

		if (choseSpecular) {
			/* Perfect specular reflection based on the microsurface normal */
			Normal m = m_distribution.sample(sample, m_alpha);
			bRec.wo = reflect(bRec.wi, m);
			bRec.sampledComponent = m_components.size()-1;
			bRec.sampledType = EGlossyReflection;

			/* Side check */
			if (Frame::cosTheta(bRec.wo) * Frame::cosTheta(bRec.wi) <= 0)
				return Spectrum(0.0f);
		} else {
			Vector wiBackup = bRec.wi;
			bRec.wi = refractTo(EInterior, bRec.wi);
			Spectrum result = m_nested->sample(bRec, _pdf, sample);
			bRec.wi = wiBackup;
			if (result.isZero()) 
				return Spectrum(0.0f);
			bRec.wo = refractTo(EExterior, bRec.wo);
			if (bRec.wo.isZero())
				return Spectrum(0.0f);
		}

		/* Guard against numerical imprecisions */
		EMeasure measure = getMeasure(bRec.sampledType);
		_pdf = pdf(bRec, measure);

		if (_pdf == 0) 
			return Spectrum(0.0f);
		else
			return eval(bRec, measure) / _pdf;
	}

	Spectrum sample(BSDFQueryRecord &bRec, const Point2 &sample) const {
		Float pdf;
		return RoughCoating::sample(bRec, pdf, sample);
	}

	void serialize(Stream *stream, InstanceManager *manager) const {
		BSDF::serialize(stream, manager);

		stream->writeUInt((uint32_t) m_distribution.getType());
		manager->serialize(stream, m_nested.get());
		manager->serialize(stream, m_sigmaA.get());
		manager->serialize(stream, m_roughTransmittance.get());
		stream->writeFloat(m_alpha);
		stream->writeFloat(m_intIOR);
		stream->writeFloat(m_extIOR);
		stream->writeFloat(m_thickness);
	}

	void addChild(const std::string &name, ConfigurableObject *child) {
		if (child->getClass()->derivesFrom(MTS_CLASS(BSDF))) {
			if (m_nested != NULL)
				Log(EError, "Only a single nested BRDF can be added!");
			m_nested = static_cast<BSDF *>(child);
		} else if (child->getClass()->derivesFrom(MTS_CLASS(Texture)) && name == "sigmaA") {
			m_sigmaA = static_cast<Texture *>(m_sigmaA);
		} else {
			BSDF::addChild(name, child);
		}
	}

	std::string toString() const {
		std::ostringstream oss;
		oss << "RoughCoating[" << endl
			<< "  name = \"" << getName() << "\"," << endl
			<< "  distribution = " << m_distribution.toString() << "," << endl
			<< "  alpha = " << m_alpha << "," << endl
			<< "  sigmaA = " << m_sigmaA->toString() << "," << endl
			<< "  specularSamplingWeight = " << m_specularSamplingWeight << "," << endl
			<< "  diffuseSamplingWeight = " << (1-m_specularSamplingWeight) << "," << endl
			<< "  intIOR = " << m_intIOR << "," << endl
			<< "  extIOR = " << m_extIOR << "," << endl
			<< "  nested = " << indent(m_nested.toString()) << endl
			<< "]";
		return oss.str();
	}

	Shader *createShader(Renderer *renderer) const;

	MTS_DECLARE_CLASS()
private:
	MicrofacetDistribution m_distribution;
	ref<CubicSpline> m_roughTransmittance;
	ref<Texture> m_sigmaA;
	ref<BSDF> m_nested;
	Float m_alpha, m_intIOR, m_extIOR;
	Float m_specularSamplingWeight;
	Float m_thickness;
};

/**
 * GLSL port of the rough coating shader. This version is much more
 * approximate -- it only supports the Beckmann distribution, 
 * does everything in RGB, uses a cheaper shadowing-masking term, and 
 * it also makes use of the Schlick approximation to the Fresnel 
 * reflectance of dielectrics. When the roughness is lower than 
 * \alpha < 0.2, the shader clamps it to 0.2 so that it will still perform
 * reasonably well in a VPL-based preview.
 */
class RoughCoatingShader : public Shader {
public:
	RoughCoatingShader(Renderer *renderer, 
			const BSDF *nested, 
			const Texture *sigmaA, 
			Float alpha, Float extIOR, 
			Float intIOR) : Shader(renderer, EBSDFShader), 
			m_nested(nested), 
			m_sigmaA(sigmaA), 
			m_alpha(alpha), m_extIOR(extIOR), m_intIOR(intIOR) {
		m_nestedShader = renderer->registerShaderForResource(m_nested.get());
		m_sigmaAShader = renderer->registerShaderForResource(m_sigmaA.get());
		m_alpha = std::max(m_alpha, (Float) 0.2f);
		m_R0 = fresnel(1.0f, m_extIOR, m_intIOR);
		m_eta = extIOR / intIOR;
	}

	bool isComplete() const {
		return m_nestedShader.get() != NULL
			&& m_sigmaAShader.get() != NULL;
	}

	void putDependencies(std::vector<Shader *> &deps) {
		deps.push_back(m_nestedShader.get());
		deps.push_back(m_sigmaAShader.get());
	}

	void cleanup(Renderer *renderer) {
		renderer->unregisterShaderForResource(m_nested.get());
		renderer->unregisterShaderForResource(m_sigmaA.get());
	}

	void resolve(const GPUProgram *program, const std::string &evalName, std::vector<int> &parameterIDs) const {
		parameterIDs.push_back(program->getParameterID(evalName + "_R0", false));
		parameterIDs.push_back(program->getParameterID(evalName + "_eta", false));
		parameterIDs.push_back(program->getParameterID(evalName + "_alpha", false));
	}

	void bind(GPUProgram *program, const std::vector<int> &parameterIDs, int &textureUnitOffset) const {
		program->setParameter(parameterIDs[0], m_R0);
		program->setParameter(parameterIDs[1], m_eta);
		program->setParameter(parameterIDs[2], m_alpha);
	}

	void generateCode(std::ostringstream &oss,
			const std::string &evalName,
			const std::vector<std::string> &depNames) const {
		oss << "uniform float " << evalName << "_R0;" << endl
			<< "uniform float " << evalName << "_eta;" << endl
			<< "uniform float " << evalName << "_alpha;" << endl
			<< endl
			<< "float " << evalName << "_schlick(float ct) {" << endl
			<< "    float ctSqr = ct*ct, ct5 = ctSqr*ctSqr*ct;" << endl
			<< "    return " << evalName << "_R0 + (1.0 - " << evalName << "_R0) * ct5;" << endl
			<< "}" << endl
			<< endl
			<< "vec3 " << evalName << "_refract(vec3 wi, out float T) {" << endl
			<< "    float cosThetaI = cosTheta(wi);" << endl
			<< "    bool entering = cosThetaI > 0.0;" << endl
			<< "    float eta = " << evalName << "_eta;" << endl
			<< "    float sinThetaTSqr =  eta * eta * sinTheta2(wi);" << endl
			<< "    if (sinThetaTSqr >= 1.0) {" << endl
			<< "        T = 0.0; /* Total internal reflection */" << endl
			<< "        return vec3(0.0);" << endl
			<< "    } else {" << endl
			<< "        float cosThetaT = sqrt(1.0 - sinThetaTSqr);" << endl
			<< "        T = 1.0 - " << evalName << "_schlick(1.0 - abs(cosThetaI));" << endl
			<< "        return vec3(eta*wi.x, eta*wi.y, entering ? cosThetaT : -cosThetaT);" << endl
			<< "    }" << endl
			<< "}" << endl
			<< endl
			<< "float " << evalName << "_D(vec3 m) {" << endl
			<< "    float ct = cosTheta(m);" << endl
			<< "    if (cosTheta(m) <= 0.0)" << endl
			<< "        return 0.0;" << endl
			<< "    float ex = tanTheta(m) / " << evalName << "_alpha;" << endl
			<< "    return exp(-(ex*ex)) / (pi * " << evalName << "_alpha" << endl
			<< "        * " << evalName << "_alpha * pow(cosTheta(m), 4.0));" << endl
			<< "}" << endl
			<< endl
			<< "float " << evalName << "_G(vec3 m, vec3 wi, vec3 wo) {" << endl
			<< "    if ((dot(wi, m) * cosTheta(wi)) <= 0 || " << endl
			<< "        (dot(wo, m) * cosTheta(wo)) <= 0)" << endl
			<< "        return 0.0;" << endl
			<< "    float nDotM = cosTheta(m);" << endl
			<< "    return min(1.0, min(" << endl
			<< "        abs(2 * nDotM * cosTheta(wo) / dot(wo, m))," << endl
			<< "        abs(2 * nDotM * cosTheta(wi) / dot(wi, m))));" << endl
			<< "}" << endl
			<< endl
			<< "vec3 " << evalName << "(vec2 uv, vec3 wi, vec3 wo) {" << endl
			<< "    float T12, T21;" << endl
			<< "    vec3 wiPrime = " << evalName << "_refract(wi, T12);" << endl
			<< "    vec3 woPrime = " << evalName << "_refract(wo, T21);" << endl
			<< "    vec3 nested = " << depNames[0] << "(uv, wiPrime, woPrime);" << endl
			<< "    vec3 sigmaA = " << depNames[1] << "(uv);" << endl
			<< "    vec3 result = nested * " << evalName << "_eta * " << evalName << "_eta" << endl
			<< "                  * T12 * T21 * (cosTheta(wi)*cosTheta(wo)) /" << endl
			<< "                  (cosTheta(wiPrime)*cosTheta(woPrime));" << endl
			<< "    if (sigmaA != vec3(0.0))" << endl
			<< "        result *= exp(-sigmaA * (1/abs(cosTheta(wiPrime)) + " << endl
			<< "                                 1/abs(cosTheta(woPrime))));" << endl
			<< "    if (cosTheta(wi)*cosTheta(wo) > 0) {" << endl
			<< "        vec3 H = normalize(wi + wo);" << endl
			<< "        float D = " << evalName << "_D(H)" << ";" << endl
			<< "        float G = " << evalName << "_G(H, wi, wo);" << endl
			<< "        float F = " << evalName << "_schlick(1-dot(wi, H));" << endl
			<< "        result += vec3(F * D * G / (4*cosTheta(wi)));" << endl
			<< "    }" << endl
			<< "    return result;" << endl
			<< "}" << endl
			<< endl
			<< "vec3 " << evalName << "_diffuse(vec2 uv, vec3 wi, vec3 wo) {" << endl
			<< "    return " << depNames[0] << "_diffuse(uv, wi, wo);" << endl
			<< "}" << endl;
	}

	MTS_DECLARE_CLASS()
private:
	ref<const BSDF> m_nested;
	ref<Shader> m_nestedShader;
	ref<const Texture> m_sigmaA;
	ref<Shader> m_sigmaAShader;
	Float m_alpha, m_extIOR, m_intIOR, m_R0, m_eta;
};

Shader *RoughCoating::createShader(Renderer *renderer) const { 
	return new RoughCoatingShader(renderer, m_nested.get(), 
		m_sigmaA.get(), m_alpha, m_extIOR, m_intIOR);
}

MTS_IMPLEMENT_CLASS(RoughCoatingShader, false, Shader)
MTS_IMPLEMENT_CLASS_S(RoughCoating, false, BSDF)
MTS_EXPORT_PLUGIN(RoughCoating, "Rough coating BSDF");
MTS_NAMESPACE_END
