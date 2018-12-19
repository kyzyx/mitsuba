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

#include <mitsuba/render/trimesh.h>
#include <mitsuba/core/plugin.h>
#include <mitsuba/core/fresolver.h>
#include <mitsuba/core/timer.h>
#include <mitsuba/render/emitter.h>
#include <mitsuba/render/bsdf.h>
#include <mitsuba/render/subsurface.h>
#include <mitsuba/render/medium.h>
#include <mitsuba/render/sensor.h>
#include <mitsuba/hw/basicshader.h>
#include <set>

MTS_NAMESPACE_BEGIN


class ShapeNetOBJ : public Shape {
public:
	struct ShapeNetTriangle {
		int p[3];
		int n[3];
		int uv[3];

		std::string mtl[2];

		ShapeNetTriangle() {
			p[0] = p[1] = p[2] = 0;
			n[0] = n[1] = n[2] = 0;
			uv[0] = uv[1] = uv[2] = 0;
			mtl[0] = "";
			mtl[1] = "";
		}

		// determine whether the same face
		bool operator==(const ShapeNetTriangle& tri)
		{
			return (p[0] == tri.p[0] || p[0] == tri.p[1] || p[0] == tri.p[2]) &&
					(p[1] == tri.p[0] || p[1] == tri.p[1] || p[1] == tri.p[2]) &&
					(p[2] == tri.p[0] || p[2] == tri.p[1] || p[2] == tri.p[2]);
		}

		void flip()
		{
			std::swap(mtl[1], mtl[0]);
			std::swap(p[1], p[0]);
			std::swap(n[1], n[0]);
			std::swap(uv[1], uv[0]);
		}
	};

	std::vector<ShapeNetTriangle> m_triPool;

	bool isGoodUV(int uv[3])
	{
		return uv[0] != uv[1] && uv[1] != uv[2] && uv[0] != uv[2] &&
			uv[0] && uv[1] && uv[2];
	}

	bool checkAndAddTriangle(std::vector<ShapeNetTriangle>& triangles, ShapeNetTriangle& t)
	{
		bool find = false;

		for (ShapeNetTriangle& tri : triangles)
		{
			if (tri == t)
			{
				// double face exists
				find = true;

				if (isGoodUV(t.uv) && !isGoodUV(tri.uv))
				{
					// sometimes double-sided face contains bad tex coords
					std::string temp = tri.mtl[0];
					tri = t;
					tri.mtl[1] = temp;
				}
				else
				{
					tri.mtl[1] = t.mtl[0];
				}

				// well, flip face based on material name sorting
				if (tri.mtl[1].compare(tri.mtl[0]) < 0)
					tri.flip();

				break;
			}
		}

		if (!find)
		{
			triangles.push_back(t);
		}
		return !find;
	}

	// group triangles by double-sided material
	typedef std::map<std::string, std::map<std::string, std::vector<ShapeNetTriangle> > > TriGroup;

	TriGroup groupTriByMtl(std::vector<ShapeNetTriangle> triangles)
	{
		TriGroup group;

		for (ShapeNetTriangle tri : triangles)
		{
			if (group.find(tri.mtl[0]) == group.end())
			{
				group[tri.mtl[0]] = std::map<std::string, std::vector<ShapeNetTriangle> >();
			}

			if (group[tri.mtl[0]].find(tri.mtl[1]) == group[tri.mtl[0]].end())
			{
				group[tri.mtl[0]][tri.mtl[1]] = std::vector<ShapeNetTriangle>();
			}
			group[tri.mtl[0]][tri.mtl[1]].push_back(tri);
		}

		return group;
	}

	bool fetch_line(std::istream &is, std::string &line) {
		/// Fetch a line from the stream, while handling line breaks with backslashes
		if (!std::getline(is, line))
			return false;
		if (line == "")
			return true;
		int lastCharacter = (int)line.size() - 1;
		while (lastCharacter >= 0 &&
			(line[lastCharacter] == '\r' ||
			line[lastCharacter] == '\n' ||
			line[lastCharacter] == '\t' ||
			line[lastCharacter] == ' '))
			lastCharacter--;

		if (lastCharacter >= 0 && line[lastCharacter] == '\\') {
			std::string nextLine;
			fetch_line(is, nextLine);
			line = line.substr(0, lastCharacter) + nextLine;
		}
		else {
			line.resize(lastCharacter + 1);
		}
		return true;
	}

	ShapeNetOBJ(const Properties &props) : Shape(props) {
		ref<FileResolver> fileResolver = Thread::getThread()->getFileResolver()->clone();
		fs::path path = fileResolver->resolve(props.getString("filename"));

		m_name = path.stem().string();

		/* Object-space -> World-space transformation */
		Transform objectToWorld = props.getTransform("toWorld", Transform());
		Float maxSmoothAngle = props.getFloat("maxSmoothAngle", -1.0);

		/* Load the geometry */
		Log(EInfo, "Loading geometry from \"%s\" ..", path.filename().string().c_str());
		fs::ifstream is(path);
		if (is.bad() || is.fail())
			Log(EError, "ShapeNet OBJ file '%s' not found!", path.string().c_str());

		fileResolver->prependPath(fs::absolute(path).parent_path());

		ref<Timer> timer = new Timer();
		std::string buf, line;
		std::vector<Point> vertices;
		std::vector<Normal> normals;
		std::vector<Point2> texcoords;
		std::vector<ShapeNetTriangle> triangles;
		std::vector<Vertex> vertexBuffer;

		std::string materialName;

		while (is.good() && !is.eof() && fetch_line(is, line)) {
			std::istringstream iss(line);
			if (!(iss >> buf))
				continue;

			if (buf == "v") {
				/* Parse + transform vertices */
				Point p;
				iss >> p.x >> p.y >> p.z;
				vertices.push_back(p);
			}
			else if (buf == "vn") {
				Normal n;
				iss >> n.x >> n.y >> n.z;
				normals.push_back(n);
			}
			else if (buf == "mtllib") {

				fs::path materialLibrary = fileResolver->resolve(trim(line.substr(6, line.length() - 1)));

				// we load material library from .mtl file first
				if (!materialLibrary.empty())
					loadMaterialLibrary(fileResolver, materialLibrary);
			}
			else if (buf == "vt") {
				Float u, v;
				iss >> u >> v;
				// fix texture orientation
				v = -v;
				texcoords.push_back(Point2(u, v));
			}
			else if (buf == "f") {
				std::string  tmp;
				ShapeNetTriangle t;

				t.mtl[0] = materialName;
				t.mtl[1] = materialName;

				iss >> tmp; parse(t, 0, tmp);
				iss >> tmp; parse(t, 1, tmp);
				iss >> tmp; parse(t, 2, tmp);

				// check double face here
				//triangles.push_back(t);
				checkAndAddTriangle(triangles, t);
				/* Handle n-gons assuming a convex shape */
				while (iss >> tmp) {
					t.p[1] = t.p[2];
					t.uv[1] = t.uv[2];
					t.n[1] = t.n[2];
					parse(t, 2, tmp);

					// check double face here
					//triangles.push_back(t);
					checkAndAddTriangle(triangles, t);
				}
			}
			if (buf == "usemtl")
			{
				materialName = trim(line.substr(6, line.length() - 1));
			}
			else if (buf == "o") {
				std::string objName = trim(line.substr(1, line.length() - 1));
			}
			else if (buf == "g") {
				std::string groupName = trim(line.substr(1, line.length() - 1));
			}
			else if (buf == "s") {
				std::string smooth = trim(line.substr(1, line.length() - 1));
				if (smooth == "1")
				{
				}
				else if (smooth == "off")
				{
				}
			}

			else {
				/* Ignore */
			}
		}

		if (!triangles.empty())
		{
			createMesh0("model",
				vertices, normals, texcoords,
				triangles, objectToWorld, vertexBuffer, maxSmoothAngle < 0);

			triangles.clear();
		}

		// well, we use some smooth here
		if (maxSmoothAngle > 0)
		{
			for (size_t i = 0; i<m_meshes.size(); ++i)
				m_meshes[i]->rebuildTopology(maxSmoothAngle);
		}


		Log(EInfo, "Done with \"%s\" (took %i ms)", path.filename().string().c_str(), timer->getMilliseconds());
	}


	ShapeNetOBJ(Stream *stream, InstanceManager *manager) : Shape(stream, manager) {
		m_aabb = AABB(stream);
		uint32_t meshCount = stream->readUInt();
		m_meshes.resize(meshCount);

		for (uint32_t i = 0; i<meshCount; ++i) {
			m_meshes[i] = static_cast<TriMesh *>(manager->getInstance(stream));
			m_meshes[i]->incRef();
		}
	}

	void serialize(Stream *stream, InstanceManager *manager) const {
		Shape::serialize(stream, manager);

		m_aabb.serialize(stream);
		stream->writeUInt((uint32_t)m_meshes.size());
		for (size_t i = 0; i<m_meshes.size(); ++i)
			manager->serialize(stream, m_meshes[i]);
	}

	void parse(ShapeNetTriangle &t, int i, const std::string &str) {
		std::vector<std::string> tokens = tokenize(str, "/");
		if (tokens.size() == 1) {
			t.p[i] = atoi(tokens[0].c_str());
		}
		else if (tokens.size() == 2) {
			if (str.find("//") == std::string::npos) {
				t.p[i] = atoi(tokens[0].c_str());
				t.uv[i] = atoi(tokens[1].c_str());
			}
			else {
				t.p[i] = atoi(tokens[0].c_str());
				t.n[i] = atoi(tokens[1].c_str());
			}
		}
		else if (tokens.size() == 3) {
			t.p[i] = atoi(tokens[0].c_str());
			t.uv[i] = atoi(tokens[1].c_str());
			t.n[i] = atoi(tokens[2].c_str());
		}
		else {
			Log(EError, "Invalid OBJ face format!");
		}
	}

	Texture *loadTexture(const FileResolver *fileResolver,
		std::map<std::string, Texture *> &cache,
		const fs::path &mtlPath, std::string filename,
		bool noGamma = false) {
		/* Prevent Linux/OSX fs::path handling issues for DAE files created on Windows */
		for (size_t i = 0; i<filename.length(); ++i) {
			if (filename[i] == '\\')
				filename[i] = '/';
		}

		if (cache.find(filename) != cache.end())
			return cache[filename];

		fs::path path = fileResolver->resolve(filename);
		if (!fs::exists(path)) {
			path = fileResolver->resolve(fs::path(filename).filename());
			if (!fs::exists(path)) {
				Log(EWarn, "Unable to find texture \"%s\" referenced from \"%s\"!",
					path.string().c_str(), mtlPath.string().c_str());
				return new ConstantSpectrumTexture(Spectrum(0.0f));
			}
		}
		Properties props("bitmap");
		props.setString("filename", path.string());
		if (noGamma)
			props.setFloat("gamma", 1.0f);
		ref<Texture> texture = static_cast<Texture *> (PluginManager::getInstance()->
			createObject(MTS_CLASS(Texture), props));
		texture->configure();
		texture->incRef();
		cache[filename] = texture;
		return texture;
	}

	void loadMaterialLibrary(const FileResolver *fileResolver, const fs::path &mtlPath) {
		if (!fs::exists(mtlPath)) {
			Log(EWarn, "Could not find referenced material library '%s'",
				mtlPath.string().c_str());
			return;
		}

		Log(EInfo, "Loading OBJ materials from \"%s\" ..", mtlPath.filename().string().c_str());
		fs::ifstream is(mtlPath);
		if (is.bad() || is.fail())
			Log(EError, "Unexpected I/O error while accessing material file '%s'!",
			mtlPath.string().c_str());
		std::string buf, line;
		std::string mtlName;
		ref<Texture> specular, diffuse, exponent, bump, mask;
		int illum = 0;
		specular = new ConstantSpectrumTexture(Spectrum(0.0f));
		diffuse = new ConstantSpectrumTexture(Spectrum(0.0f));
		exponent = new ConstantFloatTexture(0.0f);
		std::map<std::string, Texture *> cache;

		while (is.good() && !is.eof() && fetch_line(is, line)) {
			std::istringstream iss(line);
			if (!(iss >> buf))
				continue;

			if (buf == "newmtl") {
				if (mtlName != "")
					addMaterial(mtlName, diffuse, specular, exponent, bump, mask, illum);

				mtlName = trim(line.substr(6, line.length() - 6));

				specular = new ConstantSpectrumTexture(Spectrum(0.0f));
				diffuse = new ConstantSpectrumTexture(Spectrum(0.0f));
				exponent = new ConstantFloatTexture(0.0f);
				mask = NULL;
				bump = NULL;
				illum = 0;
			}
			else if (buf == "Kd") {
				Float r, g, b;
				iss >> r >> g >> b;
				Spectrum value;
				value.fromSRGB(r, g, b);
				diffuse = new ConstantSpectrumTexture(value);
			}
			else if (buf == "map_Kd") {
				std::string filename;
				iss >> filename;
				diffuse = loadTexture(fileResolver, cache, mtlPath, filename);
			}
			else if (buf == "Ks") {
				Float r, g, b;
				iss >> r >> g >> b;
				Spectrum value;
				value.fromSRGB(r, g, b);
				specular = new ConstantSpectrumTexture(value);
			}
			else if (buf == "map_Ks") {
				std::string filename;
				iss >> filename;
				specular = loadTexture(fileResolver, cache, mtlPath, filename);
			}
			else if (buf == "bump") {
				std::string filename;
				iss >> filename;
				bump = loadTexture(fileResolver, cache, mtlPath, filename, true);
			}
			else if (buf == "map_d") {
				std::string filename;
				iss >> filename;
				mask = loadTexture(fileResolver, cache, mtlPath, filename);
			}
			else if (buf == "d" /* || buf == "Tr" */) {
				Float value;
				iss >> value;
				if (value == 1)
					mask = NULL;
				else
					mask = new ConstantFloatTexture(value);
			}
			else if (buf == "Ns") {
				Float value;
				iss >> value;
				exponent = new ConstantFloatTexture(value);
			}
			else if (buf == "illum") {
				iss >> illum;
			}
			else {
				/* Ignore */
			}
		}

		addMaterial(mtlName, diffuse, specular, exponent, bump, mask, illum);

		for (std::map<std::string, Texture *>::iterator it = cache.begin();
			it != cache.end(); ++it)
			it->second->decRef();
	}

	void addMaterial(const std::string &name, Texture *diffuse, Texture *specular,
		Texture *exponent, Texture *bump, Texture *mask, int model) {
		ref<BSDF> bsdf;
		Properties props;

		if (model == 2 && (specular->getMaximum().isZero() || exponent->getMaximum().isZero()))
			model = 1;

		if (model == 2) {
			props.setPluginName("phong");

			bsdf = static_cast<BSDF *> (PluginManager::getInstance()->
				createObject(MTS_CLASS(BSDF), props));
			bsdf->addChild("diffuseReflectance", diffuse);
			bsdf->addChild("specularReflectance", specular);
			bsdf->addChild("exponent", exponent);
		}
		else if (model == 4 || model == 6 || model == 7 || model == 9) {
			props.setPluginName("dielectric");
			bsdf = static_cast<BSDF *> (PluginManager::getInstance()->
				createObject(MTS_CLASS(BSDF), props));
		}
		else if (model == 5 || model == 8) {
			props.setPluginName("conductor");
			props.setString("material", "Al");
			bsdf = static_cast<BSDF *> (PluginManager::getInstance()->
				createObject(MTS_CLASS(BSDF), props));
		}
		else {
			props.setPluginName("diffuse");
			bsdf = static_cast<BSDF *> (PluginManager::getInstance()->
				createObject(MTS_CLASS(BSDF), props));
			bsdf->addChild("reflectance", diffuse);
		}

		bsdf->configure();

		if (bump) {
			props = Properties("bumpmap");
			ref<BSDF> bumpBSDF = static_cast<BSDF *> (PluginManager::getInstance()->
				createObject(MTS_CLASS(BSDF), props));

			bumpBSDF->addChild(bump);
			bumpBSDF->addChild(bsdf);
			bumpBSDF->configure();
			bsdf = bumpBSDF;
		}

		if (mask) {
			props = Properties("mask");
			ref<BSDF> maskedBSDF = static_cast<BSDF *> (PluginManager::getInstance()->
				createObject(MTS_CLASS(BSDF), props));

			maskedBSDF->addChild("opacity", mask);
			maskedBSDF->addChild(bsdf);
			maskedBSDF->configure();
			bsdf = maskedBSDF;
		}

		bsdf->setID(name);
		addChild(name, bsdf, false);
		// save the BSDF reference
		m_mtl[name] = bsdf;
	}

	struct Vertex {
		Point p;
		Normal n;
		Point2 uv;
	};

	/// For using vertices as keys in an associative structure
	struct vertex_key_order : public
		std::binary_function<Vertex, Vertex, bool> {
	public:
		bool operator()(const Vertex &v1, const Vertex &v2) const {
			if (v1.p.x < v2.p.x) return true;
			else if (v1.p.x > v2.p.x) return false;
			if (v1.p.y < v2.p.y) return true;
			else if (v1.p.y > v2.p.y) return false;
			if (v1.p.z < v2.p.z) return true;
			else if (v1.p.z > v2.p.z) return false;
			if (v1.n.x < v2.n.x) return true;
			else if (v1.n.x > v2.n.x) return false;
			if (v1.n.y < v2.n.y) return true;
			else if (v1.n.y > v2.n.y) return false;
			if (v1.n.z < v2.n.z) return true;
			else if (v1.n.z > v2.n.z) return false;
			if (v1.uv.x < v2.uv.x) return true;
			else if (v1.uv.x > v2.uv.x) return false;
			if (v1.uv.y < v2.uv.y) return true;
			else if (v1.uv.y > v2.uv.y) return false;
			return false;
		}
	};

	void createMesh(const std::string &name,
		const std::vector<Point> &vertices,
		const std::vector<Normal> &normals,
		const std::vector<Point2> &texcoords,
		const std::vector<ShapeNetTriangle> &triangles,
		const Transform &objectToWorld,
		std::vector<Vertex> &vertexBuffer,
		const std::string &mtlName,
		ref<BSDF>	bsdf,
		bool faceNormal)
	{
		if (triangles.size() == 0)
			return;
		typedef std::map<Vertex, uint32_t, vertex_key_order> VertexMapType;
		VertexMapType vertexMap;

		vertexBuffer.reserve(vertices.size());
		size_t numMerged = 0;
		AABB aabb;
		bool hasTexcoords = false;
		bool hasNormals = false;
		ref<Timer> timer = new Timer();
		vertexBuffer.clear();

		/* Collapse the mesh into a more usable form */
		Triangle *triangleArray = new Triangle[triangles.size()];
		for (uint32_t i = 0; i<triangles.size(); i++) {
			Triangle tri;
			for (uint32_t j = 0; j<3; j++) {
				int vertexId = triangles[i].p[j];
				int normalId = triangles[i].n[j];
				int uvId = triangles[i].uv[j];
				uint32_t key;

				Vertex vertex;
				if (vertexId < 0)
					vertexId += (int)vertices.size() + 1;
				if (normalId < 0)
					normalId += (int)normals.size() + 1;
				if (uvId < 0)
					uvId += (int)texcoords.size() + 1;

				if (vertexId >(int) vertices.size() || vertexId <= 0)
					Log(EError, "Out of bounds: tried to access vertex %i (max: %i)", vertexId, (int)vertices.size());

				vertex.p = objectToWorld(vertices[vertexId - 1]);
				aabb.expandBy(vertex.p);

				if (normalId != 0) {
					if (normalId > (int)normals.size() || normalId < 0)
						Log(EError, "Out of bounds: tried to access normal %i (max: %i)", normalId, (int)normals.size());
					vertex.n = objectToWorld(normals[normalId - 1]);
					if (!vertex.n.isZero())
						vertex.n = normalize(vertex.n);
					hasNormals = true;
				}
				else {
					vertex.n = Normal(0.0f);
				}

				if (uvId != 0) {
					if (uvId > (int)texcoords.size() || uvId < 0)
						Log(EError, "Out of bounds: tried to access uv %i (max: %i)", uvId, (int)texcoords.size());
					vertex.uv = texcoords[uvId - 1];
					hasTexcoords = true;
				}
				else {
					vertex.uv = Point2(0.0f);
				}

				VertexMapType::iterator it = vertexMap.find(vertex);
				if (it != vertexMap.end()) {
					key = it->second;
					numMerged++;
				}
				else {
					key = (uint32_t)vertexBuffer.size();
					vertexMap[vertex] = key;
					vertexBuffer.push_back(vertex);
				}

				tri.idx[j] = key;
			}
			triangleArray[i] = tri;
		}

		ref<TriMesh> mesh = new TriMesh(name,
			triangles.size(), vertexBuffer.size(),
			hasNormals, hasTexcoords, false, false, faceNormal);
			//m_flipNormals, m_faceNormals);

		std::copy(triangleArray, triangleArray + triangles.size(), mesh->getTriangles());

		Point    *target_positions = mesh->getVertexPositions();
		Normal   *target_normals = mesh->getVertexNormals();
		Point2   *target_texcoords = mesh->getVertexTexcoords();

		mesh->getAABB() = aabb;

		for (size_t i = 0; i<vertexBuffer.size(); i++) {
			*target_positions++ = vertexBuffer[i].p;
			if (hasNormals)
				*target_normals++ = vertexBuffer[i].n;
			if (hasTexcoords)
				*target_texcoords++ = vertexBuffer[i].uv;
		}

		mesh->incRef();
		m_meshes.push_back(mesh);
		Log(EInfo, "%s: " SIZE_T_FMT " triangles, " SIZE_T_FMT
			" vertices (merged " SIZE_T_FMT " vertices).", name.c_str(),
			triangles.size(), vertexBuffer.size(), numMerged);

		// apply the bsdf to mesh
		mesh->addChild(mtlName, bsdf);

	}

	void createMesh0(const std::string& targetName,
		const std::vector<Point> &vertices,
		const std::vector<Normal> &normals,
		const std::vector<Point2> &texcoords,
		const std::vector<ShapeNetTriangle> &triangles,
		const Transform &objectToWorld,
		std::vector<Vertex> &vertexBuffer,
		bool faceNormal)
	{
		TriGroup group = groupTriByMtl(triangles);

		int counter = 1;

		for (auto it1 = group.begin(); it1 != group.end(); ++it1)
		{
			ref<BSDF> bsdf1 = m_mtl[it1->first];

			for (auto it2 = it1->second.begin(); it2 != it1->second.end(); ++it2)
			{
				ref<BSDF> bsdf2 = m_mtl[it2->first];

				std::string name = formatString("%s-%s", it1->first.c_str(), it2->first.c_str());

				ref<BSDF> bsdf;

				if (bsdf1->hasComponent(BSDF::ETransmission))
				{
					bsdf = bsdf1;
				}
				else if (bsdf2->hasComponent(BSDF::ETransmission))
				{
					bsdf = bsdf2;
				}
				else if (m_mtl.find(name) != m_mtl.end())
				{
					bsdf = m_mtl[name];
				}
				else
				{
					// create two-sided bsdf
					Properties props;
					props.setPluginName("twosided");

					bsdf = static_cast<BSDF *> (PluginManager::getInstance()->
						createObject(MTS_CLASS(BSDF), props));
					bsdf->addChild("side-1", bsdf1);
					bsdf->addChild("side-2", bsdf2);
					bsdf->configure();

					m_mtl[name] = bsdf;
				}

				createMesh(formatString("%s-%i", targetName.c_str(), counter),
					vertices, normals, texcoords,
					it2->second, objectToWorld, vertexBuffer,
					name, bsdf, faceNormal);

				counter++;
			}
		}
	}

	virtual ~ShapeNetOBJ() {
		for (size_t i = 0; i<m_meshes.size(); ++i)
			m_meshes[i]->decRef();
	}

	void configure() {
		Shape::configure();

		m_aabb.reset();
		for (size_t i = 0; i<m_meshes.size(); ++i) {
			m_meshes[i]->configure();
			m_aabb.expandBy(m_meshes[i]->getAABB());
		}
	}

	void addChild(const std::string &name, ConfigurableObject *child) {
		addChild(name, child, true);
	}

	void addChild(const std::string &name, ConfigurableObject *child, bool warn) {
		const Class *cClass = child->getClass();
		if (cClass->derivesFrom(MTS_CLASS(BSDF))) {
			Shape::addChild(name, child);
			//Log(EInfo, "register the material named %s", name);
			if (name == "") {
				for (size_t i = 0; i<m_meshes.size(); ++i)
					m_meshes[i]->addChild(name, child);
			}
			/*
			Assert(m_meshes.size() > 0);
			if (name == "") {
				for (size_t i = 0; i<m_meshes.size(); ++i)
					m_meshes[i]->addChild(name, child);
			}
			else {
				bool found = false;
				for (size_t i = 0; i<m_meshes.size(); ++i) {
					if (m_materialAssignment[i] == name) {
						found = true;
						m_meshes[i]->addChild(name, child);
					}
				}
				if (!found && warn)
					Log(EWarn, "Attempted to register the material named "
					"'%s', which does not occur in the OBJ file!", name.c_str());
			}*/
			m_bsdf->setParent(NULL);
		}
		else if (cClass->derivesFrom(MTS_CLASS(Emitter))) {
			if (m_meshes.size() > 1)
				Log(EError, "Cannot attach an emitter to an OBJ file "
				"containing multiple objects!");
			m_emitter = static_cast<Emitter *>(child);
			child->setParent(m_meshes[0]);
			m_meshes[0]->addChild(name, child);
		}
		else if (cClass->derivesFrom(MTS_CLASS(Sensor))) {
			if (m_meshes.size() > 1)
				Log(EError, "Cannot attach an sensor to an OBJ file "
				"containing multiple objects!");
			m_sensor = static_cast<Sensor *>(child);
			child->setParent(m_meshes[0]);
			m_meshes[0]->addChild(name, child);
		}
		else if (cClass->derivesFrom(MTS_CLASS(Subsurface))) {
			Assert(m_subsurface == NULL);
			m_subsurface = static_cast<Subsurface *>(child);
			for (size_t i = 0; i<m_meshes.size(); ++i) {
				child->setParent(m_meshes[i]);
				m_meshes[i]->addChild(name, child);
			}
		}
		else if (cClass->derivesFrom(MTS_CLASS(Medium))) {
			Shape::addChild(name, child);
			for (size_t i = 0; i<m_meshes.size(); ++i)
				m_meshes[i]->addChild(name, child);
		}
		else {
			Shape::addChild(name, child);
		}
	}

	bool isCompound() const {
		return true;
	}

	Shape *getElement(int index) {
		if (index >= (int)m_meshes.size())
			return NULL;
		Shape *shape = m_meshes[index];
		BSDF *bsdf = shape->getBSDF();
		Emitter *emitter = shape->getEmitter();
		Subsurface *subsurface = shape->getSubsurface();
		if (bsdf)
			bsdf->setParent(shape);
		if (emitter)
			emitter->setParent(shape);
		if (subsurface)
			subsurface->setParent(shape);
		return shape;
	}

	AABB getAABB() const {
		return m_aabb;
	}

	Float getSurfaceArea() const {
		Float sa = 0;
		for (size_t i = 0; i<m_meshes.size(); ++i)
			sa += m_meshes[i]->getSurfaceArea();
		return sa;
	}

	size_t getPrimitiveCount() const {
		size_t result = 0;
		for (size_t i = 0; i<m_meshes.size(); ++i)
			result += m_meshes[i]->getPrimitiveCount();
		return result;
	}

	size_t getEffectivePrimitiveCount() const {
		size_t result = 0;
		for (size_t i = 0; i<m_meshes.size(); ++i)
			result += m_meshes[i]->getEffectivePrimitiveCount();
		return result;
	}

	MTS_DECLARE_CLASS()
private:
	std::vector<TriMesh *> m_meshes;
	AABB m_aabb;

	// store material from .mtl file
	std::map<std::string, ref<BSDF> > m_mtl;
};

MTS_IMPLEMENT_CLASS_S(ShapeNetOBJ, false, Shape)
MTS_EXPORT_PLUGIN(ShapeNetOBJ, "ShapeNet mesh loader");
MTS_NAMESPACE_END
