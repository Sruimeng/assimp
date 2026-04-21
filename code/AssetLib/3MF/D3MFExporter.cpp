/*
Open Asset Import Library (assimp)
----------------------------------------------------------------------

Copyright (c) 2006-2026, assimp team

All rights reserved.

Redistribution and use of this software in source and binary forms,
with or without modification, are permitted provided that the
following conditions are met:

* Redistributions of source code must retain the above
copyright notice, this list of conditions and the
following disclaimer.

* Redistributions in binary form must reproduce the above
copyright notice, this list of conditions and the
following disclaimer in the documentation and/or other
materials provided with the distribution.

* Neither the name of the assimp team, nor the names of its
contributors may be used to endorse or promote products
derived from this software without specific prior
written permission of the assimp team.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

----------------------------------------------------------------------
*/
#ifndef ASSIMP_BUILD_NO_EXPORT
#ifndef ASSIMP_BUILD_NO_3MF_EXPORTER

#include "D3MFExporter.h"

#include <assimp/Exceptional.h>
#include <assimp/StringUtils.h>
#include <assimp/scene.h>
#include <assimp/DefaultLogger.hpp>
#include <assimp/Exporter.hpp>
#include <assimp/IOSystem.hpp>

#include <cstdlib>
#include <cstring>
#include <unordered_map>

#include "3MFXmlTags.h"
#include "D3MFOpcPackage.h"

#ifdef ASSIMP_USE_HUNTER
#include <zip/zip.h>
#else
#include <contrib/zip/src/zip.h>
#endif

namespace Assimp {

void ExportScene3MF(const char *pFile, IOSystem *pIOSystem, const aiScene *pScene, const ExportProperties *pProperties) {
    if (nullptr == pIOSystem) {
        throw DeadlyExportError("Could not export 3MP archive: " + std::string(pFile));
    }
    const bool joinPositionVertices = (pProperties != nullptr) ?
            pProperties->GetPropertyBool("assimpjs.3mf.join_position_vertices", false) :
            false;
    D3MF::D3MFExporter myExporter(pFile, pScene, joinPositionVertices);
    if (myExporter.validate()) {
        if (pIOSystem->Exists(pFile)) {
            if (!pIOSystem->DeleteFile(pFile)) {
                throw DeadlyExportError("File exists, cannot override : " + std::string(pFile));
            }
        }
        bool ok = myExporter.exportArchive(pFile);
        if (!ok) {
            throw DeadlyExportError("Could not export 3MP archive: " + std::string(pFile));
        }
#if defined(__EMSCRIPTEN__)
        const auto &buffer = myExporter.GetArchiveBuffer();
        if (buffer.empty()) {
            throw DeadlyExportError("Could not export 3MP archive: " + std::string(pFile));
        }
        std::unique_ptr<IOStream> outfile (pIOSystem->Open(pFile, "wb"));
        if (outfile == nullptr) {
            throw DeadlyExportError("could not open output .3mf file: " + std::string(pFile));
        }
        outfile->Write(buffer.data(), 1, buffer.size());
#endif
    }
}

namespace D3MF {

struct Vec3Key {
    uint32_t x;
    uint32_t y;
    uint32_t z;

    bool operator==(const Vec3Key &other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct Vec3KeyHash {
    size_t operator()(const Vec3Key &k) const {
        size_t h = static_cast<size_t>(2166136261u);
        const uint32_t *words = reinterpret_cast<const uint32_t *>(&k);
        for (size_t i = 0; i < 3; ++i) {
            h ^= static_cast<size_t>(words[i]);
            h *= static_cast<size_t>(16777619u);
        }
        return h;
    }
};

struct SharedPositionMesh {
    std::vector<aiVector3D> points;
    std::vector<unsigned int> indices;
};

static uint32_t FloatBits(float value) {
    uint32_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    return bits;
}

static Vec3Key MakeVec3Key(const aiVector3D &value) {
    return {FloatBits(value.x), FloatBits(value.y), FloatBits(value.z)};
}

static bool BuildSharedPositionMesh(const aiMesh *mesh, SharedPositionMesh &out) {
    if (mesh == nullptr) {
        return false;
    }

    out.points.clear();
    out.indices.clear();

    std::unordered_map<Vec3Key, unsigned int, Vec3KeyHash> pointMap;
    pointMap.reserve(mesh->mNumVertices);
    out.indices.reserve(mesh->mNumFaces * 3);

    for (unsigned int faceIndex = 0; faceIndex < mesh->mNumFaces; ++faceIndex) {
        const aiFace &face = mesh->mFaces[faceIndex];
        if (face.mNumIndices < 3) {
            continue;
        }
        for (unsigned int indexIndex = 0; indexIndex < 3; ++indexIndex) {
            const unsigned int vertexIndex = face.mIndices[indexIndex];
            if (vertexIndex >= mesh->mNumVertices) {
                return false;
            }
            const aiVector3D &position = mesh->mVertices[vertexIndex];
            const Vec3Key key = MakeVec3Key(position);
            auto it = pointMap.find(key);
            if (it != pointMap.end()) {
                out.indices.push_back(it->second);
                continue;
            }
            const unsigned int pointIndex = static_cast<unsigned int>(out.points.size());
            out.points.push_back(position);
            pointMap.emplace(key, pointIndex);
            out.indices.push_back(pointIndex);
        }
    }

    return true;
}

static bool IsIdentityTransform(const aiMatrix4x4 &m) {
    const float eps = 1e-6f;
    return std::abs(m.a1 - 1.0f) < eps && std::abs(m.b2 - 1.0f) < eps && std::abs(m.c3 - 1.0f) < eps && std::abs(m.d4 - 1.0f) < eps &&
            std::abs(m.a2) < eps && std::abs(m.a3) < eps && std::abs(m.a4) < eps &&
            std::abs(m.b1) < eps && std::abs(m.b3) < eps && std::abs(m.b4) < eps &&
            std::abs(m.c1) < eps && std::abs(m.c2) < eps && std::abs(m.c4) < eps &&
            std::abs(m.d1) < eps && std::abs(m.d2) < eps && std::abs(m.d3) < eps;
}

D3MFExporter::D3MFExporter(const char *pFile, const aiScene *pScene, bool joinPositionVertices) :
        mArchiveName(pFile), m_zipArchive(nullptr), mScene(pScene), mJoinPositionVertices(joinPositionVertices) {
    // empty
}

D3MFExporter::~D3MFExporter() {
    for (size_t i = 0; i < mRelations.size(); ++i) {
        delete mRelations[i];
    }
    mRelations.clear();
}

const std::vector<uint8_t>& D3MFExporter::GetArchiveBuffer() const {
    return mArchiveBuffer;
}

bool D3MFExporter::validate() const {
    if (mArchiveName.empty()) {
        return false;
    }

    if (nullptr == mScene) {
        return false;
    }

    return true;
}

bool D3MFExporter::exportArchive(const char *file) {
    bool ok(true);
    mArchiveBuffer.clear();

#if defined(__EMSCRIPTEN__)
    m_zipArchive = zip_stream_open(nullptr, 0, ZIP_DEFAULT_COMPRESSION_LEVEL, 'w');
#else
    m_zipArchive = zip_open(file, ZIP_DEFAULT_COMPRESSION_LEVEL, 'w');
#endif
    if (nullptr == m_zipArchive) {
        return false;
    }

    ok &= exportContentTypes();
    ok &= export3DModel();
    ok &= exportRelations();

#if defined(__EMSCRIPTEN__)
    if (ok) {
        void *buf = nullptr;
        size_t bufsize = 0;
        if (zip_stream_copy(m_zipArchive, &buf, &bufsize) < 0 || buf == nullptr || bufsize == 0) {
            ok = false;
        } else {
            mArchiveBuffer.assign(static_cast<uint8_t*>(buf), static_cast<uint8_t*>(buf) + bufsize);
        }
        std::free(buf);
    }
    zip_stream_close(m_zipArchive);
#else
    zip_close(m_zipArchive);
#endif
    m_zipArchive = nullptr;

    return ok;
}

bool D3MFExporter::exportContentTypes() {
    mContentOutput.clear();

    mContentOutput << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>";
    mContentOutput << std::endl;
    mContentOutput << "<Types xmlns = \"http://schemas.openxmlformats.org/package/2006/content-types\">";
    mContentOutput << std::endl;
    mContentOutput << "<Default Extension = \"rels\" ContentType = \"application/vnd.openxmlformats-package.relationships+xml\" />";
    mContentOutput << std::endl;
    mContentOutput << "<Default Extension = \"model\" ContentType = \"application/vnd.ms-package.3dmanufacturing-3dmodel+xml\" />";
    mContentOutput << std::endl;
    mContentOutput << "</Types>";
    mContentOutput << std::endl;
    zipContentType(XmlTag::CONTENT_TYPES_ARCHIVE);

    return true;
}

bool D3MFExporter::exportRelations() {
    mRelOutput.clear();

    mRelOutput << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>";
    mRelOutput << std::endl;
    mRelOutput << "<Relationships xmlns=\"http://schemas.openxmlformats.org/package/2006/relationships\">";

    for (size_t i = 0; i < mRelations.size(); ++i) {
        if (mRelations[i]->target[0] == '/') {
            mRelOutput << "<Relationship Target=\"" << mRelations[i]->target << "\" ";
        } else {
            mRelOutput << "<Relationship Target=\"/" << mRelations[i]->target << "\" ";
        }
        mRelOutput << "Id=\"" << mRelations[i]->id << "\" ";
        mRelOutput << "Type=\"" << mRelations[i]->type << "\" />";
        mRelOutput << std::endl;
    }
    mRelOutput << "</Relationships>";
    mRelOutput << std::endl;

    zipRelInfo("_rels", ".rels");
    mRelOutput.flush();

    return true;
}

bool D3MFExporter::export3DModel() {
    mModelOutput.clear();

    writeHeader();
    mModelOutput << "<" << XmlTag::model << " " << XmlTag::model_unit << "=\"millimeter\""
                 << " xmlns=\"http://schemas.microsoft.com/3dmanufacturing/core/2015/02\">"
                 << std::endl;
    mModelOutput << "<" << XmlTag::resources << ">";
    mModelOutput << std::endl;

    writeMetaData();

    writeBaseMaterials();

    writeObjects();

    mModelOutput << "</" << XmlTag::resources << ">";
    mModelOutput << std::endl;
    writeBuild();

    mModelOutput << "</" << XmlTag::model << ">\n";

    OpcPackageRelationship *info = new OpcPackageRelationship;
    info->id = "rel0";
    info->target = "/3D/3DModel.model";
    info->type = XmlTag::PACKAGE_START_PART_RELATIONSHIP_TYPE;
    mRelations.push_back(info);

    zipModel("3D", "3DModel.model");
    mModelOutput.flush();

    return true;
}

void D3MFExporter::writeHeader() {
    mModelOutput << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>";
    mModelOutput << std::endl;
}

void D3MFExporter::writeMetaData() {
    if (nullptr == mScene->mMetaData) {
        return;
    }

    const unsigned int numMetaEntries(mScene->mMetaData->mNumProperties);
    if (0 == numMetaEntries) {
        return;
    }

    const aiString *key = nullptr;
    const aiMetadataEntry *entry(nullptr);
    for (size_t i = 0; i < numMetaEntries; ++i) {
        mScene->mMetaData->Get(i, key, entry);
        std::string k(key->C_Str());
        aiString value;
        mScene->mMetaData->Get(k, value);
        mModelOutput << "<" << XmlTag::meta << " " << XmlTag::meta_name << "=\"" << key->C_Str() << "\">";
        mModelOutput << value.C_Str();
        mModelOutput << "</" << XmlTag::meta << ">" << std::endl;
    }
}

void D3MFExporter::writeBaseMaterials() {
    mModelOutput << "<basematerials id=\"1\">\n";
    std::string strName, hexDiffuseColor, tmp;
    for (size_t i = 0; i < mScene->mNumMaterials; ++i) {
        aiMaterial *mat = mScene->mMaterials[i];
        aiString name;
        if (mat->Get(AI_MATKEY_NAME, name) != aiReturn_SUCCESS) {
            strName = "basemat_" + ai_to_string(i);
        } else {
            strName = name.C_Str();
        }
        aiColor4D color;
        if (mat->Get(AI_MATKEY_COLOR_DIFFUSE, color) == aiReturn_SUCCESS) {
            hexDiffuseColor.clear();
            tmp.clear();
            // rgbs %
            if (color.r <= 1 && color.g <= 1 && color.b <= 1 && color.a <= 1) {

                hexDiffuseColor = ai_rgba2hex(
                        (int)(((ai_real)color.r) * 255),
                        (int)(((ai_real)color.g) * 255),
                        (int)(((ai_real)color.b) * 255),
                        (int)(((ai_real)color.a) * 255),
                        true);

            } else {
                hexDiffuseColor = "#";
                tmp = ai_decimal_to_hexa((ai_real)color.r);
                hexDiffuseColor += tmp;
                tmp = ai_decimal_to_hexa((ai_real)color.g);
                hexDiffuseColor += tmp;
                tmp = ai_decimal_to_hexa((ai_real)color.b);
                hexDiffuseColor += tmp;
                tmp = ai_decimal_to_hexa((ai_real)color.a);
                hexDiffuseColor += tmp;
            }
        } else {
            hexDiffuseColor = "#FFFFFFFF";
        }

        mModelOutput << "<base name=\"" + strName + "\" " + " displaycolor=\"" + hexDiffuseColor + "\" />\n";
    }
    mModelOutput << "</basematerials>\n";
}

void D3MFExporter::writeObjects() {
    if (nullptr == mScene || mScene->mNumMeshes == 0) {
        return;
    }

    for (unsigned int i = 0; i < mScene->mNumMeshes; ++i) {
        aiMesh *currentMesh = mScene->mMeshes[i];
        if (nullptr == currentMesh) {
            continue;
        }

        unsigned int objectId = i + 2;
        mModelOutput << "<" << XmlTag::object << " id=\"" << objectId << "\" type=\"model\">";
        mModelOutput << std::endl;
        writeMesh(currentMesh);
        mModelOutput << "</" << XmlTag::object << ">";
        mModelOutput << std::endl;
    }

    mBuildItems.clear();
    if (mScene->mRootNode == nullptr) {
        return;
    }
    collectBuildItems(mScene->mRootNode, aiMatrix4x4());
    if (!mBuildItems.empty()) {
        return;
    }
    for (unsigned int i = 0; i < mScene->mNumMeshes; ++i) {
        if (mScene->mMeshes[i] == nullptr) {
            continue;
        }
        BuildItem item;
        item.objectId = i + 2;
        mBuildItems.push_back(item);
    }
}

void D3MFExporter::writeMesh(aiMesh *mesh) {
    if (nullptr == mesh) {
        return;
    }

    SharedPositionMesh sharedMesh;
    const bool useSharedPositions = mJoinPositionVertices && BuildSharedPositionMesh(mesh, sharedMesh);

    mModelOutput << "<"
                 << XmlTag::mesh
                 << ">" << "\n";
    mModelOutput << "<"
                 << XmlTag::vertices
                 << ">" << "\n";
    if (useSharedPositions) {
        for (size_t i = 0; i < sharedMesh.points.size(); ++i) {
            writeVertex(sharedMesh.points[i]);
        }
    } else {
        for (unsigned int i = 0; i < mesh->mNumVertices; ++i) {
            writeVertex(mesh->mVertices[i]);
        }
    }
    mModelOutput << "</"
                 << XmlTag::vertices << ">"
                 << "\n";

    const unsigned int matIdx(mesh->mMaterialIndex);
    if (useSharedPositions) {
        mModelOutput << "<"
                     << XmlTag::triangles << ">"
                     << "\n";
        const size_t triangleCount = sharedMesh.indices.size() / 3;
        for (size_t triangleIndex = 0; triangleIndex < triangleCount; ++triangleIndex) {
            const size_t baseIndex = triangleIndex * 3;
            mModelOutput << "<" << XmlTag::triangle << " v1=\"" << sharedMesh.indices[baseIndex] << "\" v2=\""
                         << sharedMesh.indices[baseIndex + 1] << "\" v3=\"" << sharedMesh.indices[baseIndex + 2]
                         << "\" pid=\"1\" p1=\"" + ai_to_string(matIdx) + "\" />";
            mModelOutput << "\n";
        }
        mModelOutput << "</"
                     << XmlTag::triangles
                     << ">";
        mModelOutput << "\n";
    } else {
        writeFaces(mesh, matIdx);
    }

    mModelOutput << "</"
                 << XmlTag::mesh << ">"
                 << "\n";
}

void D3MFExporter::writeVertex(const aiVector3D &pos) {
    mModelOutput << "<" << XmlTag::vertex << " x=\"" << pos.x << "\" y=\"" << pos.y << "\" z=\"" << pos.z << "\" />";
    mModelOutput << std::endl;
}

void D3MFExporter::writeFaces(aiMesh *mesh, unsigned int matIdx) {
    if (nullptr == mesh) {
        return;
    }

    if (!mesh->HasFaces()) {
        return;
    }
    mModelOutput << "<"
                 << XmlTag::triangles << ">"
                 << "\n";
    for (unsigned int i = 0; i < mesh->mNumFaces; ++i) {
        aiFace &currentFace = mesh->mFaces[i];
        mModelOutput << "<" << XmlTag::triangle << " v1=\"" << currentFace.mIndices[0] << "\" v2=\""
                     << currentFace.mIndices[1] << "\" v3=\"" << currentFace.mIndices[2]
                     << "\" pid=\"1\" p1=\"" + ai_to_string(matIdx) + "\" />";
        mModelOutput << "\n";
    }
    mModelOutput << "</"
                 << XmlTag::triangles
                 << ">";
    mModelOutput << "\n";
}

void D3MFExporter::writeBuild() {
    mModelOutput << "<"
                 << XmlTag::build
                 << ">"
                 << "\n";

    for (const BuildItem &item : mBuildItems) {
        mModelOutput << "<" << XmlTag::item << " objectid=\"" << item.objectId << "\"";
        if (item.hasTransform) {
            mModelOutput << " transform=\"";
            writeTransform(item.transform);
            mModelOutput << "\"";
        }
        mModelOutput << "/>";
        mModelOutput << "\n";
    }
    mModelOutput << "</" << XmlTag::build << ">";
    mModelOutput << "\n";
}

void D3MFExporter::collectBuildItems(const aiNode *node, const aiMatrix4x4 &parentTransform) {
    if (node == nullptr) {
        return;
    }

    aiMatrix4x4 currentTransform = parentTransform * node->mTransformation;
    for (unsigned int i = 0; i < node->mNumMeshes; ++i) {
        unsigned int meshIndex = node->mMeshes[i];
        if (meshIndex >= mScene->mNumMeshes || mScene->mMeshes[meshIndex] == nullptr) {
            continue;
        }
        BuildItem item;
        item.objectId = meshIndex + 2;
        item.transform = currentTransform;
        item.hasTransform = !IsIdentityTransform(currentTransform);
        mBuildItems.push_back(item);
    }

    for (unsigned int i = 0; i < node->mNumChildren; ++i) {
        collectBuildItems(node->mChildren[i], currentTransform);
    }
}

void D3MFExporter::writeTransform(const aiMatrix4x4 &transform) {
    std::ios::fmtflags oldFlags = mModelOutput.flags();
    std::streamsize oldPrecision = mModelOutput.precision();
    mModelOutput << std::setprecision(9);
    mModelOutput << transform.a1 << " " << transform.b1 << " " << transform.c1 << " "
                 << transform.a2 << " " << transform.b2 << " " << transform.c2 << " "
                 << transform.a3 << " " << transform.b3 << " " << transform.c3 << " "
                 << transform.a4 << " " << transform.b4 << " " << transform.c4;
    mModelOutput.flags(oldFlags);
    mModelOutput.precision(oldPrecision);
}

void D3MFExporter::zipContentType(const std::string &filename) {
    addFileInZip(filename, mContentOutput.str());
}

void D3MFExporter::zipModel(const std::string &folder, const std::string &modelName) {
    const std::string entry = folder + "/" + modelName;
    addFileInZip(entry, mModelOutput.str());
}

void D3MFExporter::zipRelInfo(const std::string &folder, const std::string &relName) {
    const std::string entry = folder + "/" + relName;
    addFileInZip(entry, mRelOutput.str());
}

void D3MFExporter::addFileInZip(const std::string& entry, const std::string& content) {
    if (nullptr == m_zipArchive) {
        throw DeadlyExportError("3MF-Export: Zip archive not valid, nullptr.");
    }

    zip_entry_open(m_zipArchive, entry.c_str());
    zip_entry_write(m_zipArchive, content.c_str(), content.size());
    zip_entry_close(m_zipArchive);
}

} // Namespace D3MF
} // Namespace Assimp

#endif // ASSIMP_BUILD_NO_3MF_EXPORTER
#endif // ASSIMP_BUILD_NO_EXPORT
