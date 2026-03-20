/*
Open Asset Import Library (assimp)
----------------------------------------------------------------------

Copyright (c) 2006-2025, assimp team

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

#ifdef ASSIMP_ENABLE_KTX2_DECODER

#include "KTX2Decoder.h"
#include <assimp/DefaultLogger.hpp>
#include <basisu_transcoder.h>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#define STBIW_WINDOWS_UTF8
#include "../../contrib/stb/stb_image_write.h"

#include <cstring>

namespace Assimp {

static const uint8_t KTX2_IDENTIFIER[12] = {
    0xAB, 0x4B, 0x54, 0x58, 0x20, 0x32, 0x30, 0xBB, 0x0D, 0x0A, 0x1A, 0x0A
};

struct KTX2Header {
    uint8_t identifier[12];
    uint32_t vkFormat;
    uint32_t typeSize;
    uint32_t pixelWidth;
    uint32_t pixelHeight;
    uint32_t pixelDepth;
    uint32_t layerCount;
    uint32_t faceCount;
    uint32_t levelCount;
    uint32_t supercompressionScheme;
    uint32_t dfdByteOffset;
    uint32_t dfdByteLength;
    uint32_t kvdByteOffset;
    uint32_t kvdByteLength;
    uint64_t sgdByteOffset;
    uint64_t sgdByteLength;
};

static void PNGWriteCallback(void* context, void* data, int size) {
    std::vector<uint8_t>* vec = static_cast<std::vector<uint8_t>*>(context);
    const uint8_t* bytes = static_cast<const uint8_t*>(data);
    vec->insert(vec->end(), bytes, bytes + size);
}

bool DecodeKTX2ToPNG(
    const uint8_t* ktx2Data,
    size_t length,
    std::vector<uint8_t>& pngData,
    uint32_t& width,
    uint32_t& height
) {
    if (length < sizeof(KTX2Header)) {
        ASSIMP_LOG_ERROR("KTX2: Invalid file size");
        return false;
    }

    KTX2Header header;
    std::memcpy(&header, ktx2Data, sizeof(KTX2Header));

    if (std::memcmp(header.identifier, KTX2_IDENTIFIER, 12) != 0) {
        ASSIMP_LOG_ERROR("KTX2: Invalid identifier");
        return false;
    }

    if (header.pixelWidth == 0 || header.pixelHeight == 0) {
        ASSIMP_LOG_ERROR("KTX2: Invalid dimensions");
        return false;
    }

    width = header.pixelWidth;
    height = header.pixelHeight;

    static bool transcoderInitialized = false;
    if (!transcoderInitialized) {
        basist::basisu_transcoder_init();
        transcoderInitialized = true;
    }

    basist::basisu_transcoder transcoder;

    if (!transcoder.validate_header(ktx2Data, static_cast<uint32_t>(length))) {
        ASSIMP_LOG_ERROR("KTX2: Invalid Basis Universal header");
        return false;
    }

    basist::basisu_image_info imageInfo;
    if (!transcoder.get_image_info(ktx2Data, static_cast<uint32_t>(length), imageInfo, 0)) {
        ASSIMP_LOG_ERROR("KTX2: Failed to get image info");
        return false;
    }

    if (!transcoder.start_transcoding(ktx2Data, static_cast<uint32_t>(length))) {
        ASSIMP_LOG_ERROR("KTX2: Failed to start transcoding");
        return false;
    }

    const uint32_t outputRowPitch = width * 4;
    const uint32_t outputSize = outputRowPitch * height;
    std::vector<uint8_t> rgbaData(outputSize);

    if (!transcoder.transcode_image_level(
            ktx2Data,
            static_cast<uint32_t>(length),
            0, 0, rgbaData.data(),
            width * height,
            basist::transcoder_texture_format::cTFRGBA32,
            0, outputRowPitch)) {
        ASSIMP_LOG_ERROR("KTX2: Failed to transcode image");
        return false;
    }

    pngData.clear();
    if (!stbi_write_png_to_func(PNGWriteCallback, &pngData,
                                 width, height, 4, rgbaData.data(), outputRowPitch)) {
        ASSIMP_LOG_ERROR("KTX2: Failed to encode PNG");
        return false;
    }

    return true;
}

} // namespace Assimp

#endif // ASSIMP_ENABLE_KTX2_DECODER
