#ifndef OWDS_EMBEDDEDASSERTS_H
#define OWDS_EMBEDDEDASSERTS_H

#include <bgfx/embedded_shader.h>

// OpenGL ES shaders
#include <fs_default.sc.essl.bin.h>
#include <vs_default.sc.essl.bin.h>

// OpenGL shaders
#include <fs_default.sc.glsl.bin.h>
#include <vs_default.sc.glsl.bin.h>

// Vulkan shaders
#include <fs_default.sc.spv.bin.h>
#include <vs_default.sc.spv.bin.h>

#if defined(__WIN32__) || defined(_WIN32)

// HLSL shaders
#include <fs_default.sc.hlsl.bin.h>
#include <vs_default.sc.hlsl.bin.h>

#else

// todo: figure out how to tell bgfx to stop trying to use DirectX11/X12 on linux
#undef BGFX_EMBEDDED_SHADER_DXBC
#define BGFX_EMBEDDED_SHADER_DXBC(type, name) {},

#endif

static const ::bgfx::EmbeddedShader s_owds_embedded_shaders[] = {
  BGFX_EMBEDDED_SHADER(vs_default),
  BGFX_EMBEDDED_SHADER(fs_default),
  BGFX_EMBEDDED_SHADER_END()};

#endif // OWDS_EMBEDDEDASSERTS_H
