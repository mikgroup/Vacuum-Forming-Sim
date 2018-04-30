#include "texture.h"
#include "CGL/color.h"

namespace CGL {

Color Texture::sample(const SampleParams &sp) {
  // Part 5: Fill this in.

  // Determine the mipmap level.
  float level = 0;
  if (sp.lsm != L_ZERO) {
    level = get_level(sp);
  }

  if (sp.lsm == L_LINEAR) {
    return sample_trilinear(sp.p_uv, level);
  } else if (sp.psm == P_NEAREST) {
    return sample_nearest(sp.p_uv, round(level));
  } else if (sp.psm == P_LINEAR) {
    return sample_bilinear(sp.p_uv, round(level));
  }
}

float Texture::get_level(const SampleParams &sp) {
  // Part 6: Fill this in.
  Vector2D diffX = (sp.p_dx_uv - sp.p_uv) * width;
  Vector2D diffY = (sp.p_dy_uv - sp.p_uv) * height;
  float valX = diffX[0]*diffX[0] + diffX[1]*diffX[1];
  float valY = diffY[0]*diffY[0] + diffY[1]*diffY[1];
  float val = max(valX, valY);
  return max(0.0f, min((float)mipmap.size()-1, (float)(log2(val) / 2)));
}

Color Texture::sample_nearest(Vector2D uv, int level) {
  // Part 5: Fill this in.

  // Initialize variables
  MipLevel& m = mipmap[level];
  int u = round(uv[0] * m.width), v = round(uv[1] * m.height);
  u = max(0, min(u, (int)m.width-1)); v = max(0, min(v, (int)m.height-1));
  int index = 4 * (v*m.width + u);

  // Fill in color
  unsigned char r = m.texels[index];
  unsigned char g = m.texels[index + 1];
  unsigned char b = m.texels[index + 2];
  unsigned char a = m.texels[index + 3];
  unsigned char c[4] = {r, g, b, a};
  return Color(c);
}

Color Texture::sample_bilinear(Vector2D uv, int level) {
  // Part 5: Fill this in.
  
  // Initialize uv values
  MipLevel& m = mipmap[level];
  vector<unsigned char>& t = m.texels;
  int lowU = floor(uv[0] * m.width), lowV = floor(uv[1] * m.height);
  int highU = ceil(uv[0] * m.width), highV = ceil(uv[1] * m.height);
  lowU = max(0, min(lowU, (int)m.width-1)); lowV = max(0, min(lowV, (int)m.height-1));
  highU = max(0, min(highU, (int)m.width-1)); highV = max(0, min(highV, (int)m.height-1));

  // Create corner colors
  int i = 4 * (lowV * m.width + lowU);
  unsigned char a00[4] = {t[i], t[i+1], t[i+2], t[i+3]};
  Color c00 = Color(a00);
  i = 4 * (lowV * m.width + highU);
  unsigned char a10[4] = {t[i], t[i+1], t[i+2], t[i+3]};
  Color c10 = Color(a10);
  i = 4 * (highV * m.width + lowU);
  unsigned char a01[4] = {t[i], t[i+1], t[i+2], t[i+3]};
  Color c01 = Color(a01);
  i = 4 * (highV * m.width + highU);
  unsigned char a11[4] = {t[i], t[i+1], t[i+2], t[i+3]};
  Color c11 = Color(a11);

  // Compute horizontal ratio and get top/bottom colors
  float hRatio = (uv[0] * m.width) - lowU;
  Color bottom = c00 * (1 - hRatio) + c10 * hRatio;
  Color top = c01 * (1 - hRatio) + c11 * hRatio;

  // Compute vertical ratio and get weighted color
  float vRatio = (uv[1] * m.height) - lowV;
  return bottom * (1 - vRatio) + top * vRatio;
}

Color Texture::sample_trilinear(Vector2D uv, float level) {
  // Part 6: Fill this in.
  Color lowColor = sample_bilinear(uv, floor(level));
  Color highColor = sample_bilinear(uv, ceil(level));
  float ratio = level - floor(level);
  //cout << level << "\n";
  return lowColor * (1 - ratio) + highColor * ratio;
}




/****************************************************************************/



inline void uint8_to_float(float dst[4], unsigned char *src) {
  uint8_t *src_uint8 = (uint8_t *)src;
  dst[0] = src_uint8[0] / 255.f;
  dst[1] = src_uint8[1] / 255.f;
  dst[2] = src_uint8[2] / 255.f;
  dst[3] = src_uint8[3] / 255.f;
}

inline void float_to_uint8(unsigned char *dst, float src[4]) {
  uint8_t *dst_uint8 = (uint8_t *)dst;
  dst_uint8[0] = (uint8_t)(255.f * max(0.0f, min(1.0f, src[0])));
  dst_uint8[1] = (uint8_t)(255.f * max(0.0f, min(1.0f, src[1])));
  dst_uint8[2] = (uint8_t)(255.f * max(0.0f, min(1.0f, src[2])));
  dst_uint8[3] = (uint8_t)(255.f * max(0.0f, min(1.0f, src[3])));
}

void Texture::generate_mips(int startLevel) {

  // make sure there's a valid texture
  if (startLevel >= mipmap.size()) {
    std::cerr << "Invalid start level";
  }

  // allocate sublevels
  int baseWidth = mipmap[startLevel].width;
  int baseHeight = mipmap[startLevel].height;
  int numSubLevels = (int)(log2f((float)max(baseWidth, baseHeight)));

  numSubLevels = min(numSubLevels, kMaxMipLevels - startLevel - 1);
  mipmap.resize(startLevel + numSubLevels + 1);

  int width = baseWidth;
  int height = baseHeight;
  for (int i = 1; i <= numSubLevels; i++) {

    MipLevel &level = mipmap[startLevel + i];

    // handle odd size texture by rounding down
    width = max(1, width / 2);
    //assert (width > 0);
    height = max(1, height / 2);
    //assert (height > 0);

    level.width = width;
    level.height = height;
    level.texels = vector<unsigned char>(4 * width * height);
  }

  // create mips
  int subLevels = numSubLevels - (startLevel + 1);
  for (int mipLevel = startLevel + 1; mipLevel < startLevel + subLevels + 1;
       mipLevel++) {

    MipLevel &prevLevel = mipmap[mipLevel - 1];
    MipLevel &currLevel = mipmap[mipLevel];

    int prevLevelPitch = prevLevel.width * 4; // 32 bit RGBA
    int currLevelPitch = currLevel.width * 4; // 32 bit RGBA

    unsigned char *prevLevelMem;
    unsigned char *currLevelMem;

    currLevelMem = (unsigned char *)&currLevel.texels[0];
    prevLevelMem = (unsigned char *)&prevLevel.texels[0];

    float wDecimal, wNorm, wWeight[3];
    int wSupport;
    float hDecimal, hNorm, hWeight[3];
    int hSupport;

    float result[4];
    float input[4];

    // conditional differentiates no rounding case from round down case
    if (prevLevel.width & 1) {
      wSupport = 3;
      wDecimal = 1.0f / (float)currLevel.width;
    } else {
      wSupport = 2;
      wDecimal = 0.0f;
    }

    // conditional differentiates no rounding case from round down case
    if (prevLevel.height & 1) {
      hSupport = 3;
      hDecimal = 1.0f / (float)currLevel.height;
    } else {
      hSupport = 2;
      hDecimal = 0.0f;
    }

    wNorm = 1.0f / (2.0f + wDecimal);
    hNorm = 1.0f / (2.0f + hDecimal);

    // case 1: reduction only in horizontal size (vertical size is 1)
    if (currLevel.height == prevLevel.height) {
      //assert (currLevel.height == 1);

      for (int i = 0; i < currLevel.width; i++) {
        wWeight[0] = wNorm * (1.0f - wDecimal * i);
        wWeight[1] = wNorm * 1.0f;
        wWeight[2] = wNorm * wDecimal * (i + 1);

        result[0] = result[1] = result[2] = result[3] = 0.0f;

        for (int ii = 0; ii < wSupport; ii++) {
          uint8_to_float(input, prevLevelMem + 4 * (2 * i + ii));
          result[0] += wWeight[ii] * input[0];
          result[1] += wWeight[ii] * input[1];
          result[2] += wWeight[ii] * input[2];
          result[3] += wWeight[ii] * input[3];
        }

        // convert back to format of the texture
        float_to_uint8(currLevelMem + (4 * i), result);
      }

      // case 2: reduction only in vertical size (horizontal size is 1)
    } else if (currLevel.width == prevLevel.width) {
      //assert (currLevel.width == 1);

      for (int j = 0; j < currLevel.height; j++) {
        hWeight[0] = hNorm * (1.0f - hDecimal * j);
        hWeight[1] = hNorm;
        hWeight[2] = hNorm * hDecimal * (j + 1);

        result[0] = result[1] = result[2] = result[3] = 0.0f;
        for (int jj = 0; jj < hSupport; jj++) {
          uint8_to_float(input, prevLevelMem + prevLevelPitch * (2 * j + jj));
          result[0] += hWeight[jj] * input[0];
          result[1] += hWeight[jj] * input[1];
          result[2] += hWeight[jj] * input[2];
          result[3] += hWeight[jj] * input[3];
        }

        // convert back to format of the texture
        float_to_uint8(currLevelMem + (currLevelPitch * j), result);
      }

      // case 3: reduction in both horizontal and vertical size
    } else {

      for (int j = 0; j < currLevel.height; j++) {
        hWeight[0] = hNorm * (1.0f - hDecimal * j);
        hWeight[1] = hNorm;
        hWeight[2] = hNorm * hDecimal * (j + 1);

        for (int i = 0; i < currLevel.width; i++) {
          wWeight[0] = wNorm * (1.0f - wDecimal * i);
          wWeight[1] = wNorm * 1.0f;
          wWeight[2] = wNorm * wDecimal * (i + 1);

          result[0] = result[1] = result[2] = result[3] = 0.0f;

          // convolve source image with a trapezoidal filter.
          // in the case of no rounding this is just a box filter of width 2.
          // in the general case, the support region is 3x3.
          for (int jj = 0; jj < hSupport; jj++)
            for (int ii = 0; ii < wSupport; ii++) {
              float weight = hWeight[jj] * wWeight[ii];
              uint8_to_float(input, prevLevelMem +
                                        prevLevelPitch * (2 * j + jj) +
                                        4 * (2 * i + ii));
              result[0] += weight * input[0];
              result[1] += weight * input[1];
              result[2] += weight * input[2];
              result[3] += weight * input[3];
            }

          // convert back to format of the texture
          float_to_uint8(currLevelMem + currLevelPitch * j + 4 * i, result);
        }
      }
    }
  }
}

}
