//
// Created by wlt on 19-5-9.
//

#ifndef PROJECT_COLOR_H
#define PROJECT_COLOR_H

namespace vis {
    typedef float f32;
    typedef unsigned short u16;
    typedef short s16;
    typedef unsigned int u32;
    typedef int s32;
    typedef unsigned char u8;

    const f32 ColorRCP = 1.0f / 255.0f;

    template<typename T>
    inline T clamp(T x, T min, T max)
    {
        return std::max(std::min(x, max), min);
    }

    //! Creates a 16 bit A1R5G5B5 color
    inline u16 RGBA16(u32 r, u32 g, u32 b, u32 a = 0xFF) {
        return (u16) ((a & 0x80) << 8 |
                      (r & 0xF8) << 7 |
                      (g & 0xF8) << 2 |
                      (b & 0xF8) >> 3);
    }


    //! Creates a 16 bit A1R5G5B5 color
    inline u16 RGB16(u32 r, u32 g, u32 b) {
        return RGBA16(r, g, b);
    }


    //! Creates a 16bit A1R5G5B5 color, based on 16bit input values
    inline u16 RGB16from16(u16 r, u16 g, u16 b) {
        return (0x8000 |
                (r & 0x1F) << 10 |
                (g & 0x1F) << 5 |
                (b & 0x1F));
    }


    //! Converts a 32bit (X8R8G8B8) color to a 16bit A1R5G5B5 color
    inline u16 X8R8G8B8toA1R5G5B5(u32 color) {
        return (u16) (0x8000 |
                      (color & 0x00F80000) >> 9 |
                      (color & 0x0000F800) >> 6 |
                      (color & 0x000000F8) >> 3);
    }


    //! Converts a 32bit (A8R8G8B8) color to a 16bit A1R5G5B5 color
    inline u16 A8R8G8B8toA1R5G5B5(u32 color) {
        return (u16) ((color & 0x80000000) >> 16 |
                      (color & 0x00F80000) >> 9 |
                      (color & 0x0000F800) >> 6 |
                      (color & 0x000000F8) >> 3);
    }


    //! Converts a 32bit (A8R8G8B8) color to a 16bit R5G6B5 color
    inline u16 A8R8G8B8toR5G6B5(u32 color) {
        return (u16) ((color & 0x00F80000) >> 8 |
                      (color & 0x0000FC00) >> 5 |
                      (color & 0x000000F8) >> 3);
    }


    //! Convert A8R8G8B8 Color from A1R5G5B5 color
    /** build a nicer 32bit Color by extending dest lower bits with source high bits. */
    inline u32 A1R5G5B5toA8R8G8B8(u16 color) {
        return (((-((s32) color & 0x00008000) >> (s32) 31) & 0xFF000000) |
                ((color & 0x00007C00) << 9) | ((color & 0x00007000) << 4) |
                ((color & 0x000003E0) << 6) | ((color & 0x00000380) << 1) |
                ((color & 0x0000001F) << 3) | ((color & 0x0000001C) >> 2)
        );
    }


    //! Returns A8R8G8B8 Color from R5G6B5 color
    inline u32 R5G6B5toA8R8G8B8(u16 color) {
        return 0xFF000000 |
               ((color & 0xF800) << 8) |
               ((color & 0x07E0) << 5) |
               ((color & 0x001F) << 3);
    }


    //! Returns A1R5G5B5 Color from R5G6B5 color
    inline u16 R5G6B5toA1R5G5B5(u16 color) {
        return 0x8000 | (((color & 0xFFC0) >> 1) | (color & 0x1F));
    }


    //! Returns R5G6B5 Color from A1R5G5B5 color
    inline u16 A1R5G5B5toR5G6B5(u16 color) {
        return (((color & 0x7FE0) << 1) | (color & 0x1F));
    }



    //! Returns the alpha component from A1R5G5B5 color
    /** In Irrlicht, alpha refers to opacity.
    \return The alpha value of the color. 0 is transparent, 1 is opaque. */
    inline u32 getAlpha(u16 color) {
        return ((color >> 15) & 0x1);
    }


    //! Returns the red component from A1R5G5B5 color.
    /** Shift left by 3 to get 8 bit value. */
    inline u32 getRed(u16 color) {
        return ((color >> 10) & 0x1F);
    }


    //! Returns the green component from A1R5G5B5 color
    /** Shift left by 3 to get 8 bit value. */
    inline u32 getGreen(u16 color) {
        return ((color >> 5) & 0x1F);
    }


    //! Returns the blue component from A1R5G5B5 color
    /** Shift left by 3 to get 8 bit value. */
    inline u32 getBlue(u16 color) {
        return (color & 0x1F);
    }


    //! Returns the average from a 16 bit A1R5G5B5 color
    inline s32 getAverage(s16 color) {
        return ((getRed(color) << 3) + (getGreen(color) << 3) + (getBlue(color) << 3)) / 3;
    }


    inline u32 COLORARGB(u32 a, u32 r, u32 g, u32 b) {
        return (((a & 0xff) << 24) | ((r & 0xff) << 16) | ((g & 0xff) << 8) | (b & 0xff));
    }

    // r,g,b values are from 0 to 1
    // h = [0,360], s = [0,1], v = [0,1]
    //		if s == 0, then h = -1 (undefined)
    inline void HSVtoRGB(float *r, float *g, float *b, float h, float s, float v) {
        int i;
        float f, p, q, t;
        if (s == 0) {
            // achromatic (grey)
            *r = *g = *b = v;
            return;
        }
        h /= 60;            // sector 0 to 5
        i = (int) floor(h);
        f = h - i;            // factorial part of h
        p = v * (1 - s);
        q = v * (1 - s * f);
        t = v * (1 - s * (1 - f));
        switch (i) {
            case 0:
                *r = v;
                *g = t;
                *b = p;
                break;
            case 1:
                *r = q;
                *g = v;
                *b = p;
                break;
            case 2:
                *r = p;
                *g = v;
                *b = t;
                break;
            case 3:
                *r = p;
                *g = q;
                *b = v;
                break;
            case 4:
                *r = t;
                *g = p;
                *b = v;
                break;
            default:        // case 5:
                *r = v;
                *g = p;
                *b = q;
                break;
        }
    }

    inline void RGBtoYUV(u8 r, u8 g, u8 b, u8 &y, u8 &u, u8 &v) {
        y = ((66 * r + 129 * g + 25 * b + 128) >> 8) + 16;
        u = ((-38 * r - 74 * g + 112 * b + 128) >> 8) + 128;
        v = ((112 * r - 79 * g - 18 * b + 128) >> 8) + 128;
    }

    struct RGB{
        u8 r;
        u8 g;
        u8 b;

        RGB(u8 a,u8 b,u8 c)
        {
            r = a;
            g = b;
            b = c;
        }
    };

    static RGB colorRampList[5][11] =
     {
       {
               RGB(202, 111, 255),
               RGB(150, 111, 255),
               RGB(70, 111, 255),
               RGB(0, 111, 255),
               RGB(0, 50, 255),
               RGB(0, 0, 255),
               RGB(0, 0, 150),
               RGB(0, 70, 150),
               RGB(0, 150, 150),
               RGB(0, 255, 150),
               RGB(0, 255, 255)
       },
       {
               RGB(0, 255, 0),
               RGB(0, 255, 70),
               RGB(0, 255, 150),
               RGB(0, 150, 150),
               RGB(0, 70, 150),
               RGB(0, 0, 255),
               RGB(70, 0, 255),
               RGB(150, 0, 255),
               RGB(255, 0, 150),
               RGB(255, 0, 0),
               RGB(255, 0, 0)
       },
       {
               RGB(255, 0, 0),
               RGB(255, 180, 0),
               RGB(255, 120, 0),
               RGB(255, 0, 0),
               RGB(255, 0, 80),
               RGB(255, 0, 150),
               RGB(255, 120, 150),
               RGB(180, 120, 150),
               RGB(120, 120, 80),
               RGB(80, 180, 80),
               RGB(0, 255, 0)
       },
       {
               RGB(245, 245, 150),
               RGB(250, 245, 120),
               RGB(255, 255, 60),
               RGB(255, 220, 0),
               RGB(255, 120, 0),
               RGB(255, 0, 60),
               RGB(255, 0, 150),
               RGB(220, 0, 240),
               RGB(150, 20, 220),
               RGB(75, 30, 200),
               RGB(20, 20, 175)
       },
       {
               RGB(0, 0, 180),
               RGB(0, 120, 255),
               RGB(0, 220, 255),
               RGB(0, 220, 120),
               RGB(0, 220, 0),
               RGB(120, 220, 0),
               RGB(255, 220, 0),
               RGB(255, 120, 0),
               RGB(255, 90, 0),
               RGB(255, 60, 0),
               RGB(180, 0, 0)
       },
     };

    class ColorRamp {
    private:
        s32 m_difA;

        s32 m_difR;

        s32 m_difG;

        s32 m_difB;
    private:
        s32 m_begA;
        s32 m_begR;
        s32 m_begG;
        s32 m_begB;
        s32 m_tempA[10];
        s32 m_tempR[10];
        s32 m_tempG[10];
        s32 m_tempB[10];

        u32 m_colorBegin;
        u32 m_colorEnd;
        int m_nSelCursel;
    public:

        ColorRamp(u32 beginClr, u32 endClr)
                : m_colorBegin(beginClr)//,m_colorEnd(endClr)
        {
            m_begA = (beginClr >> 24);
            m_begR = ((beginClr >> 16) & 0xff);
            m_begG = ((beginClr >> 8) & 0xff);
            m_begB = (beginClr & 0xff);

            m_difA = (endClr >> 24) - m_begA;
            m_difR = ((endClr >> 16) & 0xff) - m_begR;
            m_difG = ((endClr >> 8) & 0xff) - m_begG;
            m_difB = ((endClr) & 0xff) - m_begB;
        }

        bool operator==(const ColorRamp &other) const {
            return other.m_colorBegin == m_colorBegin && other.m_colorEnd == m_colorEnd;
        }

        inline void GetColor4f(f32 scale, f32 &a, f32 &r, f32 &g, f32 &b) {
            //scale = clamp(scale, 0.f, 1.f);
            a = (scale * m_difA + m_begA) * ColorRCP;
            r = (scale * m_difR + m_begR) * ColorRCP;
            g = (scale * m_difG + m_begG) * ColorRCP;
            b = (scale * m_difB + m_begB) * ColorRCP;
        }

        ColorRamp(u32 beginClr, u32 endClr, int nStep)
                : m_colorBegin(beginClr), m_colorEnd(endClr) {
            m_begA = (beginClr >> 24);
            m_begR = ((beginClr >> 16) & 0xff);
            m_begG = ((beginClr >> 8) & 0xff);
            m_begB = (beginClr & 0xff);

            for (int i = 0; i < 10; i++) {
                m_tempA[i] = 255;
            }
            m_tempR[0] = 0;
            m_tempR[1] = 0;
            m_tempR[2] = 0;
            m_tempR[3] = 0;
            m_tempR[4] = 120;
            m_tempR[5] = 255;
            m_tempR[6] = 255;
            m_tempR[7] = 255;
            m_tempR[8] = 255;
            m_tempG[0] = 120;
            m_tempG[1] = 220;
            m_tempG[2] = 220;
            m_tempG[3] = 220;
            m_tempG[4] = 220;
            m_tempG[5] = 220;
            m_tempG[6] = 120;
            m_tempG[7] = 90;
            m_tempG[8] = 60;
            m_tempB[0] = 255;
            m_tempB[1] = 255;
            m_tempB[2] = 120;
            m_tempB[3] = 0;
            m_tempB[4] = 0;
            m_tempB[5] = 0;
            m_tempB[6] = 0;
            m_tempB[7] = 0;
            m_tempB[8] = 0;

            m_tempA[9] = (m_colorEnd >> 24);
            m_tempR[9] = ((m_colorEnd >> 16) & 0xff);
            m_tempG[9] = ((m_colorEnd >> 8) & 0xff);
            m_tempB[9] = (m_colorEnd & 0xff);

            m_nSelCursel = 4;
        }

        inline void GetColor4f(f32 scale, f32 &a, f32 &r, f32 &g, f32 &b, int step) {
            scale = clamp(scale, 0.f, 1.f);
            step = clamp(step, 1, 10);

            if (step == 1) {
                a = (scale * (m_tempA[0] - m_begA) + m_begA) * ColorRCP;
                r = (scale * (m_tempR[0] - m_begR) + m_begR) * ColorRCP;
                g = (scale * (m_tempG[0] - m_begG) + m_begG) * ColorRCP;
                b = (scale * (m_tempB[0] - m_begB) + m_begB) * ColorRCP;
            } else {
                a = (scale * (m_tempA[step - 1] - m_tempA[step - 2]) + m_tempA[step - 2]) * ColorRCP;
                r = (scale * (m_tempR[step - 1] - m_tempR[step - 2]) + m_tempR[step - 2]) * ColorRCP;
                g = (scale * (m_tempG[step - 1] - m_tempG[step - 2]) + m_tempG[step - 2]) * ColorRCP;
                b = (scale * (m_tempB[step - 1] - m_tempB[step - 2]) + m_tempB[step - 2]) * ColorRCP;
            }
        }

        inline void GetColor4ub(f32 scale, u8 &a, u8 &r, u8 &g, u8 &b, int step) {
            scale = clamp(scale, 0.f, 1.f);
            step = clamp(step, 1, 10);

            if (step == 1) {
                a = (u8)(scale * (m_tempA[0] - m_begA) + m_begA);
                r = (u8)(scale * (m_tempR[0] - m_begR) + m_begR);
                g = (u8)(scale * (m_tempG[0] - m_begG) + m_begG);
                b = (u8)(scale * (m_tempB[0] - m_begB) + m_begB);
            } else {
                a = (u8)(scale * (m_tempA[step - 1] - m_tempA[step - 2]) + m_tempA[step - 2]);
                r = (u8)(scale * (m_tempR[step - 1] - m_tempR[step - 2]) + m_tempR[step - 2]);
                g = (u8)(scale * (m_tempG[step - 1] - m_tempG[step - 2]) + m_tempG[step - 2]);
                b = (u8)(scale * (m_tempB[step - 1] - m_tempB[step - 2]) + m_tempB[step - 2]);
            }
        }

        inline void GetColor4i(f32 scale, u32 &a, u32 &r, u32 &g, u32 &b, int step) {
            scale = clamp(scale, 0.f, 1.f);
            step = clamp(step, 1, 10);

            if (step == 1) {
                a = (u32) (scale * (m_tempA[0] - m_begA) + m_begA);
                r = (u32) (scale * (m_tempR[0] - m_begR) + m_begR);
                g = (u32) (scale * (m_tempG[0] - m_begG) + m_begG);
                b = (u32) (scale * (m_tempB[0] - m_begB) + m_begB);
            } else {
                a = (u32) (scale * (m_tempA[step - 1] - m_tempA[step - 2]) + m_tempA[step - 2]);
                r = (u32) (scale * (m_tempR[step - 1] - m_tempR[step - 2]) + m_tempR[step - 2]);
                g = (u32) (scale * (m_tempG[step - 1] - m_tempG[step - 2]) + m_tempG[step - 2]);
                b = (u32) (scale * (m_tempB[step - 1] - m_tempB[step - 2]) + m_tempB[step - 2]);
            }
        }

        inline int GetCurCursel() const { return m_nSelCursel; }


        inline void SetBegColor4f(u32 begColor, u32 endColor) {
            m_colorBegin = begColor;
            m_colorEnd = endColor;

            m_begA = (begColor >> 24);
            m_begR = ((begColor >> 16) & 0xff);
            m_begG = ((begColor >> 8) & 0xff);
            m_begB = (begColor & 0xff);

            for (int i = 0; i < 10; i++) {
                m_tempA[i] = 255;
            }
            m_tempR[0] = 0;
            m_tempR[1] = 0;
            m_tempR[2] = 0;
            m_tempR[3] = 0;
            m_tempR[4] = 120;
            m_tempR[5] = 255;
            m_tempR[6] = 255;
            m_tempR[7] = 255;
            m_tempR[8] = 255;
            m_tempG[0] = 120;
            m_tempG[1] = 220;
            m_tempG[2] = 220;
            m_tempG[3] = 220;
            m_tempG[4] = 220;
            m_tempG[5] = 220;
            m_tempG[6] = 120;
            m_tempG[7] = 90;
            m_tempG[8] = 60;
            m_tempB[0] = 255;
            m_tempB[1] = 255;
            m_tempB[2] = 120;
            m_tempB[3] = 0;
            m_tempB[4] = 0;
            m_tempB[5] = 0;
            m_tempB[6] = 0;
            m_tempB[7] = 0;
            m_tempB[8] = 0;

            m_tempA[9] = (endColor >> 24);
            m_tempR[9] = ((endColor >> 16) & 0xff);
            m_tempG[9] = ((endColor >> 8) & 0xff);
            m_tempB[9] = (endColor & 0xff);
        }

        inline u32 GetBeginColor() const { return m_colorBegin; }

        inline u32 GetEndColor() const { return m_colorEnd; }
    };

}

#endif //PROJECT_COLOR_H
