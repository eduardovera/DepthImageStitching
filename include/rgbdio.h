#ifndef RGBDIO_H
#define RGBDIO_H

#include <stdint.h>
#include <string>
#include <cstring>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>

namespace rgbd
{

typedef struct
{
    uint8_t r;
    uint8_t g;
    uint8_t b;

} RGB888Pixel;
typedef uint8_t ColorPixel;
typedef uint16_t DepthPixel;

template<class T>
class Frame
{
public:
    Frame( uint16_t width=0, uint16_t height=0, uint64_t timestamp=0, const T* data=NULL)
        : m_width(width)
        , m_height(height)
        , m_timestamp(timestamp)
        , m_data(NULL)
    {
        //NOTE: Load as identity Matrices
        std::memset(m_intrinsicMatrix, 0, 9 * sizeof(float));
        m_intrinsicMatrix[0] = m_intrinsicMatrix[4] = m_intrinsicMatrix[8] = 1.0f;

        std::memset(m_extrinsicMatrix, 0, 16 * sizeof(float));
        m_extrinsicMatrix[0] = m_extrinsicMatrix[5] = m_extrinsicMatrix[10] = m_extrinsicMatrix[15] = 1.0f;

        this->setData(data);
    }

    Frame(const Frame<T> &that)
    {
        this->setData(that.data());
        this->setExtrinsicMatrix(that.extrinsicMatrix());
        this->setIntrinsicMatrix(that.intrinsicMatrix());
        this->setTimestamp(that.timestamp());
        this->m_width = that.m_width;
        this->m_height = that.m_height;
    }

    Frame & operator =(const Frame<T> &that)
    {
        this->setData(that.data());
        this->setExtrinsicMatrix(that.extrinsicMatrix());
        this->setIntrinsicMatrix(that.intrinsicMatrix());
        this->setTimestamp(that.timestamp());
        this->m_width = that.m_width;
        this->m_height = that.m_height;
        return *this;
    }

    virtual ~Frame()
    {
        if(m_data != NULL)
        {
            delete [] m_data;
        }
    }

    inline uint16_t width() const
    {
        return m_width;
    }

    inline uint16_t height() const
    {
        return m_height;
    }

    inline uint64_t timestamp() const
    {
        return m_timestamp;
    }

    inline void setTimestamp(uint64_t timestamp)
    {
        m_timestamp = timestamp;
    }

    inline T* data() const
    {
        return m_data;
    }

    void setData(const T* data, uint8_t channel=1)
    {
        if(data != NULL)
        {
            if(m_data != NULL)
                delete [] m_data;
            m_data = new T[m_width*m_height*channel];
            std::memcpy(m_data, data, m_width * m_height * channel * sizeof(T));
        }
    }

    inline const float* intrinsicMatrix() const
    {
        return m_intrinsicMatrix;
    }

    inline void setIntrinsicMatrix(const float* matrix)
    {
        std::memcpy(m_intrinsicMatrix, matrix, 9 * sizeof(float));
    }

    inline const float* extrinsicMatrix() const
    {
        return m_extrinsicMatrix;
    }

    void setExtrinsicMatrix(const float* matrix)
    {
        std::memcpy(m_extrinsicMatrix, matrix, 16 * sizeof(float));
    }

    bool isValid() const
    {
        return m_data != NULL && m_width > 0 && m_height > 0 && m_timestamp > 0;
    }

    bool save(std::string filename, uint8_t channel=1)
    {
        std::ofstream out(filename.c_str(), std::ios::out | std::ios::binary);
        if(out.is_open())
        {
            std::cout << "width=" << m_width << " height=" << m_height << std::endl;

            out.write((char *) &m_width, sizeof(uint16_t));
            out.write((char *) &m_height, sizeof(uint16_t));
            out.write((char *) &m_timestamp, sizeof(uint64_t));
            out.write((char *) m_intrinsicMatrix, 9 * sizeof(float));
            out.write((char *) m_extrinsicMatrix, 16 * sizeof(float));
            out.write((char *) m_data, m_width * m_height * channel * sizeof(T));
            out.close();

            return true;
        }

        std::cerr << "Cannot save file " << filename << std::endl;
        return false;
    }

    bool load(std::string filename, uint8_t channel=1, bool legacy=false)
    {
        std::ifstream in(filename.c_str(), std::ios::in | std::ios::binary);
        if(in)
        {
            in.read((char *) &m_width, sizeof(uint16_t));
            in.read((char *) &m_height, sizeof(uint16_t));
            in.read((char *) &m_timestamp, sizeof(uint64_t));
            if(!legacy)
            {
                in.read((char *) m_intrinsicMatrix, 9 * sizeof(float));
            }
            else
            {
                std::cout << "Warning! You are using legacy mode!" << std::endl;
            }
            in.read((char *) m_extrinsicMatrix, 16 * sizeof(float));

            if(m_data != NULL)
                delete [] m_data;
            m_data = new T[m_width*m_height*channel];
            in.read((char *)m_data, m_width * m_height * channel * sizeof(T));

            std::cout << "filename: " << filename << std::endl;
            std::cout << "\twidth=" << m_width << " height=" << m_height << std::endl;
            // see how many bytes have been read
            std::cout << "\t" << in.gcount() << " bytes read" << std::endl;

            in.close();

            return true;

        }

        std::cerr << "Cannot open file " << filename << std::endl;
        return false;

    }


private:
    uint16_t m_width;
    uint16_t m_height;
    uint64_t m_timestamp;
    T* m_data;
    float m_intrinsicMatrix[9];
    float m_extrinsicMatrix[16];

};
typedef Frame<DepthPixel> DepthFrame;
typedef Frame<RGB888Pixel> RGBFrame;
typedef Frame<ColorPixel> ColorFrame;


class Utils
{
public:
    static bool buildPLYMesh(const DepthFrame &depthFrame, const RGBFrame &rgbFrame, std::string filename,  bool isZInMillimeters=true, uint16_t zMin=0, uint16_t zMax=10000)
    {
        std::ofstream out(filename.c_str(), std::ios::out);
        if(out.is_open() && depthFrame.isValid())
        {
            uint16_t *depth = depthFrame.data();
            const uint16_t w = depthFrame.width();
            const uint16_t h = depthFrame.height();

            // Parâmetros intrínsicos do Kinect
            // fx = 594.21f;
            // fy = 591.04f;
            // cx = 339.5f;
            // cy = 242.7f;
            const float *matrix = depthFrame.intrinsicMatrix();
            const float fx = matrix[0];
            const float fy = matrix[4];
            const float cx = matrix[2];
            const float cy = matrix[5];

            std::cerr << fx << " " << fy << " " << cx << " " << cy << std::endl;

            RGB888Pixel *colors = rgbFrame.data();
            RGB888Pixel color;
            color.r = 255;
            color.g = 0;
            color.b = 0;

            uint32_t mark[w*h];
            uint32_t verticeId = 0;

            std::stringstream sout;
            std::stringstream header;

            for(int v=0; v < h; ++v)
            {
                for(int u=0; u < w; ++u)
                {
                    float z = depth[u+w*v];
                    if(z > zMin && z < zMax)
                    {
                        if(rgbFrame.isValid())
                        {
                           color = colors[u+w*v];
                        }

                        if(isZInMillimeters)
                        {
                            //NOTE: http://pille.iwr.uni-heidelberg.de/~kinect01/doc/classdescription.html#kinectcloud-section
                            const float x = ((u - cx) * z )/fx;
                            const float y = ((v - cy) * z )/fy;
                            sout << std::fixed << x << " " << h-y << " " << z
                                 <<  " " << (uint32_t)color.r << " " << (uint32_t)color.g << " " << (uint32_t)color.b
                                 << std::endl;
                        }
                        else
                        {
                            sout << std::fixed << u << " " << h-v << " " << z
                                 <<  " " << (uint32_t)color.r << " " << (uint32_t)color.g << " " << (uint32_t)color.b
                                 << std::endl;
                        }
                        mark[u+w*v] = verticeId++;
                    }
                    else
                    {
                        mark[u+w*v] = 0;
                    }
                }
            }

            uint32_t faces = 0;
            for(int y=0; y < h; ++y)
            {
                for(int x=0; x < w; ++x)
                {
                    int Vx = x + 1 + y*w;

                    if(mark[Vx-1] && mark[Vx] && mark[Vx+w] && mark[Vx+w-1])
                    {
                        const float diff = 10.f;
                        float v1 = depth[Vx-1];
                        float v2 = depth[Vx];
                        float v3 = depth[Vx+w];
                        float v4 = depth[Vx+w-1];

                        // Somente cria as faces dos quads não "degenerados"
                        if( (std::abs(v1-v2) < diff) && (std::abs(v2-v3) < diff)  &&  (std::abs(v3-v4) < diff))
                        {
                            sout << "4 " << mark[Vx-1] << " " << mark[Vx] << " " << mark[Vx+w] << " " << mark[Vx+w-1] << std::endl;
                            faces++;
                        }
                    }
                }
            }

            header << "ply" << std::endl;
            header << "format ascii 1.0" << std::endl;
            header << "element vertex " << verticeId << std::endl;
            header << "property float x" << std::endl;
            header << "property float y" << std::endl;
            header << "property float z" << std::endl;
            header << "property uchar red" << std::endl;
            header << "property uchar green" << std::endl;
            header << "property uchar blue" << std::endl;
            header << "element face " << faces << std::endl;
            header << "property list uchar int vertex_index" << std::endl;
            header << "end_header" << std::endl;

            out << header.rdbuf() << sout.rdbuf();
            out.close();
            return true;
        }

        return false;
    }

    static bool buildOBJMesh(const DepthFrame &depthFrame, std::string filename, bool isZInMillimeters=true, uint16_t zMin=0, uint16_t zMax=10000)
    {
        std::ofstream out(filename.c_str(), std::ios::out);
        if(out.is_open() && depthFrame.isValid())
        {
            uint16_t *depth = depthFrame.data();
            const uint16_t w = depthFrame.width();
            const uint16_t h = depthFrame.height();

            const float *matrix = depthFrame.intrinsicMatrix();
            const float fx = matrix[0];
            const float fy = matrix[4];
            const float cx = matrix[2];
            const float cy = matrix[5];

            uint32_t mark[w*h];
            uint32_t verticeId = 1;

            std::stringstream sout;
            std::stringstream header;

            for(int v=0; v < h; ++v)
            {
                for(int u=0; u < w; ++u)
                {
                    float z = depth[u+w*v];
                    if(z > zMin && z < zMax)
                    {
                        if(isZInMillimeters)
                        {
                            //NOTE: http://pille.iwr.uni-heidelberg.de/~kinect01/doc/classdescription.html#kinectcloud-section
                            const float x = ((u - cx) * z )/fx;
                            const float y = ((v - cy) * z )/fy;
                            sout << "v " << std::fixed << w-x << " " << h-y << " " << z << std::endl;
                        }
                        else
                        {
                            sout << "v " << std::fixed << w-u << " " << h-v << " " << z << std::endl;
                        }

                        mark[u+w*v] = verticeId++;
                    }
                    else
                    {
                        mark[u+w*v] = 0;
                    }
                }
            }

            for(int y=0; y < h; ++y)
            {
                for(int x=0; x < w; ++x)
                {
                    int Vx = x + 1 + y*w;

                    if(mark[Vx-1] && mark[Vx] && mark[Vx+w] && mark[Vx+w-1])
                    {
                        const float diff = 10.f;
                        float v1 = depth[Vx-1];
                        float v2 = depth[Vx];
                        float v3 = depth[Vx+w];
                        float v4 = depth[Vx+w-1];

                        // Somente cria as faces dos quads não "degenerados"
                        if( (std::abs(v1-v2) < diff) && (std::abs(v2-v3) < diff)  &&  (std::abs(v3-v4) < diff))
                        {
                            sout << "f " << mark[Vx-1] << " " << mark[Vx] << " " << mark[Vx+w] << " " << mark[Vx+w-1] << std::endl;
                        }
                    }
                }
            }

            header << "#Comentario" << std::endl;
            out << header.rdbuf() << sout.rdbuf();
            out.close();
            return true;
        }

        return false;
    }
};

}

#endif // RGBDIO_H
