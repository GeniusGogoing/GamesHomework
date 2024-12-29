// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

static float cross2D(const Vector2f& a, const Vector2f& b)
{
    return a.x() * b.y() - a.y() * b.x();
}


static bool insideTriangle(float x, float y, const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    Vector2f p =  Vector2f(x, y);
	Vector2f v0 = Vector2f(_v[0].x(), _v[0].y());
	Vector2f v1 = Vector2f(_v[1].x(), _v[1].y());
	Vector2f v2 = Vector2f(_v[2].x(), _v[2].y());
	Vector2f s0 = v1 - v0;
	Vector2f s1 = v2 - v1;
	Vector2f s2 = v0 - v2;
    Vector2f p0 = p - v0;
    Vector2f p1 = p - v1;
    Vector2f p2 = p - v2;
    float a = cross2D(s0, p0);
    float b = cross2D(s1, p1);
    float c =  cross2D(s2, p2);
    return a > 0 && b > 0 && c > 0;
}


static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        //rasterize_triangle(t);

        //rasterize_wireframe(t);

        //rasterize_triangle_msaa_with_black_edge(t);

        rasterize_triangle_msaa(t);
    }
}

void rst::rasterizer::rasterize_wireframe(const Triangle& t)
{
    draw_line(t.v[2], t.v[0]);
    draw_line(t.v[0], t.v[1]);
    draw_line(t.v[1], t.v[2]);
}

// Bresenham's line drawing algorithm
// Code taken from a stack overflow answer: https://stackoverflow.com/a/16405254
void rst::rasterizer::draw_line(Eigen::Vector3f begin, Eigen::Vector3f end)
{
    auto x1 = begin.x();
    auto y1 = begin.y();
    auto x2 = end.x();
    auto y2 = end.y();

    Eigen::Vector3f line_color = { 255, 255, 255 };

    int x, y, dx, dy, dx1, dy1, px, py, xe, ye, i;

    dx = x2 - x1;
    dy = y2 - y1;
    dx1 = fabs(dx);
    dy1 = fabs(dy);
    px = 2 * dy1 - dx1;
    py = 2 * dx1 - dy1;

    if (dy1 <= dx1)
    {
        if (dx >= 0)
        {
            x = x1;
            y = y1;
            xe = x2;
        }
        else
        {
            x = x2;
            y = y2;
            xe = x1;
        }
        Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
        set_pixel(point, line_color);
        for (i = 0; x < xe; i++)
        {
            x = x + 1;
            if (px < 0)
            {
                px = px + 2 * dy1;
            }
            else
            {
                if ((dx < 0 && dy < 0) || (dx > 0 && dy > 0))
                {
                    y = y + 1;
                }
                else
                {
                    y = y - 1;
                }
                px = px + 2 * (dy1 - dx1);
            }
            //            delay(0);
            Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
            set_pixel(point, line_color);
        }
    }
    else
    {
        if (dy >= 0)
        {
            x = x1;
            y = y1;
            ye = y2;
        }
        else
        {
            x = x2;
            y = y2;
            ye = y1;
        }
        Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
        set_pixel(point, line_color);
        for (i = 0; y < ye; i++)
        {
            y = y + 1;
            if (py <= 0)
            {
                py = py + 2 * dx1;
            }
            else
            {
                if ((dx < 0 && dy < 0) || (dx > 0 && dy > 0))
                {
                    x = x + 1;
                }
                else
                {
                    x = x - 1;
                }
                py = py + 2 * (dx1 - dy1);
            }
            //            delay(0);
            Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
            set_pixel(point, line_color);
        }
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    // 三角形屏幕顶点坐标
    auto v = t.toVector4();
    
    // 确认三角形包围盒
    float xmin = std::floor(std::min(v[0].x(), std::min(v[1].x(), v[2].x())));
    float xmax = std::ceil(std::max(v[0].x(), std::max(v[1].x(), v[2].x())));
    float ymin = std::floor(std::min(v[0].y(), std::min(v[1].y(), v[2].y())));
    float ymax = std::ceil(std::max(v[0].y(), std::max(v[1].y(), v[2].y())));

    // 遍历包围盒像素
    for (float x = xmin; x <= xmax; ++x) 
    {
        for (float y = ymin; y <= ymax; ++y)
        {
            int index = get_index(x, y);
            // 判断是否在三角形内
            if (insideTriangle(x + 0.5, y + 0.5, t.v)) 
            {
                // 计算重心坐标
                float alpha, beta, gamma;
                std::tie(alpha,beta,gamma) =  computeBarycentric2D(x+0.5, y+0.5, t.v);

                // 计算插值深度
                float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                // 然后可以得到任意属性插值 这里其实w_reciprocal就等于插值深度 这里只是为了展示任意属性插值的用法
                float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                z_interpolated *= w_reciprocal;

                // 比较深度缓存 如果离相机更近 则绘制该像素 且更新深度缓存
                // 这里depth_buf采用的是正值记录
                if (-z_interpolated < depth_buf[index]) {
					depth_buf[index] = -z_interpolated;
					set_pixel({ x, y, 1 }, t.getColor());
                }
            }
        }
    }
}

void rst::rasterizer::rasterize_triangle_msaa_with_black_edge(const Triangle& t) 
{
    // 三角形屏幕顶点坐标
    auto v = t.toVector4();

    // MSAA采样间隔
    Vector2f delta = { 0.25,0.75 };

    // 确认三角形包围盒
    float xmin = std::floor(std::min(v[0].x(), std::min(v[1].x(), v[2].x())));
    float xmax = std::ceil(std::max(v[0].x(), std::max(v[1].x(), v[2].x())));
    float ymin = std::floor(std::min(v[0].y(), std::min(v[1].y(), v[2].y())));
    float ymax = std::ceil(std::max(v[0].y(), std::max(v[1].y(), v[2].y())));

    // 遍历包围盒像素
    for (float x = xmin; x <= xmax; ++x)
    {
        for (float y = ymin; y <= ymax; ++y)
        {
            // 每个像素采样四次
            float sampleX, sampleY;
            float zMin = std::numeric_limits<float>::max();
            // 每有一个采样点在三角形内 就增加一个单位的比重
            float counter = 0;
            for (int i = 0; i < 2; i++) 
            {
                for (int j = 0; j < 2; j++) 
                {
                    sampleX = x + delta[i];
                    sampleY = y + delta[j];
                    // 判断是否在三角形内
                    if (insideTriangle(sampleX, sampleY, t.v))
                    {

                        // 计算重心坐标
                        float alpha, beta, gamma;
                        std::tie(alpha, beta, gamma) = computeBarycentric2D(sampleX, sampleY, t.v);

                        // 计算插值深度
                        float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                        // 然后可以得到任意属性插值 这里其实w_reciprocal就等于插值深度 这里只是为了展示任意属性插值的用法
                        float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                        z_interpolated *= w_reciprocal;

                        // 更新最小深度值
                        zMin = zMin > -z_interpolated ? -z_interpolated : zMin;

                        // 增加比重
                        counter += delta[0];
                    }
                }
            }

            int index = get_index(x, y);
            // 比较深度缓存 如果离相机更近 则绘制该像素 且更新深度缓存
            // 这里depth_buf采用的是正值记录
            if (counter > 0 && zMin < depth_buf[index]) {
                depth_buf[index] = zMin;
                // ERROR
                // 这里的处理对于每个像素只保留了顶层三角形在此处的颜色  导致在和其他三角形重叠的边缘看起来有黑边（颜色太暗） 该像素颜色应该考虑其他三角形在此处的颜色占比
                set_pixel({ x, y, 1 }, t.getColor() * counter);
            }
        }
    }
}

void rst::rasterizer::rasterize_triangle_msaa(const Triangle& t)
{
    // 三角形屏幕顶点坐标
    auto v = t.toVector4();

    // MSAA采样间隔
    Vector2f delta = { 0.25,0.75 };

    // 确认三角形包围盒
    float xmin = std::floor(std::min(v[0].x(), std::min(v[1].x(), v[2].x())));
    float xmax = std::ceil(std::max(v[0].x(), std::max(v[1].x(), v[2].x())));
    float ymin = std::floor(std::min(v[0].y(), std::min(v[1].y(), v[2].y())));
    float ymax = std::ceil(std::max(v[0].y(), std::max(v[1].y(), v[2].y())));

    // 遍历包围盒像素
    for (float x = xmin; x <= xmax; ++x)
    {
        for (float y = ymin; y <= ymax; ++y)
        {
            // 每个像素采样四次
            float sampleX, sampleY;
            int sampleIndex = get_index(x, y) * 4;
            float zMin = std::numeric_limits<float>::max();
            for (int i = 0; i < 2; i++)
            {
                for (int j = 0; j < 2; j++)
                {
                    sampleX = x + delta[i];
                    sampleY = y + delta[j];
                    // 判断是否在三角形内
                    if (insideTriangle(sampleX, sampleY, t.v))
                    {
                        // 计算重心坐标
                        float alpha, beta, gamma;
                        std::tie(alpha, beta, gamma) = computeBarycentric2D(sampleX, sampleY, t.v);

                        // 计算插值深度
                        float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                        // 然后可以得到任意属性插值 这里其实w_reciprocal就等于插值深度 这里只是为了展示任意属性插值的用法
                        float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                        z_interpolated *= w_reciprocal;

                        // 比较MSAA深度缓存 尝试更新
                        if (-z_interpolated < msaa_depth_buf[sampleIndex + i * 2 + j]) {
                            msaa_depth_buf[sampleIndex + i * 2 + j] = -z_interpolated;
                            msaa_frame_buf[sampleIndex + i * 2 + j] = t.getColor() / 4;
                        }

                        // 更新最小深度值
                        zMin = zMin > -z_interpolated ? -z_interpolated : zMin;
                    }
                }
            }

            // 这边不再做深度测试 直接根据MSAA缓存更新深度缓冲和颜色缓存
            int index = get_index(x, y);
            depth_buf[index] = depth_buf[index] > zMin ? zMin : depth_buf[index];
            set_pixel({ x, y, 1 }, msaa_frame_buf[sampleIndex] + msaa_frame_buf[sampleIndex + 1] + msaa_frame_buf[sampleIndex + 2] + msaa_frame_buf[sampleIndex + 3]);
        }
    }
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }
    if ((buff & rst::Buffers::MSAA_COLOR) == rst::Buffers::MSAA_COLOR)
    {
        std::fill(msaa_depth_buf.begin(), msaa_depth_buf.end(), std::numeric_limits<float>::infinity());
    }
    if ((buff & rst::Buffers::MSAA_DEPTH) == rst::Buffers::MSAA_DEPTH)
    {
        std::fill(msaa_frame_buf.begin(), msaa_frame_buf.end(), Eigen::Vector3f{ 0, 0, 0 });
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
    msaa_frame_buf.resize(w * h * 4);
    msaa_depth_buf.resize(w * h * 4);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

// clang-format on