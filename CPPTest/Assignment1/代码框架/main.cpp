#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>
using namespace Eigen;
using namespace std;

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    float rad = rotation_angle / 180 * MY_PI;
    Matrix4f rotate;
    rotate << cos(rad), -sin(rad), 0, 0,
        sin(rad), cos(rad), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
	return rotate * model;
    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.
    float height = 2 * zNear * tan(eye_fov / 2);
	float width = height * aspect_ratio;
	Matrix4f proj;
    proj << zNear,0,0,0,
        0,zNear,0,0,
    0,0,zNear+zFar,-zNear*zFar, 
	0,0,-1,0;

    Matrix4f ortho;
    ortho << 2 / width, 0, 0, 0,
        0, 2 / height, 0, 0,
        0, 0, 2 / (zFar - zNear), (zFar + zNear) / 2,
        0, 0, 0, 1;
	return ortho * proj * projection;
}

Eigen::Matrix4f get_rotation(Vector3f axis, float angle) {//任意轴旋转矩阵（罗德里格斯旋转公式，默认轴过原点）
    double fangle = angle / 180 * MY_PI;
    Eigen::Matrix4f I, N, Rod;
    Eigen::Vector4f axi;

    axi << axis.x(), axis.y(), axis.z(), 0;

    I.Identity();

    N << 0, -axis.z(), axis.y(), 0,
        axis.z(), 0, -axis.x(), 0,
        -axis.y(), axis.x(), 0, 0,
        0, 0, 0, 1;

    Rod = cos(fangle) * I + (1 - cos(fangle)) * axi * axi.transpose() + sin(fangle) * N;
    Rod(3, 3) = 1;//这里要注意，非齐次坐标的公式应用在齐次坐标上时记得运算完成后把矩阵的右下角改为1，否则会导致图形比例错误
    return Rod;
}


int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    // 绕经过原点的任意轴旋转
    Vector3f raxis = {0,0,1};
    float ra;
    float rangle = 0;
    std::cout << "Please enter the axis and angle:" << std::endl;
    std::cin >> raxis.x() >> raxis.y() >> raxis.z() >> ra;//定义罗德里格斯旋转轴和每次旋转的角度

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));
        r.set_rodrigues(get_rotation(raxis, rangle));
       

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
        else if (key == 'q') {
            rangle += ra;
        }
        else if (key == 'e') {
            rangle -= ra;
        }
    }

    return 0;
}
