#include "planning/planner/bezier_path.h"
#include "planning/common/stdandard_def.h"

float Path::distance_resolution_ = 0.002;
float Path::angular_resolution_ = 0.002;

BezierPath::BezierPath(const Point2d &start_point,
                       const Point2d &end_point,
                       const std::vector<Point2d> &control_points,
                       const PathInfo_base &path_info,
                       float start_s) : Path(start_point, end_point, path_info, start_s),
                                        start_theta_(start_point.theta),
                                        end_theta_(end_point.theta)
{

    control_points_.clear();
    for (auto p : control_points)
    {
        Vec2d point(p.x, p.y);
        control_points_.emplace_back(point);
    }

    // StaticPoint2dList point_list;
    // Discretize(point_list);
}

void BezierPath::Discretize(StaticPoint2dList &point_list)
{
    // std::cout << "control_points_.size(): " << control_points_.size() << std::endl;
    std::cout << "point_list_size: " << point_list.size() << std::endl;
    if (control_points_.size() < 2)
        return;

    double temp_length = 0.0;
    length_ = 0.0;
    for (unsigned int i = 1; i < control_points_.size(); i++)
    {
        temp_length += sqrt(pow(control_points_[i](0) - control_points_[i - 1](0), 2) + pow(control_points_[i](1) - control_points_[i - 1](1), 2));
    }

    int point_num = temp_length / distance_resolution_;
    double delta_s = 0.0;
    double cur_s = start_s_;

    const static auto factorial = [](int n) {
        int fact = 1;

        for (int i = n; i > 0; i--)
            fact *= i;

        return fact;
    };

    const static auto combinatorial = [](int n, int k) {
        return factorial(n) / (factorial(k) * factorial(n - k));
    };

    std::vector<double> CList, CvList, CaList, CjList;
    int order = control_points_.size() - 1;
    for (int k = 0; k <= order; k++)
    {
        CList.emplace_back(combinatorial(order, k));
        if (k <= (order - 1))
            CvList.emplace_back(combinatorial(order - 1, k));
        if (k <= (order - 2))
            CaList.emplace_back(combinatorial(order - 2, k));
        if (k <= (order - 3))
            CjList.emplace_back(combinatorial(order - 3, k));
    }

    if (order == 1)
    {
        float delta_x = end_point_(0) - start_point_(0);
        float delta_y = end_point_(1) - start_point_(1);
        Vec2d direction(delta_x / temp_length, delta_y / temp_length);
        start_theta_ = atan2(delta_y, delta_x);
        end_theta_ = start_theta_;

        for (int i = 0; i <= point_num; i++)
        {
            StaticPoint2dInfo point(Vec2d(start_point_ + direction * delta_s),
                                    direction, 0.0, 0.0, cur_s, start_theta_, path_info_);
            point_list.push_back(point);
            delta_s += distance_resolution_;
            cur_s += distance_resolution_;
            length_ += distance_resolution_;
        }
        // int cflag = 0;
        // for (const auto &sp : point_list)
        // {
        //     if (fabs(sp.curvature()) > 0.001)
        //     {
        //         cflag = 0;
        //         std::cout << "curvature: " << sp.curvature() << std::endl;
        //     }
        //     else
        //     {
        //         if (cflag == 0)
        //         {
        //             std::cout << "curvature: " << sp.curvature() << std::endl;
        //             cflag = 1;
        //         }
        //     }
        // }
    }
    else if (order >= 2)
    {
        Vec2d last_point(start_point_);
        delta_s = 0.0;
        for (int i = 1; i <= point_num; i++)
        {
            double x = 0, dx = 0, ddx = 0, dddx = 0;
            double y = 0, dy = 0, ddy = 0, dddy = 0;
            double u = double(i) / double(point_num);

            for (int j = 0; j <= order; j++)
            {
                x += CList[j] * control_points_[j](0) * pow(u, j) * pow((1 - u), order - j);
                y += CList[j] * control_points_[j](1) * pow(u, j) * pow((1 - u), order - j);

                if (j < order)
                {
                    dx += CvList[j] * order * (control_points_[j + 1](0) - control_points_[j](0)) * pow(u, j) * pow((1 - u), order - j - 1);
                    dy += CvList[j] * order * (control_points_[j + 1](1) - control_points_[j](1)) * pow(u, j) * pow((1 - u), order - j - 1);
                }

                if (j < order - 1)
                {
                    ddx += CaList[j] * order * (order - 1) * (control_points_[j + 2](0) - 2 * control_points_[j + 1](0) + control_points_[j](0)) * pow(u, j) * pow((1 - u), order - j - 2);
                    ddy += CaList[j] * order * (order - 1) * (control_points_[j + 2](1) - 2 * control_points_[j + 1](1) + control_points_[j](1)) * pow(u, j) * pow((1 - u), order - j - 2);
                }

                if (j < order - 2)
                {
                    dddx += CjList[j] * order * (order - 1) * (order - 2) * (control_points_[j + 3](0) - 3 * control_points_[j + 2](0) + 3 * control_points_[j + 1](0) - control_points_[j](0)) * pow(u, j) * pow((1 - u), order - j - 3);
                    dddy += CjList[j] * order * (order - 1) * (order - 2) * (control_points_[j + 3](1) - 3 * control_points_[j + 2](1) + 3 * control_points_[j + 1](1) - control_points_[j](1)) * pow(u, j) * pow((1 - u), order - j - 3);
                }
            }

            double theta = atan2(dy, dx);
            Vec2d direction(std::cos(theta), std::sin(theta));

            const double a = dx * ddy - dy * ddx;
            auto norm_square = dx * dx + dy * dy;
            auto norm = std::sqrt(norm_square);
            const double norm_cubic = norm * norm_square;
            double curvature_at_u = a / norm_cubic;

            if (order > 2)
            {

                const double b = dx * dddy - dy * dddx;
                const double c = dx * ddx + dy * ddy;
                const double d = dx * dx + dy * dy;
                double dcurvature_at_u = (b * d - 3.0 * a * c) / (d * d * d);

                delta_s += sqrt(pow(x - last_point(0), 2) + pow(y - last_point(1), 2));
                length_ += sqrt(pow(x - last_point(0), 2) + pow(y - last_point(1), 2));

                StaticPoint2dInfo point(Vec2d(x, y), direction, curvature_at_u, dcurvature_at_u, cur_s + delta_s, theta, path_info_);
                point_list.emplace_back(point);
            }
            else
            {
                double dcurvature_at_u = 0;

                delta_s += sqrt(pow(x - last_point(0), 2) + pow(y - last_point(1), 2));
                length_ += sqrt(pow(x - last_point(0), 2) + pow(y - last_point(1), 2));

                StaticPoint2dInfo point(Vec2d(x, y), direction, curvature_at_u, dcurvature_at_u, cur_s + delta_s, theta, path_info_);
                point_list.emplace_back(point);
            }
            // std::cout << "curvature_at_u: " << curvature_at_u << std::endl;

            // std::cout << "curvature_at_u: " << curvature_at_u << "dcurvature_at_u: " << dcurvature_at_u << std::endl;

            last_point(0) = x;
            last_point(1) = y;
        }
 
    }

    start_theta_ = point_list.begin()->theta();
    end_theta_ = point_list.back().theta();
    length_ = delta_s;
    end_s_ = start_s_ + length_;
    // for (const auto &sp : point_list)
    // {
    //     if (fabs(sp.curvature()) > 0.001)
    //         std::cout << "curvature: " << sp.curvature() << std::endl;
    // }

    return;
}

void BezierPath::GetPose(float s, Point2d &pose) const
{
    //     if(std::isfinite(curvature_)) {
    //         float delta_s = s - start_s_;
    //         pose.x = start_point_[0] + delta_s * direction_[0];
    //         pose.y = start_point_[1] + delta_s * direction_[1];
    //         pose.theta = start_theta_;
    //     }
    //     else {
    //         //If the curvature is infinite, the position of the path will not change. In this case take s as the angular
    //         //distance theta
    //         pose.x = start_point_[0];
    //         pose.y = start_point_[1];
    //         pose.theta = s;
    //     }
}
