#include "planning/planner/bezier_path_smoother.h"
#include "planning/planner/bezier_path.h"
#include "planning/planner/geo_2d.h"
BezierPathSmoother::BezierPathSmoother()
{
    c1_ = 7.2364;
    c2_ = 0.4 * (sqrt(6) - 1);                // 0.579795896
    c3_ = (c2_ + 4) / (c1_ + 6);              //0.346
    c4_ = (c2_ + 4) * (c2_ + 4) / (54 * c3_); //1.12259315
    distance_resolution_ = 0.002;             //路径分辨率参数
}
BezierPathSmoother::~BezierPathSmoother() {}
bool BezierPathSmoother::Smooth(const std::vector<geometry_msgs::Point> &sample_points,
                                PathList &smoothed_paths)
{
    if (sample_points.size() < 2)
    {
        return false;
    }

    std::vector<std::vector<std::vector<double>>> smooth_points; //output
    std::vector<std::vector<double>> raw_points;                 //input

    for (std::vector<geometry_msgs::Point>::const_iterator iter = sample_points.begin(); iter != sample_points.end(); iter++)
    {
        std::vector<double> point;
        point.push_back((*iter).x);
        point.push_back((*iter).y);
        raw_points.push_back(point);
    }

    bool flag = Smooth(3.0, raw_points, smooth_points);

    if (flag)
    {
        for (auto c_points : smooth_points)
        {
            Point2d start(c_points.front()[0], c_points.front()[1], 0);
            Point2d end(c_points.back()[0], c_points.back()[1], 0);
            std::vector<Point2d> control_points;
            for (auto c_point : c_points)
            {
                Point2d ppp(c_point[0], c_point[1], 0);
                control_points.push_back(ppp);
            }
            double start_s = 0.0;
            std::string frame_id = "/map";
            PathInfo_base pi(0, false, true, frame_id);
            PathPtr bpath(new BezierPath(start,
                                         end,
                                         control_points,
                                         pi,
                                         start_s));
            smoothed_paths.emplace_back(bpath);
        }
    }

    return flag;
}

bool BezierPathSmoother::Smooth(double min_radius,
                                const std::vector<std::vector<double>> &raw_points,
                                std::vector<std::vector<std::vector<double>>> &smooth_points)
{
    if (raw_points.size() < 2)
    {
        //std::cout<<" The number of points must be greater than 2"<<std::endl;
        return false;
    }
    else if (raw_points.size() == 2)
    {
        smooth_points.emplace_back(raw_points);
        return true;
    }

    const int psize = raw_points.size();

    std::vector<double> v_dist;
    v_dist.reserve(psize);

    std::vector<double> v_tan_angle;
    v_tan_angle.reserve(psize);

    std::vector<double> v_angle_diff;
    v_angle_diff.reserve(psize);

    std::vector<double> v_beta;
    v_beta.reserve(psize);

    std::vector<double> v_sin_beta, v_cos_beta;
    v_sin_beta.reserve(psize);
    v_cos_beta.reserve(psize);

    std::vector<double> v_coeff_by_sin_cos;
    v_coeff_by_sin_cos.reserve(psize);

    std::vector<double> v_unit_x, v_unit_y;
    v_unit_x.reserve(psize);
    v_unit_y.reserve(psize);

    for (int i = 1; i < psize; i++)
    {
        double dx = raw_points[i][0] - raw_points[i - 1][0];
        double dy = raw_points[i][1] - raw_points[i - 1][1];
        double dist = sqrt(dx * dx + dy * dy);
        double unit_dx = dx / dist;
        double unit_dy = dy / dist;

        v_dist.emplace_back(dist);
        v_unit_x.emplace_back(unit_dx);
        v_unit_y.emplace_back(unit_dy);
        v_tan_angle.emplace_back(std::atan2(dy, dx));
    }

    for (int i = 1; i < psize - 1; i++)
    {
        double angle_bias = fabs(v_tan_angle[i] - v_tan_angle[i - 1]);

        if (fabs(angle_bias - M_PI) < 0.0001)
        {
            std::cout << "foldback in waypoints, can't smooth" << std::endl;
            return false;
        }

        if (angle_bias > M_PI)
            angle_bias = 2.0 * M_PI - angle_bias;

        v_angle_diff.emplace_back(angle_bias);
        double angle_beta = 0.5 * angle_bias;
        v_beta.emplace_back(angle_beta);
        v_sin_beta.emplace_back(sin(angle_beta));
        v_cos_beta.emplace_back(cos(angle_beta));
        v_coeff_by_sin_cos.emplace_back(v_sin_beta.back() / pow(v_cos_beta.back(), 2));
    }

    std::vector<double> v_smooth_radius_ceil;
    v_smooth_radius_ceil.reserve(psize);

    double inv_c4 = 1 / c4_;

    double radius_ceil_first = 10000.0;
    double radius_ceil_last = 10000.0;
    double min_smooth_radius = 10000.0;

    if (v_coeff_by_sin_cos[0] > 0)
        radius_ceil_first = v_dist[0] * inv_c4 / v_coeff_by_sin_cos[0];

    v_smooth_radius_ceil.emplace_back(radius_ceil_first);
    min_smooth_radius = radius_ceil_first;

    for (int i = 1; i < psize - 2; i++)
    {
        double radius_ceil = 10000.0;
        if (v_coeff_by_sin_cos[i - 1] > 0 || v_coeff_by_sin_cos[i] > 0)
            radius_ceil = v_dist[i] * inv_c4 / (v_coeff_by_sin_cos[i - 1] + v_coeff_by_sin_cos[i]);
        v_smooth_radius_ceil.emplace_back(radius_ceil);
        if (radius_ceil < min_smooth_radius)
            min_smooth_radius = radius_ceil; //求解所有平滑半径，找到最小平滑半径
    }

    if (v_coeff_by_sin_cos.back() > 0)
        radius_ceil_last = v_dist.back() * inv_c4 / v_coeff_by_sin_cos.back();
    v_smooth_radius_ceil.emplace_back(radius_ceil_last);
    if (min_smooth_radius > radius_ceil_last)
        min_smooth_radius = radius_ceil_last;

    if (min_smooth_radius < min_radius)
    {
        std::cout << " Minimum smoothing radius is less than min radius " << std::endl;
        return false;
    }

    std::vector<double> last_control_point = raw_points[0];
    for (int i = 0; i < psize - 2; i++)
    {
        if (v_coeff_by_sin_cos[i] < 0.0001) //直线
        {
            std::vector<std::vector<double>> cpoints;
            cpoints.emplace_back(last_control_point);
            cpoints.emplace_back(raw_points[i + 1]);

            smooth_points.emplace_back(cpoints);
            last_control_point = raw_points[i + 1];
        }
        else
        {
            double dist_in_smooth = c4_ * v_coeff_by_sin_cos[i] * min_radius;
            double para_h = c3_ * dist_in_smooth;
            double para_g = c2_ * para_h;
            double para_k = 6 * c3_ * v_cos_beta[i] * dist_in_smooth / (c2_ + 4);

            double dx_prior = v_unit_x[i];
            double dy_prior = v_unit_y[i];
            double dx_after = v_unit_x[i + 1];
            double dy_after = v_unit_y[i + 1];
            //
            std::vector<double> point_at_mid = raw_points[i + 1];

            std::vector<double> c_point_B0 = {point_at_mid[0] - dist_in_smooth * dx_prior,
                                              point_at_mid[1] - dist_in_smooth * dy_prior};
            std::vector<double> c_point_B1 = {c_point_B0[0] + para_g * dx_prior,
                                              c_point_B0[1] + para_g * dy_prior};
            std::vector<double> c_point_B2 = {c_point_B1[0] + para_h * dx_prior,
                                              c_point_B1[1] + para_h * dy_prior};

            std::vector<double> c_point_E0 = {point_at_mid[0] + dist_in_smooth * dx_after,
                                              point_at_mid[1] + dist_in_smooth * dy_after};
            std::vector<double> c_point_E1 = {c_point_E0[0] - para_g * dx_after,
                                              c_point_E0[1] - para_g * dy_after};
            std::vector<double> c_point_E2 = {c_point_E1[0] - para_h * dx_after,
                                              c_point_E1[1] - para_h * dy_after};

            std::vector<double> vec_B2_to_E2 = {c_point_E2[0] - c_point_B2[0],
                                                c_point_E2[1] - c_point_B2[1]};

            double dist_B2_to_E2 = sqrt(vec_B2_to_E2[0] * vec_B2_to_E2[0] + vec_B2_to_E2[1] * vec_B2_to_E2[1]);

            std::vector<double> unit_vec_B2_to_E2{vec_B2_to_E2[0] / dist_B2_to_E2, vec_B2_to_E2[1] / dist_B2_to_E2};

            std::vector<double> c_point_B3 = {c_point_B2[0] + para_k * unit_vec_B2_to_E2[0],
                                              c_point_B2[1] + para_k * unit_vec_B2_to_E2[1]};
            std::vector<double> c_point_E3 = {c_point_E2[0] - para_k * unit_vec_B2_to_E2[0],
                                              c_point_E2[1] - para_k * unit_vec_B2_to_E2[1]};

            std::vector<std::vector<double>> cpoints1 = {last_control_point, c_point_B0};
            std::vector<std::vector<double>> cpoints2 = {c_point_B0, c_point_B1, c_point_B2, c_point_B3};
            std::vector<std::vector<double>> cpoints3 = {c_point_E3, c_point_E2, c_point_E1, c_point_E0};

            smooth_points.emplace_back(cpoints1);
            smooth_points.emplace_back(cpoints2);
            smooth_points.emplace_back(cpoints3);

            last_control_point = c_point_E0;
        }
    }

    if (v_coeff_by_sin_cos.back() > 0)
    {
        std::vector<std::vector<double>> cpoints = {last_control_point, raw_points.back()};
        smooth_points.emplace_back(cpoints);
    }

    return true;
}
void BezierPathSmoother::Discretize(const std::vector<std::vector<double>> &sample_points, const std::vector<double> &X, const std::vector<double> &Y, StaticPoint2dList &point_list)
{
    tk::spline s;
    double last_x = 0, x = 0;
    double last_y = 0, y = 0;
    double theta = 0;
    double delta_s = 0;
    double temp_length = 0;
    s.set_points(X, Y);

    for (int j = 0; j < sample_points.size(); j++)
    {
        temp_length = sqrt(pow(sample_points[j + 1][0] - sample_points[j][0], 2) + pow(sample_points[j + 1][1] - sample_points[j][1], 2));
        int point_num = temp_length / distance_resolution_;
        double delta_x = (sample_points[j + 1][0] - sample_points[j][0]) / point_num;
        x = sample_points[j + 1][0];
        y = s(x);
        last_x = x;
        last_y = y;
        for (int i = 0; i <= point_num; i++)
        {
            x += delta_x * i;
            y += s(x);
            delta_s += sqrt(pow(x - last_x, 2) + pow(y - last_y, 2));
            last_x = x;
            last_y = y;

            Vec2d direction(std::cos(theta), std::sin(theta));

            double curvature_at_u = 0;
            double dcurvature_at_u = 0;
            double start_s = 0.0;
            std::string frame_id = "/map";
            PathInfo_base pi(0, false, true, frame_id);
            StaticPoint2dInfo point(Vec2d(x, y), direction, curvature_at_u, dcurvature_at_u, start_s + delta_s, theta, pi);
            point_list.emplace_back(point);
        }
    }

    return;
}

void BezierPathSmoother::PathInfo::AddPath(const PathPtr &path)
{
    paths.push_back(path);

    path->Discretize(points);
    // for (const auto &sp : points)
    // {
    //     if (fabs(sp.curvature()) > 0.001)
    //     {
    //         std::cout << "curvature: " << sp.curvature() << std::endl;
    //     }
    // }
    // std::cout << std::endl;
    StaticPoint2dIter last_iter = points.end();
    std::advance(last_iter, -1);
    end_points.push_back(last_iter);
    if (current_point == points.end())
    {
        current_point = points.begin();
    }
}

int BezierPathSmoother::PathInfo::clear_between(const StaticPoint2dIter &begin,
                                                const StaticPoint2dIter &end)
{
    int finished_end_points_number = 0;
    StaticPoint2dIterList::const_iterator next_end_point_iter = end_points.begin();

    StaticPoint2dIter iter = begin;
    //Iterate the points between begin and end, check if the end point is in the list,
    //and count the finished path number
    while (iter != end)
    {
        if (iter == *next_end_point_iter)
        {
            ++next_end_point_iter;
            paths.pop_front();
            points.erase(points.begin(), iter);
            end_points.pop_front();
            ++finished_end_points_number;
        }
        ++iter;
    }
    return finished_end_points_number;
}

void BezierPathSmoother::AddSmoothedPath(PathList &smoothed_path_list)
{

    float start_s = 0.0;
    if (smoothed_path_info_.paths.size() > 0)
    {
        start_s = smoothed_path_info_.paths.back()->end_s();
    }
    StaticPoint2dIter start_smoothed_point = std::prev(smoothed_path_info_.points.end());
    StaticPoint2dIter prev_smoothed_point;
    // std::cout << "size: " << smoothed_path_list.size() << std::endl;
    //Discretize the smoothed path and get the smoothed path point
    for (PathListIter iter = smoothed_path_list.begin();
         iter != smoothed_path_list.end();
         ++iter)
    {
        (*iter)->set_start_s(start_s);
        smoothed_path_info_.AddPath(*iter);
        // for (const auto &sp : smoothed_path_info_.points)
        // {
        //     if (fabs(sp.curvature()) > 0.001)
        //     {
        //         std::cout << "curture_sp: " << sp.curvature() << std::endl;
        //     }
        // }
        // if (start_smoothed_point == smoothed_path_info_.points.end())
        // {
        //     start_smoothed_point = smoothed_path_info_.points.begin();
        // }
        // else
        // {
        //     prev_smoothed_point = std::prev(start_smoothed_point);
        //     //If there is only one point in the previous list, there is no need to connect the points
        //     if (prev_smoothed_point != smoothed_path_info_.points.end())
        //     {
        //         (*start_smoothed_point).connect_point(*prev_smoothed_point, smoothed_path_info_.points.back());
        //     }
        // }

        // start_s = (*iter)->end_s();
        // start_smoothed_point = std::prev(smoothed_path_info_.points.end());
    }
    // int cflag = 0;
    // for (const auto &sp : smoothed_path_info_.points)
    // {
    //     if (fabs(sp.curvature()) > 0.001)
    //     {
    //         cflag = 0;
    //         std::cout << "curture_sp: " << sp.curvature() << std::endl;
    //     }
    //     else
    //     {
    //         if (cflag == 0)
    //         {
    //             std::cout << "curture_sp: " << sp.curvature() << std::endl;
    //             cflag = 1;
    //         }
    //     }
    // }
}
