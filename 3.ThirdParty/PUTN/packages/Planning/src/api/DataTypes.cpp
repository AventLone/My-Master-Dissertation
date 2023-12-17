#include "api/DataTypes.h"

namespace putn
{
Node::Node(const Node& node)
{
    children = node.children;
    parent = node.parent;
    position = node.position;
    cost = node.cost;

    plane = std::make_shared<Plane>();

    if (node.plane != nullptr) *plane = *node.plane;
}

Plane::Plane(const Eigen::Vector3d& p_surface, std::shared_ptr<World> world, double radius, const FitPlaneArg& arg)
{
    init_coord = project2plane(p_surface);
    Eigen::Vector3d ball_center = world->coordRounding(p_surface);
    float resolution = world->getResolution();

    int fit_num = static_cast<int>(radius / resolution);
    Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> vac(2 * fit_num + 1, 2 * fit_num + 1);
    int vac_cout_init = (2 * fit_num + 1) * (2 * fit_num + 1);
    for (int i = -fit_num; i <= fit_num; i++)
    {
        for (int j = -fit_num; j <= fit_num; j++)
        {
            vac(i + fit_num, j + fit_num) = false;
            for (int k = -3; k <= 3; k++)
            {
                Eigen::Vector3d point = ball_center + resolution * Eigen::Vector3d(i, j, k);

                if (world->isInsideBorder(point) && !world->isFree(point))
                {
                    plane_points.push_back(point);
                    if (!vac(i + fit_num, j + fit_num))
                    {
                        vac(i + fit_num, j + fit_num) = true;
                        vac_cout_init--;
                    }
                }
            }
        }
    }

    size_t pt_num = plane_points.size();
    Eigen::Vector3d center;
    for (const auto& pt : plane_points) center += pt;
    center /= pt_num;
    Eigen::MatrixXd A(pt_num, 3);
    for (size_t i = 0; i < pt_num; i++) A.row(i) = plane_points[i] - center;

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullV);
    normal_vector = svd.matrixV().col(2);

    /* Calculate indicator1: flatness */
    float flatness = 0;
    for (size_t i = 0; i < pt_num; i++)
    {
        flatness += powf(normal_vector.dot(A.row(i)), 4);
    }
    flatness /= (1 + pt_num);

    /* Calculate indicator2: slope */
    Eigen::Vector3d z_axies(0, 0, 1);
    float slope = 180.0f * (float)std::acos(z_axies.dot(normal_vector)) / M_PI;

    /* Calculate indicator3: sparsity */
    float sparsity = 0.0f;
    if (vac_cout_init > 0)
    {
        int vac_cout = 0;
        Eigen::MatrixXd M_vac(2, vac_cout_init);
        for (int i = 0; i < vac.rows(); i++)
        {
            for (int j = 0; j < vac.cols(); j++)
            {
                if (!vac(i, j))
                {
                    M_vac(0, vac_cout) = i;
                    M_vac(1, vac_cout) = j;
                    vac_cout++;
                }
            }
        }

        Eigen::MatrixXd meanVec = M_vac.colwise().mean();
        Eigen::MatrixXd zeroMeanMat = M_vac;
        Eigen::RowVectorXd meanVecRow(Eigen::RowVectorXd::Map(meanVec.data(), M_vac.cols()));
        zeroMeanMat.rowwise() -= meanVecRow;
        Eigen::MatrixXd covMat = (zeroMeanMat.adjoint() * zeroMeanMat) / float(M_vac.rows());
        float trace = (covMat.transpose() * covMat(0, 0)).trace();
        float ratio = vac_cout / (float)(vac.rows() * vac.cols());

        if (ratio > arg.ratio_max)
        {
            sparsity = 1;
        }
        else if (ratio > arg.ratio_min && ratio < arg.ratio_max && (1 / trace) > arg.conv_thre)
        {
            sparsity = (ratio - arg.ratio_min) / (arg.ratio_max - arg.ratio_min);
        }
        else
        {
            sparsity = 0;
        }
    }

    /* The traversability is linear combination of the three indicators. */
    traversability = arg.w_total * (arg.w_flatness * flatness + arg.w_slope * slope + arg.w_sparsity * sparsity);
    traversability = (1 < traversability) ? 1 : traversability;
}

void World::clearMap()
{
    if (exist_map)
    {
        for (int i = 0; i < index_count(0); i++)
        {
            for (int j = 0; j < index_count(1); j++)
            {
                delete[] grid_map[i][j];
                grid_map[i][j] = nullptr;
            }
            delete[] grid_map[i];
            grid_map[i] = nullptr;
        }
        delete[] grid_map;
        grid_map = nullptr;
    }
}

void World::initGridMap(const Eigen::Vector3d& lowerbound, const Eigen::Vector3d& upperbound)
{
    mLowerBound = lowerbound;
    mUpperBound = upperbound;
    index_count = ((mUpperBound - mLowerBound) / mResolution).cast<int>() + Eigen::Vector3i::Ones();
    grid_map = new bool**[index_count(0)];
    for (int i = 0; i < index_count(0); i++)
    {
        grid_map[i] = new bool*[index_count(1)];
        for (int j = 0; j < index_count(1); j++)
        {
            grid_map[i][j] = new bool[index_count(2)];
            memset(grid_map[i][j], true, index_count(2) * sizeof(bool));
        }
    }
    exist_map = true;
}

void World::initGridMap(const pcl::PointCloud<pcl::PointXYZ>& cloud)
{
    if (cloud.points.empty())
    {
        std::cerr << "Can not initialize the map with an empty point cloud!" << std::endl;
        return;
    }
    clearMap();

    for (const auto& pt : cloud.points)
    {
        if (pt.x < mLowerBound(0)) mLowerBound(0) = pt.x;
        if (pt.y < mLowerBound(1)) mLowerBound(1) = pt.y;
        if (pt.z < mLowerBound(2)) mLowerBound(2) = pt.z;
        if (pt.x > mUpperBound(0)) mUpperBound(0) = pt.x;
        if (pt.y > mUpperBound(1)) mUpperBound(1) = pt.y;
        if (pt.z + 1.0 > mUpperBound(2)) mUpperBound(2) = pt.z + 1.0;
    }

    index_count = ((mUpperBound - mLowerBound) / mResolution).cast<int>() + Eigen::Vector3i::Ones();

    grid_map = new bool**[index_count(0)];
    for (int i = 0; i < index_count(0); i++)
    {
        grid_map[i] = new bool*[index_count(1)];
        for (int j = 0; j < index_count(1); j++)
        {
            grid_map[i][j] = new bool[index_count(2)];
            std::memset(grid_map[i][j], true, index_count(2) * sizeof(bool));
        }
    }
    exist_map = true;
}

bool World::collisionFree(const Node::ConstPtr& node_start, const Node::ConstPtr& node_end)
{
    Eigen::Vector3d e_z, e_y, e_x;
    Eigen::Matrix3d rotation_matrix;

    Eigen::Vector3d diff_pos = node_end->position - node_start->position;
    Eigen::Vector3d diff_norm_vector = node_end->plane->normal_vector - node_start->plane->normal_vector;

    size_t step = 20;
    bool is_free = true;

    for (size_t i = 0; i <= step; i++)
    {
        Eigen::Vector3d check_center = node_start->position + diff_pos * i / (double)step;
        e_z = node_start->plane->normal_vector + diff_norm_vector * i / (double)step;
        e_z.normalize();

        e_x = diff_pos - (diff_pos.dot(e_z)) * diff_pos;
        e_x.normalize();

        e_y = e_z.cross(e_x);

        rotation_matrix << e_x(0), e_y(0), e_z(0), e_x(1), e_y(1), e_z(1), e_x(2), e_y(2), e_z(2);

        Eigen::Vector3d check_point;
        for (int y = -2; y <= 2; y++)
        {
            for (int z = -2; z <= 2; z++)
            {
                check_point = check_center + rotation_matrix * Eigen::Vector3d(0, 0.15 * y, 0.1 * z);
                if (!isFree(check_point)) return false;
            }
        }
    }
    return is_free;
}

bool World::project2surface(float x, float y, Eigen::Vector3d& p_surface)
{
    bool ifsuccess = false;

    if (x >= mLowerBound(0) && x <= mUpperBound(0) && y >= mLowerBound(1) && y <= mUpperBound(1))
    {
        for (float z = mLowerBound(2); z < mUpperBound(2); z += mResolution)
        {
            if (!isFree(x, y, z) && isFree(x, y, z + mResolution))
            {
                p_surface = Eigen::Vector3d(x, y, z);
                ifsuccess = true;
                break;
            }
        }
    }
    return ifsuccess;
}
}   // namespace putn
