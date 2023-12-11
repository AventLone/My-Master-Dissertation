#include "api/putnDataType.h"

namespace putn
{
Node::Node(const Node& node)
{
    mChildren = node.mChildren;
    mParent = node.mParent;
    mPosition = node.mPosition;
    mCost = node.mCost;

    mPlane = std::make_shared<Plane>();

    if (node.mPlane != nullptr) *mPlane = *node.mPlane;
}

Plane::Plane(const Eigen::Vector3d& p_surface, World::Ptr world, double radius, const FitPlaneArg& arg)
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
    mTraversability = arg.w_total * (arg.w_flatness * flatness + arg.w_slope * slope + arg.w_sparsity * sparsity);
    mTraversability = (1 < mTraversability) ? 1 : mTraversability;
}

// World::World(float resolution) : mResolution(resolution)
// {
//     mLowerBound = gINF * Eigen::Vector3d::Ones();
//     mUpperBound = -gINF * Eigen::Vector3d::Ones();
//     mIdxCount = Eigen::Vector3i::Zero();
// }

void World::clearMap()
{
    if (mExistMap)
    {
        for (int i = 0; i < mIdxCount(0); i++)
        {
            for (int j = 0; j < mIdxCount(1); j++)
            {
                delete[] mGridMap[i][j];
                mGridMap[i][j] = nullptr;
            }
            delete[] mGridMap[i];
            mGridMap[i] = nullptr;
        }
        delete[] mGridMap;
        mGridMap = nullptr;
    }
}

void World::initGridMap(const Eigen::Vector3d& lowerbound, const Eigen::Vector3d& upperbound)
{
    mLowerBound = lowerbound;
    mUpperBound = upperbound;
    mIdxCount = ((mUpperBound - mLowerBound) / mResolution).cast<int>() + Eigen::Vector3i::Ones();
    mGridMap = new bool**[mIdxCount(0)];
    for (int i = 0; i < mIdxCount(0); i++)
    {
        mGridMap[i] = new bool*[mIdxCount(1)];
        for (int j = 0; j < mIdxCount(1); j++)
        {
            mGridMap[i][j] = new bool[mIdxCount(2)];
            memset(mGridMap[i][j], true, mIdxCount(2) * sizeof(bool));
        }
    }
    mExistMap = true;
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

    mIdxCount = ((mUpperBound - mLowerBound) / mResolution).cast<int>() + Eigen::Vector3i::Ones();

    mGridMap = new bool**[mIdxCount(0)];
    for (int i = 0; i < mIdxCount(0); i++)
    {
        mGridMap[i] = new bool*[mIdxCount(1)];
        for (int j = 0; j < mIdxCount(1); j++)
        {
            mGridMap[i][j] = new bool[mIdxCount(2)];
            std::memset(mGridMap[i][j], true, mIdxCount(2) * sizeof(bool));
        }
    }
    mExistMap = true;
}

bool World::collisionFree(const Node::ConstPtr& node_start, const Node::ConstPtr& node_end)
{
    Eigen::Vector3d e_z, e_y, e_x;
    Eigen::Matrix3d rotation_matrix;

    Eigen::Vector3d diff_pos = node_end->mPosition - node_start->mPosition;
    Eigen::Vector3d diff_norm_vector = node_end->mPlane->normal_vector - node_start->mPlane->normal_vector;

    size_t step = 20;
    bool is_free = true;

    for (size_t i = 0; i <= step; i++)
    {
        Eigen::Vector3d check_center = node_start->mPosition + diff_pos * i / (double)step;
        e_z = node_start->mPlane->normal_vector + diff_norm_vector * i / (double)step;
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

// void World::setObstacle(const Eigen::Vector3d& point)
// {
//     Eigen::Vector3i idx = coord2index(point);
//     mGridMap[idx(0)][idx(1)][idx(2)] = false;
// }

// bool World::isFree(const Eigen::Vector3d& point)
// {
//     Eigen::Vector3i idx = coord2index(point);
//     bool is_free = isInsideBorder(idx) && mGridMap[idx(0)][idx(1)][idx(2)];
//     return is_free;
// }

// Eigen::Vector3d World::coordRounding(const Eigen::Vector3d& coord)
// {
//     return index2coord(coord2index(coord));
// }

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

// bool World::isInsideBorder(const Eigen::Vector3i& index)
// {
//     return index(0) >= 0 && index(1) >= 0 && index(2) >= 0 && index(0) < mIdxCount(0) && index(1) < mIdxCount(1) &&
//            index(2) < mIdxCount(2);
// }
}   // namespace putn
