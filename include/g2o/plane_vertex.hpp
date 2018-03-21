//
// Created by hr on 18-3-20.
//

#ifndef PROJECT_VERTEX_PLANE_HPP
#define PROJECT_VERTEX_PLANE_HPP

#include <g2o/stuff/macros.h>
#include <g2o/core/factory.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/linear_solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/solvers/pcg/linear_solver_pcg.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d/edge_se3_pointxyz.h>

#include <g2o/core/base_vertex.h>
#include <g2o/core/hyper_graph_action.h>

#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d_addons/types_slam3d_addons.h>
#include <g2o/types/slam3d_addons/g2o_types_slam3d_addons_api.h>
#include <g2o/types/slam3d_addons/plane3d.h>

#include <g2o/config.h>

namespace g2o
{

    class  PlaneVertex : public BaseVertex<3, Plane3D>
{
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    PlaneVertex(){
        color << cst(.2), cst(.2), cst(.2);
    }

    virtual bool read(std::istream& is) {
        Vector4 lv;
        for (int i=0; i<4; i++)
            is >> lv[i];
        setEstimate(Plane3D(lv));
        is >> color(0) >> color(1) >> color(2);
        return true;
    }

    virtual bool write(std::ostream& os) const {
        Vector4 lv=_estimate.toVector();
        for (int i=0; i<4; i++){
            os << lv[i] << " ";
        }
        os << color(0) << " " << color(1) << " " << color(2) << " ";
        return os.good();
    }

    virtual void setToOriginImpl() { _estimate = Plane3D(); }

    virtual void oplusImpl(const number_t* update_) {
        Eigen::Map<const Vector3> update(update_);
        _estimate.oplus(update);
    }

    virtual bool setEstimateDataImpl(const number_t* est){
        Eigen::Map<const Vector4> _est(est);
        _estimate.fromVector(_est);
        return true;
    }

    virtual bool getEstimateData(number_t* est) const{
        Eigen::Map<Vector4> _est(est);
        _est = _estimate.toVector();
        return true;
    }

    virtual int estimateDimension() const {
        return 4;
    }

    Vector3 color;
};

}

#endif //PROJECT_VERTEX_PLANE_HPP




