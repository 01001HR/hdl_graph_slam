//
// Created by hr on 18-3-20.
//


#include <g2o/stuff/macros.h>
#include <g2o/core/factory.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/linear_solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/solvers/pcg/linear_solver_pcg.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d/edge_se3_pointxyz.h>
#include <g2o/types/slam3d_addons/types_slam3d_addons.h>

#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d_addons/types_slam3d_addons.h>
#include <g2o/core/sparse_optimizer.h>

// g2o example --- main.cpp
// Ross Kidson 19-1-14
//

#include <Eigen/Core>

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/edge_se3_priorxyz.hpp>
#include <g2o/edge_se3_plane.hpp>
#include <g2o/edge_se3_priorxy.hpp>
#include <g2o/plane_vertex.hpp>
G2O_USE_OPTIMIZATION_LIBRARY(csparse)

using namespace g2o;

// Landmark generation parameters
const int NUM_POSES = 20;
const int NUM_CONNECTIONS_PER_VERTEX = 4;

// noise to be added to initial estimates
const double POSE_NOISE = 10.0;

// noise to be added to relative pose edge measurement
const double EDGE_NOISE = 0.05;

// should be done as singleton
class UniqueId
{
public:
    UniqueId():unique_id(0){}
    int getUniqueId()
    {
        return unique_id++;
    }
private:
    int unique_id;
};
static UniqueId uniqueId;

double fRand(double fMin, double fMax)
{
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}

void addNoiseToTranslation(Eigen::Isometry3d & trafo, const double noise)
{
    trafo.translation()[0] += fRand(-noise, noise);
    trafo.translation()[1] += fRand(-noise, noise);
    trafo.translation()[2] += fRand(-noise, noise);
}

Eigen::Isometry3d isometryFromArray7D(const double* v)
{
    Eigen::Isometry3d result;

    result = Eigen::Quaterniond(v[6], v[3], v[4], v[5]).normalized().toRotationMatrix();
    result.translation() = Eigen::Vector3d(v[0], v[1], v[2]);
    return result;
}

void generatePoses(std::vector<Eigen::Isometry3d>& pose_list)
{
    const double step_arr [7] = {0.0, 0.0, 10.0, 0.0, -0.05, 0.0, 1.0};
    const double initial_arr [7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    Eigen::Isometry3d step = isometryFromArray7D(step_arr);
    pose_list.front() = isometryFromArray7D(initial_arr);
    for(std::vector<Eigen::Isometry3d>::iterator itr = pose_list.begin() + 1; itr != pose_list.end(); itr++)
    {
        Eigen::Isometry3d & cur_trafo = *itr;
        Eigen::Isometry3d & prev_trafo = *(itr-1);
        cur_trafo = prev_trafo * step;
    }
}

void addPosesToGraph(std::vector<Eigen::Isometry3d>& pose_list, std::vector<int>& poseNum2Id, std::unique_ptr<g2o::SparseOptimizer>& graph_ptr)
{
    for(std::vector<Eigen::Isometry3d>::const_iterator itr = pose_list.begin(); itr != pose_list.end(); itr++)
    {
        const Eigen::Isometry3d & pose = *itr;
        VertexSE3 * v = new VertexSE3();

        // add some noise to translation component
        Eigen::Isometry3d pose_estimate(pose);
        addNoiseToTranslation(pose_estimate, POSE_NOISE);

        // get id
        const int id = uniqueId.getUniqueId();
        poseNum2Id.push_back(id);

        // populate g2o vertex object and add to graph
        v->setEstimate(pose_estimate);
        v->setId(id);
        v->setFixed(false);
        graph_ptr->addVertex(v);
    }
}

g2o::PlaneVertex* add_plane_node(const Eigen::Vector4d& plane_coeffs,
                                      std::unique_ptr<g2o::SparseOptimizer>& graph_ptr)
{
    g2o::PlaneVertex* vertex =new g2o::PlaneVertex();
    vertex->setId(uniqueId.getUniqueId());
    vertex->setEstimate(plane_coeffs);
    vertex->setFixed(true);
    graph_ptr->addVertex(vertex);
    return vertex;
}


void addEdge(const Eigen::Isometry3d & a_T_b, const int id_a, const int id_b,
             std::unique_ptr<g2o::SparseOptimizer>& graph_ptr)
{
    EdgeSE3 * e = new EdgeSE3;

    // retrieve vertex pointers from graph with id's
    g2o::OptimizableGraph::Vertex * pose_a_vertex
            = dynamic_cast<g2o::OptimizableGraph::Vertex*>
            (graph_ptr->vertices()[id_a]);

    g2o::OptimizableGraph::Vertex * pose_b_vertex
            = dynamic_cast<g2o::OptimizableGraph::Vertex*>
            (graph_ptr->vertices()[id_b]);

    // error check vertices and associate them with the edge
    assert(pose_a_vertex!=NULL);
    assert(pose_a_vertex->dimension() == 6);
    e->vertices()[0] = pose_a_vertex;

    assert(pose_b_vertex!=NULL);
    assert(pose_b_vertex->dimension() == 6);
    e->vertices()[1] = pose_b_vertex;

    // add information matrix
    Eigen::Matrix<double, 6, 6> Lambda;
    Lambda.setIdentity();

    // set the observation and imformation matrix
    e->setMeasurement(a_T_b);
    e->information() = Lambda;

    // finally add the edge to the graph
    if(!graph_ptr->addEdge(e))
    {
        assert(false);
    }
}

void addEdgePriorxyz(const Eigen::Vector3d & a_position, const int id_a, std::unique_ptr<g2o::SparseOptimizer>& graph_ptr)
{
    EdgeSE3PriorXYZ * e = new EdgeSE3PriorXYZ;

    // retrieve vertex pointers from graph with id's
    g2o::OptimizableGraph::Vertex * pose_a_vertex
            = dynamic_cast<g2o::OptimizableGraph::Vertex*>
            (graph_ptr->vertices()[id_a]);

    // error check vertices and associate them with the edge
    assert(pose_a_vertex!=NULL);
    assert(pose_a_vertex->dimension() == 6);
    e->vertices()[0] = pose_a_vertex;

    // set the observation and imformation matrix
    e->setMeasurement(a_position);

    // add information matrix
    Eigen::Matrix<double, 3, 3> Lambda;
    Lambda.setIdentity();
    e->information() = Lambda;

    // finally add the edge to the graph
    if(!graph_ptr->addEdge(e))
    {
        assert(false);
    }
}

void addEdgePriorxy(const Eigen::Vector2d & a_position, const int id_a, std::unique_ptr<g2o::SparseOptimizer>& graph_ptr)
{
    EdgeSE3PriorXY * e = new EdgeSE3PriorXY;

    // retrieve vertex pointers from graph with id's
    g2o::OptimizableGraph::Vertex * pose_a_vertex
            = dynamic_cast<g2o::OptimizableGraph::Vertex*>
            (graph_ptr->vertices()[id_a]);

    // error check vertices and associate them with the edge
    assert(pose_a_vertex!=NULL);
    assert(pose_a_vertex->dimension() == 6);
    e->vertices()[0] = pose_a_vertex;

    // set the observation and imformation matrix
    e->setMeasurement(a_position);

    // add information matrix
    Eigen::Matrix<double, 2, 2> Lambda;
    Lambda.setIdentity();
    e->information() = Lambda;

    // finally add the edge to the graph
    if(!graph_ptr->addEdge(e))
    {
        assert(false);
    }
}

int main(int argc, char** argv)
{
    ////////////////////////////////////////////////////////////////////////////////////////////////////
    // initialize graph

//    g2o::SparseOptimizer graph;
    std::unique_ptr<g2o::SparseOptimizer> graph;  // g2o graph

    graph.reset(new g2o::SparseOptimizer());

    std::string g2o_solver_name = "lm_var";
    std::cout << "construct solver... " << std::endl;
    g2o::OptimizationAlgorithmFactory* solver_factory = g2o::OptimizationAlgorithmFactory::instance();
    g2o::OptimizationAlgorithmProperty solver_property;
    g2o::OptimizationAlgorithm* solver = solver_factory->construct(g2o_solver_name, solver_property);
    graph->setAlgorithm(solver);

    if (!graph->solver()) {
        std::cerr << std::endl;
        std::cerr << "error : failed to allocate solver!!" << std::endl;
        solver_factory->listSolvers(std::cerr);
        std::cerr << "-------------" << std::endl;
        std::cin.ignore(1);
        return -1;
    }
    std::cout << "done" << std::endl;

    ////////////////////////////////////////////////////////////////////////////////////////////////////
    // poses
    std::cout << "Generating Poses...\n";

    std::vector<Eigen::Isometry3d> pose_list(NUM_POSES);
    generatePoses(pose_list);

    std::vector<int> poseNum2Id;  //used to associate pose number to g2o vertex id
    addPosesToGraph(pose_list, poseNum2Id, graph);

    ////////////////////////////////////////////////////////////////////////////////////////////////////
    //Edges
    std::cout << "Generating edges...\n";

    // iterator over poses
    for(std::vector<Eigen::Isometry3d>::const_iterator itr = pose_list.begin(); itr != pose_list.end(); itr++)
    {
        const Eigen::Isometry3d & w_T_a = *itr;
        for(int i = 1; i <= NUM_CONNECTIONS_PER_VERTEX; i++)
        {
            std::vector<Eigen::Isometry3d>::const_iterator itr_other = itr + i;
            if(itr_other < pose_list.end())
            {
                const int pose_a_id = poseNum2Id[itr - pose_list.begin()];
                const int pose_b_id = poseNum2Id[itr_other - pose_list.begin()];
                const Eigen::Isometry3d & w_T_b = *itr_other;

                Eigen::Isometry3d a_T_b = w_T_a.inverse() * w_T_b;
                addNoiseToTranslation(a_T_b, EDGE_NOISE);
                addEdge(a_T_b, pose_a_id, pose_b_id, graph);
            }
        }

        const int pose_a_id = poseNum2Id[itr - pose_list.begin()];

        Eigen::Isometry3d pose_a = *itr;
        addNoiseToTranslation(pose_a, EDGE_NOISE);
        addEdgePriorxyz(pose_a.translation(), pose_a_id, graph);
        addEdgePriorxy(pose_a.translation().head<2>(), pose_a_id, graph);

    }

    ////////////

    g2o::PlaneVertex* floor_plane_node = add_plane_node(Eigen::Vector4d(0.0, 0.0, 1.0, 0.0), graph);           // ground floor plane node


    std::cout << "initializeOptimization...\n";

    ////////////////////////////////////////////////////////////////////////////////////////////////////
    // finished populating graph, export and quit
    graph->initializeOptimization();
    std::cout << "Saving Graph to example_g2o.g2o...\n";
    graph->save("example_g2o.g2o");
    std::cout << "Finished\n";
    std::cout << "rosrun g2o_viewer g2o_viewer example_g2o.g2o\n";
}