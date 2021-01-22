#include "osrm/match_parameters.hpp"
#include "osrm/nearest_parameters.hpp"
#include "osrm/route_parameters.hpp"
#include "osrm/table_parameters.hpp"
#include "osrm/trip_parameters.hpp"

#include "osrm/coordinate.hpp"
#include "osrm/engine_config.hpp"
#include "osrm/json_container.hpp"
#include "osrm/osrm.hpp"
#include "osrm/status.hpp"
#include "osrm/engine/api/base_result.hpp"
#include "mapbox/variant.hpp"

#include <exception>
#include <Eigen/Dense>
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

using namespace osrm;

namespace py = pybind11;
using RowMatrixXd = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;


struct EigenRenderer
{
    explicit EigenRenderer(Eigen::Ref<RowMatrixXd> &_out, int i, int j) : out(_out), i(i), j(j) {}
    void operator()(const json::Number &number) const {out(i,j) = number.value;}
    void operator()(const json::Null &) const {out(i,j) = 120.0*60.0;}
    void operator()(const json::Object &object) const {}
    void operator()(const json::Array &array) const {}
    void operator()(const json::True &) const {}
    void operator()(const json::False &) const {}
    void operator()(const json::String &string) const {}

private:
    Eigen::Ref<RowMatrixXd> &out;
    int i, j;
};


class PyOSRM {
private:
    EngineConfig config;
    OSRM osrm;
    RouteParameters params;

    void SetupConfig(const char *path) {
        config.storage_config = {path};
        config.use_shared_memory = false;
        config.algorithm = EngineConfig::Algorithm::CH;
    }

public:
    PyOSRM() : config{}, osrm((SetupConfig("/osrm-backend/great-britain-latest.osrm"), config)) {};

    void table(
            py::list lons,
            py::list lats,
            py::list sources,
            py::list dests,
            Eigen::Ref<RowMatrixXd> dest
    ) {
        TableParameters params;

        for (int coord_i = 0; coord_i< lons.size(); coord_i++){
            params.coordinates.push_back({util::FloatLongitude{lons[coord_i].cast<float>()},
                                          util::FloatLatitude{lats[coord_i].cast<float>()}
            });
        }
        for (auto py_source : sources){
            params.sources.push_back(py_source.cast<int>());
        }
        for (auto py_dest : dests){
            params.destinations.push_back(py_dest.cast<int>());
        }
        engine::api::ResultT result = json::Object();
        py::gil_scoped_release release;
        const auto status = osrm.Table(params, result);
        py::gil_scoped_acquire acquire;
        if (status == Status::Ok){
            auto &json_result = result.get<json::Object>();
            const auto duration = json_result.values["durations"].get<json::Array>();

            // assign into dest
            for(int i = 0; i < sources.size(); ++i){
                const auto source_array = duration.values[i].get<json::Array>();
                for(int j = 0; j < dests.size(); ++j) {
                    EigenRenderer renderer(dest, i, j);
                    mapbox::util::apply_visitor(renderer, source_array.values[j]);
                }
            }
        }
    };
};

PYBIND11_MODULE(PyOSRM, m) {
    py::class_<PyOSRM>(m, "PyOSRM")
            .def(py::init<>())
            .def("table", &PyOSRM::table);
}
