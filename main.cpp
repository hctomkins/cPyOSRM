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

#include <exception>
#include <Eigen/Dense>
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

using namespace osrm;

namespace py = pybind11;
using RowMatrixXd = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;


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
        const auto status = osrm.Table(params, result);
        if (status == Status::Ok){
            auto &json_result = result.get<json::Object>();
            const auto duration = json_result.values["durations"].get<json::Array>();

            // assign into dest
            for(int i = 0; i < sources.size(); ++i){
                const auto source_array = duration.values[i].get<json::Array>();
                for(int j = 0; j < dests.size(); ++j) {
                    float dest_time = source_array.values[j].get<json::Number>().value;
                    dest(i,j) = dest_time;
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
