#include "config.hpp"
#include <fstream>
#include <string>
#include "nlohmann/json.hpp"
#include <iostream>
#include "types.hpp"


lookupconfig lookup_cfg{};
CameraIntrinsic camera_intr_cfg{};
ReaderConfig reader_cfg{};
ProcessorConfig processor_cfg{};


nlohmann::json load_config_json(){
    std::ifstream f("../../config/config.json");
    return nlohmann::json::parse(f);;
}


ConfigInitStatus config_init(){
    ConfigInitStatus status = ConfigInitStatus::Error;
    nlohmann::json config;
    try
    {
        config = load_config_json();
    }
    catch (const std::exception& e)
    {
        std::cerr
            << "Config load error : "
            << e.what() << "\nFalling back to internal config !!"
            << std::endl;
        status = ConfigInitStatus::FallBack;
    }
    catch (...)
    {
        std::cerr
            << "Unknown error occurred while loading config."
            << "\nFalling back to internal config !!"
            << std::endl;
        status = ConfigInitStatus::FallBack;
    }

    if (!config.is_null())
    {
        // -------------------------------------------------
        // lookup_cfg
        // -------------------------------------------------

        lookup_cfg.binpath =
            config["lookup_cfg"]["binpath"]
                .get<std::string>();

        lookup_cfg.lookup_precision =
            config["lookup_cfg"]["lookup_precision"]
                .get<double>();

        lookup_cfg.tolerance =
            config["lookup_cfg"]["tolerance"]
                .get<int>();


        // -------------------------------------------------
        // camera_intr_cfg
        // -------------------------------------------------

        camera_intr_cfg.fx =
            config["camera_intr_cfg"]["fx"]
                .get<double>();

        camera_intr_cfg.fy =
            config["camera_intr_cfg"]["fy"]
                .get<double>();

        camera_intr_cfg.cx =
            config["camera_intr_cfg"]["cx"]
                .get<double>();

        camera_intr_cfg.cy =
            config["camera_intr_cfg"]["cy"]
                .get<double>();


        // -------------------------------------------------
        // reader_cfg
        // -------------------------------------------------

        reader_cfg.source =
            config["reader_cfg"]["source"]
                .get<std::string>();

        reader_cfg.file_path =
            config["reader_cfg"]["file_path"]
                .get<std::string>();

        reader_cfg.expected_width =
            config["reader_cfg"]["expected_width"]
                .get<int>();

        reader_cfg.expected_height =
            config["reader_cfg"]["expected_height"]
                .get<int>();

        reader_cfg.frequency =
            config["reader_cfg"]["frequency"]
                .get<double>();

        reader_cfg.period =
            std::chrono::milliseconds(
                config["reader_cfg"]["period"]
                    .get<int>()
            );


        // -------------------------------------------------
        // processor_cfg
        // -------------------------------------------------

        processor_cfg.frequency =
            config["processor_cfg"]["frequency"]
                .get<double>();

        processor_cfg.period =
            std::chrono::milliseconds(
                config["processor_cfg"]["period"]
                    .get<int>()
            );


        // -------------------------------------------------
        // processor_cfg.cv
        // -------------------------------------------------

        processor_cfg.cv.threshold =
            config["processor_cfg"]["cv"]["threshold"]
                .get<int>();

        processor_cfg.cv.blur_ksize =
            config["processor_cfg"]["cv"]["blur_ksize"]
                .get<int>();

        processor_cfg.cv.blur_sigma =
            config["processor_cfg"]["cv"]["blur_sigma"]
                .get<double>();

        processor_cfg.cv.area_max =
            config["processor_cfg"]["cv"]["area_max"]
                .get<int>();

        processor_cfg.cv.window_debug =
            config["processor_cfg"]["cv"]["window_debug"]
                .get<bool>();


        status = ConfigInitStatus::Json;
    }

    return status;
}
