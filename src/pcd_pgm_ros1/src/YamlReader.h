//
// Created by xiaofan on 24-11-25.
//

#ifndef YAML_READER_H
#define YAML_READER_H

#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <string>
#include <iostream>
#include <sstream>
#include <memory>

class YamlReader {
public:
    explicit YamlReader(std::string filePath);

    bool load();

    template <typename T>
    bool read(const std::string &key, T &value);

    template <typename Derived>
    bool readEigen(const std::string &key, Eigen::MatrixBase<Derived> &matrix);

private:
    template <typename T>
    bool getValueByKey(const std::string &key, T &value);

    std::shared_ptr<YAML::Node> rootNode_;
    std::string filePath_;
    char delimiter_ = '.';  // use '.' to seperate the key in yaml
};


#endif // YAML_READER_H
