
#ifndef INCLUDE_COLLECTOR_HPP_
#define INCLUDE_COLLECTOR_HPP_

class Collector {
 public:

  Collector();

  ~Collector();

  bool collector();
  std::vector<std::pair<double,double>> readTextFile();
};

#endif 
