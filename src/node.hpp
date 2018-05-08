#ifndef NODE_HPP
#define NODE_HPP

#include <uavcan/uavcan.hpp>
#include <ch.hpp>

namespace Node {

  constexpr unsigned NodePoolSize = 2048;
  uavcan::Node<NodePoolSize>& getNode();
  //void publishKeyValue(const char *key, float value);

  extern uint8_t controling_node_id;

  bool isControlingNodeAlive();

  class uavcanNodeThread : public chibios_rt::BaseStaticThread<4000> {
    public:
      void main();
  };

}

#endif

