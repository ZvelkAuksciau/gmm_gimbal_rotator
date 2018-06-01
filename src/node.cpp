#include <node.hpp>
#include <uavcan/uavcan.hpp>
#include <uavcan_stm32/uavcan_stm32.hpp>

#include <uavcan/protocol/node_status_monitor.hpp>

//#include <uavcan/protocol/debug/KeyValue.hpp>

//#include <uavcan/protocol/param_server.hpp>
//#include <uavcan/protocol/restart_request_server.hpp>
//#include <config/config.hpp>

#include <ch.hpp>

namespace Node {

//  os::config::Param<uint8_t> node_id("node.id", 1, 1, 127); //0 - automatic detection (not supported)
//  os::config::Param<uint32_t> bus_speed("node.speed", 1000000, 125000, 1000000);

    uint8_t controling_node_id = 0;
    bool controling_node_alive = true;

  uavcan_stm32::CanInitHelper<> can;

  uavcan::Node<NodePoolSize>& getNode() {
    static uavcan::Node<NodePoolSize> node(can.driver, uavcan_stm32::SystemClock::instance());
    return node;
  }

  bool isControlingNodeAlive() {
      return controling_node_alive;
  }

  class NodeMonitor: public uavcan::NodeStatusMonitor {
    /**
     * This method is not required to implement.
     * It is called when a remote node becomes online, changes status, or goes offline.
     */
    void handleNodeStatusChange(const NodeStatusChangeEvent& event) override
    {
        if (event.was_known) {
            if(event.status.mode == uavcan::protocol::NodeStatus::MODE_OFFLINE && event.node_id == controling_node_id) {
                controling_node_alive = false;
            }
        }
    }

    /**
     * This method is not required to implement.
     * It is called for every received message uavcan.protocol.NodeStatus after handleNodeStatusChange(), even
     * if the status code has not changed.
     */
    void handleNodeStatusMessage(
            const uavcan::ReceivedDataStructure<uavcan::protocol::NodeStatus>& msg)
                    override
                    {
        (void) msg;
        if(msg.getSrcNodeID().get() == controling_node_id && msg.mode != msg.MODE_OFFLINE) {
            controling_node_alive = true;
        }
        //std::cout << "Remote node status message\n" << msg << std::endl << std::endl;
    }

public:
    NodeMonitor(uavcan::INode& node) :
            uavcan::NodeStatusMonitor(node) {
    }

//    static const char* modeToString(const NodeStatus status) {
//        switch (status.mode) {
//        case uavcan::protocol::NodeStatus::MODE_OPERATIONAL:
//            return "OPERATIONAL";
//        case uavcan::protocol::NodeStatus::MODE_INITIALIZATION:
//            return "INITIALIZATION";
//        case uavcan::protocol::NodeStatus::MODE_MAINTENANCE:
//            return "MAINTENANCE";
//        case uavcan::protocol::NodeStatus::MODE_SOFTWARE_UPDATE:
//            return "SOFTWARE_UPDATE";
//        case uavcan::protocol::NodeStatus::MODE_OFFLINE:
//            return "OFFLINE";
//        default:
//            return "???";
//        }
//    }


//    static const char* healthToString(const NodeStatus status) {
//        switch (status.health) {
//        case uavcan::protocol::NodeStatus::HEALTH_OK:
//            return "OK";
//        case uavcan::protocol::NodeStatus::HEALTH_WARNING:
//            return "WARNING";
//        case uavcan::protocol::NodeStatus::HEALTH_ERROR:
//            return "ERROR";
//        case uavcan::protocol::NodeStatus::HEALTH_CRITICAL:
//            return "CRITICAL";
//        default:
//            return "???";
//        }
//    }

};

  /*
   * Param access server
   */
///*  class ParamManager : public uavcan::IParamManager
//  {
//      void convert(float native_value, ConfigDataType native_type, Value& out_value) const
//      {
//          if (native_type == CONFIG_TYPE_BOOL)
//          {
//              out_value.to<Value::Tag::boolean_value>() = !uavcan::isCloseToZero(native_value);
//          }
//          else if (native_type == CONFIG_TYPE_INT)
//          {
//              out_value.to<Value::Tag::integer_value>() = static_cast<std::int64_t>(native_value);
//          }
//          else if (native_type == CONFIG_TYPE_FLOAT)
//          {
//              out_value.to<Value::Tag::real_value>() = native_value;
//          }
//          else
//          {
//              ; // Invalid type
//          }
//      }
//
//      void convert(float native_value, ConfigDataType native_type, NumericValue& out_value) const
//      {
//          if (native_type == CONFIG_TYPE_INT)
//          {
//              out_value.to<NumericValue::Tag::integer_value>() = static_cast<std::int64_t>(native_value);
//          }
//          else if (native_type == CONFIG_TYPE_FLOAT)
//          {
//              out_value.to<NumericValue::Tag::real_value>() = native_value;
//          }
//          else
//          {
//              ; // Not applicable
//          }
//      }
//
//      void getParamNameByIndex(Index index, Name& out_name) const override
//      {
//          const char* name = configNameByIndex(index);
//          if (name != nullptr)
//          {
//              out_name = name;
//          }
//      }
//
//      void assignParamValue(const Name& name, const Value& value) override
//      {
//          float native_value = 0.F;
//
//          if (value.is(Value::Tag::boolean_value))
//          {
//              native_value = (*value.as<Value::Tag::boolean_value>()) ? 1.F : 0.F;
//          }
//          else if (value.is(Value::Tag::integer_value))
//          {
//              native_value = static_cast<float>(*value.as<Value::Tag::integer_value>());
//          }
//          else if (value.is(Value::Tag::real_value))
//          {
//              native_value = *value.as<Value::Tag::real_value>();
//          }
//          else
//          {
//              return;
//          }
//
//          (void)configSet(name.c_str(), native_value);
//      }
//
//      void readParamValue(const Name& name, Value& out_value) const override
//      {
//          ConfigParam descr;
//          const int res = configGetDescr(name.c_str(), &descr);
//          if (res >= 0)
//          {
//              convert(configGet(name.c_str()), descr.type, out_value);
//          }
//      }
//
//      void readParamDefaultMaxMin(const Name& name, Value& out_default,
//                                  NumericValue& out_max, NumericValue& out_min) const override
//      {
//          ConfigParam descr;
//          const int res = configGetDescr(name.c_str(), &descr);
//          if (res >= 0)
//          {
//              convert(descr.default_, descr.type, out_default);
//              convert(descr.max, descr.type, out_max);
//              convert(descr.min, descr.type, out_min);
//          }
//      }
//
//      int saveAllParams() override
//      {
//          return configSave();
//      }
//
//      int eraseAllParams() override
//      {
//          return configErase();
//      }
//  } param_manager;*/

/*  class RestartRequestHandler: public uavcan::IRestartRequestHandler {
    bool handleRestartRequest(uavcan::NodeID request_source) override
    {
      (void) request_source;
      NVIC_SystemReset();
      return true;
    }
  } restart_request_handler;*/

//  static uavcan::Publisher<uavcan::protocol::debug::KeyValue> kv_pub(getNode());

//  void publishKeyValue(const char *key, float value) {
//      uavcan::protocol::debug::KeyValue kv_msg;
//      kv_msg.key = key;
//      kv_msg.value = value;
//      kv_pub.broadcast(kv_msg);
//  }

  void uavcanNodeThread::main() {
    controling_node_id = 0;

    uavcan::uint32_t bitrate = 1000000;
    can.init(bitrate);

/*
    if(node_id.get() > 0) {
        getNode().setNodeID(node_id.get());
    }
*/
    getNode().setName("org.kmti.gimbal_rot");
    getNode().setNodeID(40);

    if (getNode().start() < 0) {
      chSysHalt("UAVCAN init fail");
    }
  //  kv_pub.init();

//    getNode().setRestartRequestHandler(&restart_request_handler);

//    uavcan::ParamServer server(getNode());
//    server.start(&param_manager);

    NodeMonitor monitor(getNode());
    monitor.start();

    getNode().setModeOperational();

    while(true) {
      if (getNode().spin(uavcan::MonotonicDuration::fromMSec(500)) < 0) {
        chSysHalt("UAVCAN spin fail");
      }
        //kv_pub.broadcast(kv_msg);
    }
  }

}

