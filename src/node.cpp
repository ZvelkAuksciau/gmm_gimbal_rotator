#include <node.hpp>
#include <uavcan/uavcan.hpp>
#include <uavcan_stm32/uavcan_stm32.hpp>
#include <uavcan/protocol/debug/KeyValue.hpp>

#include <uavcan/protocol/param_server.hpp>
#include <uavcan/protocol/restart_request_server.hpp>
#include <config/config.hpp>

#include <ch.hpp>

namespace Node {

  os::config::Param<uint8_t> node_id("node.id", 1, 1, 127); //0 - automatic detection (not supported)
  os::config::Param<uint32_t> bus_speed("node.speed", 1000000, 125000, 1000000);

  uavcan_stm32::CanInitHelper<> can;

  uavcan::Node<NodePoolSize>& getNode() {
    static uavcan::Node<NodePoolSize> node(can.driver, uavcan_stm32::SystemClock::instance());
    return node;
  }

  /*
   * Param access server
   */
  class ParamManager : public uavcan::IParamManager
  {
      void convert(float native_value, ConfigDataType native_type, Value& out_value) const
      {
          if (native_type == CONFIG_TYPE_BOOL)
          {
              out_value.to<Value::Tag::boolean_value>() = !uavcan::isCloseToZero(native_value);
          }
          else if (native_type == CONFIG_TYPE_INT)
          {
              out_value.to<Value::Tag::integer_value>() = static_cast<std::int64_t>(native_value);
          }
          else if (native_type == CONFIG_TYPE_FLOAT)
          {
              out_value.to<Value::Tag::real_value>() = native_value;
          }
          else
          {
              ; // Invalid type
          }
      }

      void convert(float native_value, ConfigDataType native_type, NumericValue& out_value) const
      {
          if (native_type == CONFIG_TYPE_INT)
          {
              out_value.to<NumericValue::Tag::integer_value>() = static_cast<std::int64_t>(native_value);
          }
          else if (native_type == CONFIG_TYPE_FLOAT)
          {
              out_value.to<NumericValue::Tag::real_value>() = native_value;
          }
          else
          {
              ; // Not applicable
          }
      }

      void getParamNameByIndex(Index index, Name& out_name) const override
      {
          const char* name = configNameByIndex(index);
          if (name != nullptr)
          {
              out_name = name;
          }
      }

      void assignParamValue(const Name& name, const Value& value) override
      {
          float native_value = 0.F;

          if (value.is(Value::Tag::boolean_value))
          {
              native_value = (*value.as<Value::Tag::boolean_value>()) ? 1.F : 0.F;
          }
          else if (value.is(Value::Tag::integer_value))
          {
              native_value = static_cast<float>(*value.as<Value::Tag::integer_value>());
          }
          else if (value.is(Value::Tag::real_value))
          {
              native_value = *value.as<Value::Tag::real_value>();
          }
          else
          {
              return;
          }

          (void)configSet(name.c_str(), native_value);
      }

      void readParamValue(const Name& name, Value& out_value) const override
      {
          ConfigParam descr;
          const int res = configGetDescr(name.c_str(), &descr);
          if (res >= 0)
          {
              convert(configGet(name.c_str()), descr.type, out_value);
          }
      }

      void readParamDefaultMaxMin(const Name& name, Value& out_default,
                                  NumericValue& out_max, NumericValue& out_min) const override
      {
          ConfigParam descr;
          const int res = configGetDescr(name.c_str(), &descr);
          if (res >= 0)
          {
              convert(descr.default_, descr.type, out_default);
              convert(descr.max, descr.type, out_max);
              convert(descr.min, descr.type, out_min);
          }
      }

      int saveAllParams() override
      {
          return configSave();
      }

      int eraseAllParams() override
      {
          return configErase();
      }
  } param_manager;

  class RestartRequestHandler: public uavcan::IRestartRequestHandler {
    bool handleRestartRequest(uavcan::NodeID request_source) override
    {
      (void) request_source;
      NVIC_SystemReset();
      return true;
    }
  } restart_request_handler;

  static uavcan::Publisher<uavcan::protocol::debug::KeyValue> kv_pub(getNode());

  void publishKeyValue(const char *key, float value) {
      uavcan::protocol::debug::KeyValue kv_msg;
      kv_msg.key = key;
      kv_msg.value = value;
      kv_pub.broadcast(kv_msg);
  }

  void uavcanNodeThread::main() {
    can.init(bus_speed.get());

    getNode().setName("org.kmti.gmm_controler");
    if(node_id.get() > 0) {
        getNode().setNodeID(node_id.get());
    }
    getNode().setName("org.kmti.gimbal_rot");
    getNode().setNodeID(10);

    if (getNode().start() < 0) {
      chSysHalt("UAVCAN init fail");
    }
    kv_pub.init();

    getNode().setRestartRequestHandler(&restart_request_handler);

    uavcan::ParamServer server(getNode());
    server.start(&param_manager);

    getNode().setModeOperational();

    while(true) {
      if (getNode().spin(uavcan::MonotonicDuration::fromMSec(500)) < 0) {
        chSysHalt("UAVCAN spin fail");
      }
        //kv_pub.broadcast(kv_msg);
    }
  }

}

