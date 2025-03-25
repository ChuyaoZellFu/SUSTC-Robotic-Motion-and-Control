// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Base_GpioConfiguration = require('./Base_GpioConfiguration.js');

//-----------------------------------------------------------

class GpioConfigurationList {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.port_configurations = null;
    }
    else {
      if (initObj.hasOwnProperty('port_configurations')) {
        this.port_configurations = initObj.port_configurations
      }
      else {
        this.port_configurations = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GpioConfigurationList
    // Serialize message field [port_configurations]
    // Serialize the length for message field [port_configurations]
    bufferOffset = _serializer.uint32(obj.port_configurations.length, buffer, bufferOffset);
    obj.port_configurations.forEach((val) => {
      bufferOffset = Base_GpioConfiguration.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GpioConfigurationList
    let len;
    let data = new GpioConfigurationList(null);
    // Deserialize message field [port_configurations]
    // Deserialize array length for message field [port_configurations]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.port_configurations = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.port_configurations[i] = Base_GpioConfiguration.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.port_configurations.forEach((val) => {
      length += Base_GpioConfiguration.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/GpioConfigurationList';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2692838fc0bc85259f7645a61387ad92';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    Base_GpioConfiguration[] port_configurations
    ================================================================================
    MSG: kortex_driver/Base_GpioConfiguration
    
    uint32 port_number
    GpioPinConfiguration[] pin_configurations
    ================================================================================
    MSG: kortex_driver/GpioPinConfiguration
    
    uint32 pin_id
    uint32 pin_property
    bool output_enable
    bool default_output_value
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GpioConfigurationList(null);
    if (msg.port_configurations !== undefined) {
      resolved.port_configurations = new Array(msg.port_configurations.length);
      for (let i = 0; i < resolved.port_configurations.length; ++i) {
        resolved.port_configurations[i] = Base_GpioConfiguration.Resolve(msg.port_configurations[i]);
      }
    }
    else {
      resolved.port_configurations = []
    }

    return resolved;
    }
};

module.exports = GpioConfigurationList;
