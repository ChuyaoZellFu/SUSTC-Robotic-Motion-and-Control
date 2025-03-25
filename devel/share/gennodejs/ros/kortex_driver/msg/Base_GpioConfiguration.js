// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let GpioPinConfiguration = require('./GpioPinConfiguration.js');

//-----------------------------------------------------------

class Base_GpioConfiguration {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.port_number = null;
      this.pin_configurations = null;
    }
    else {
      if (initObj.hasOwnProperty('port_number')) {
        this.port_number = initObj.port_number
      }
      else {
        this.port_number = 0;
      }
      if (initObj.hasOwnProperty('pin_configurations')) {
        this.pin_configurations = initObj.pin_configurations
      }
      else {
        this.pin_configurations = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Base_GpioConfiguration
    // Serialize message field [port_number]
    bufferOffset = _serializer.uint32(obj.port_number, buffer, bufferOffset);
    // Serialize message field [pin_configurations]
    // Serialize the length for message field [pin_configurations]
    bufferOffset = _serializer.uint32(obj.pin_configurations.length, buffer, bufferOffset);
    obj.pin_configurations.forEach((val) => {
      bufferOffset = GpioPinConfiguration.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Base_GpioConfiguration
    let len;
    let data = new Base_GpioConfiguration(null);
    // Deserialize message field [port_number]
    data.port_number = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [pin_configurations]
    // Deserialize array length for message field [pin_configurations]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.pin_configurations = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.pin_configurations[i] = GpioPinConfiguration.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 10 * object.pin_configurations.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/Base_GpioConfiguration';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '92f1de7cd4a1d8641179ab50f182b3f3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
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
    const resolved = new Base_GpioConfiguration(null);
    if (msg.port_number !== undefined) {
      resolved.port_number = msg.port_number;
    }
    else {
      resolved.port_number = 0
    }

    if (msg.pin_configurations !== undefined) {
      resolved.pin_configurations = new Array(msg.pin_configurations.length);
      for (let i = 0; i < resolved.pin_configurations.length; ++i) {
        resolved.pin_configurations[i] = GpioPinConfiguration.Resolve(msg.pin_configurations[i]);
      }
    }
    else {
      resolved.pin_configurations = []
    }

    return resolved;
    }
};

module.exports = Base_GpioConfiguration;
