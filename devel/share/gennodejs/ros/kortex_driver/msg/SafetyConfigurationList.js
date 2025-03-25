// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let SafetyConfiguration = require('./SafetyConfiguration.js');

//-----------------------------------------------------------

class SafetyConfigurationList {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.configuration = null;
    }
    else {
      if (initObj.hasOwnProperty('configuration')) {
        this.configuration = initObj.configuration
      }
      else {
        this.configuration = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SafetyConfigurationList
    // Serialize message field [configuration]
    // Serialize the length for message field [configuration]
    bufferOffset = _serializer.uint32(obj.configuration.length, buffer, bufferOffset);
    obj.configuration.forEach((val) => {
      bufferOffset = SafetyConfiguration.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SafetyConfigurationList
    let len;
    let data = new SafetyConfigurationList(null);
    // Deserialize message field [configuration]
    // Deserialize array length for message field [configuration]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.configuration = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.configuration[i] = SafetyConfiguration.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 17 * object.configuration.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/SafetyConfigurationList';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '8178a2a82961f46646c77423c6510e4d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    SafetyConfiguration[] configuration
    ================================================================================
    MSG: kortex_driver/SafetyConfiguration
    
    SafetyHandle handle
    float32 error_threshold
    float32 warning_threshold
    SafetyEnable enable
    ================================================================================
    MSG: kortex_driver/SafetyHandle
    
    uint32 identifier
    ================================================================================
    MSG: kortex_driver/SafetyEnable
    
    SafetyHandle handle
    bool enable
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SafetyConfigurationList(null);
    if (msg.configuration !== undefined) {
      resolved.configuration = new Array(msg.configuration.length);
      for (let i = 0; i < resolved.configuration.length; ++i) {
        resolved.configuration[i] = SafetyConfiguration.Resolve(msg.configuration[i]);
      }
    }
    else {
      resolved.configuration = []
    }

    return resolved;
    }
};

module.exports = SafetyConfigurationList;
