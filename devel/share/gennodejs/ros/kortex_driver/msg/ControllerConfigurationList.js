// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let ControllerConfiguration = require('./ControllerConfiguration.js');

//-----------------------------------------------------------

class ControllerConfigurationList {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.controller_configurations = null;
    }
    else {
      if (initObj.hasOwnProperty('controller_configurations')) {
        this.controller_configurations = initObj.controller_configurations
      }
      else {
        this.controller_configurations = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ControllerConfigurationList
    // Serialize message field [controller_configurations]
    // Serialize the length for message field [controller_configurations]
    bufferOffset = _serializer.uint32(obj.controller_configurations.length, buffer, bufferOffset);
    obj.controller_configurations.forEach((val) => {
      bufferOffset = ControllerConfiguration.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ControllerConfigurationList
    let len;
    let data = new ControllerConfigurationList(null);
    // Deserialize message field [controller_configurations]
    // Deserialize array length for message field [controller_configurations]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.controller_configurations = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.controller_configurations[i] = ControllerConfiguration.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.controller_configurations.forEach((val) => {
      length += ControllerConfiguration.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/ControllerConfigurationList';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4e505f81204befaff98df48e637201d5';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    ControllerConfiguration[] controller_configurations
    ================================================================================
    MSG: kortex_driver/ControllerConfiguration
    
    ControllerHandle handle
    string name
    MappingHandle active_mapping_handle
    string analog_input_identifier_enum
    string digital_input_identifier_enum
    ================================================================================
    MSG: kortex_driver/ControllerHandle
    
    uint32 type
    uint32 controller_identifier
    ================================================================================
    MSG: kortex_driver/MappingHandle
    
    uint32 identifier
    uint32 permission
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ControllerConfigurationList(null);
    if (msg.controller_configurations !== undefined) {
      resolved.controller_configurations = new Array(msg.controller_configurations.length);
      for (let i = 0; i < resolved.controller_configurations.length; ++i) {
        resolved.controller_configurations[i] = ControllerConfiguration.Resolve(msg.controller_configurations[i]);
      }
    }
    else {
      resolved.controller_configurations = []
    }

    return resolved;
    }
};

module.exports = ControllerConfigurationList;
