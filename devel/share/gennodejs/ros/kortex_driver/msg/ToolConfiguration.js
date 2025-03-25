// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let CartesianTransform = require('./CartesianTransform.js');
let ControlConfig_Position = require('./ControlConfig_Position.js');

//-----------------------------------------------------------

class ToolConfiguration {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.tool_transform = null;
      this.tool_mass = null;
      this.tool_mass_center = null;
    }
    else {
      if (initObj.hasOwnProperty('tool_transform')) {
        this.tool_transform = initObj.tool_transform
      }
      else {
        this.tool_transform = new CartesianTransform();
      }
      if (initObj.hasOwnProperty('tool_mass')) {
        this.tool_mass = initObj.tool_mass
      }
      else {
        this.tool_mass = 0.0;
      }
      if (initObj.hasOwnProperty('tool_mass_center')) {
        this.tool_mass_center = initObj.tool_mass_center
      }
      else {
        this.tool_mass_center = new ControlConfig_Position();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ToolConfiguration
    // Serialize message field [tool_transform]
    bufferOffset = CartesianTransform.serialize(obj.tool_transform, buffer, bufferOffset);
    // Serialize message field [tool_mass]
    bufferOffset = _serializer.float32(obj.tool_mass, buffer, bufferOffset);
    // Serialize message field [tool_mass_center]
    bufferOffset = ControlConfig_Position.serialize(obj.tool_mass_center, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ToolConfiguration
    let len;
    let data = new ToolConfiguration(null);
    // Deserialize message field [tool_transform]
    data.tool_transform = CartesianTransform.deserialize(buffer, bufferOffset);
    // Deserialize message field [tool_mass]
    data.tool_mass = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [tool_mass_center]
    data.tool_mass_center = ControlConfig_Position.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 40;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/ToolConfiguration';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'cf7e6c29cefe7625fb7412c1a3c76941';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    CartesianTransform tool_transform
    float32 tool_mass
    ControlConfig_Position tool_mass_center
    ================================================================================
    MSG: kortex_driver/CartesianTransform
    
    float32 x
    float32 y
    float32 z
    float32 theta_x
    float32 theta_y
    float32 theta_z
    ================================================================================
    MSG: kortex_driver/ControlConfig_Position
    
    float32 x
    float32 y
    float32 z
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ToolConfiguration(null);
    if (msg.tool_transform !== undefined) {
      resolved.tool_transform = CartesianTransform.Resolve(msg.tool_transform)
    }
    else {
      resolved.tool_transform = new CartesianTransform()
    }

    if (msg.tool_mass !== undefined) {
      resolved.tool_mass = msg.tool_mass;
    }
    else {
      resolved.tool_mass = 0.0
    }

    if (msg.tool_mass_center !== undefined) {
      resolved.tool_mass_center = ControlConfig_Position.Resolve(msg.tool_mass_center)
    }
    else {
      resolved.tool_mass_center = new ControlConfig_Position()
    }

    return resolved;
    }
};

module.exports = ToolConfiguration;
