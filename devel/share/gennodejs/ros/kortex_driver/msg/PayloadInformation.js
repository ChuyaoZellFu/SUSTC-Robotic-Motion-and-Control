// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let ControlConfig_Position = require('./ControlConfig_Position.js');

//-----------------------------------------------------------

class PayloadInformation {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.payload_mass = null;
      this.payload_mass_center = null;
    }
    else {
      if (initObj.hasOwnProperty('payload_mass')) {
        this.payload_mass = initObj.payload_mass
      }
      else {
        this.payload_mass = 0.0;
      }
      if (initObj.hasOwnProperty('payload_mass_center')) {
        this.payload_mass_center = initObj.payload_mass_center
      }
      else {
        this.payload_mass_center = new ControlConfig_Position();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PayloadInformation
    // Serialize message field [payload_mass]
    bufferOffset = _serializer.float32(obj.payload_mass, buffer, bufferOffset);
    // Serialize message field [payload_mass_center]
    bufferOffset = ControlConfig_Position.serialize(obj.payload_mass_center, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PayloadInformation
    let len;
    let data = new PayloadInformation(null);
    // Deserialize message field [payload_mass]
    data.payload_mass = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [payload_mass_center]
    data.payload_mass_center = ControlConfig_Position.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/PayloadInformation';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b6414681a38d62f0a2b6ff0a6a813f53';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    float32 payload_mass
    ControlConfig_Position payload_mass_center
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
    const resolved = new PayloadInformation(null);
    if (msg.payload_mass !== undefined) {
      resolved.payload_mass = msg.payload_mass;
    }
    else {
      resolved.payload_mass = 0.0
    }

    if (msg.payload_mass_center !== undefined) {
      resolved.payload_mass_center = ControlConfig_Position.Resolve(msg.payload_mass_center)
    }
    else {
      resolved.payload_mass_center = new ControlConfig_Position()
    }

    return resolved;
    }
};

module.exports = PayloadInformation;
