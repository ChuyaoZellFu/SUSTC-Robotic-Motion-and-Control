// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class DistortionCoefficients {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.k1 = null;
      this.k2 = null;
      this.k3 = null;
      this.p1 = null;
      this.p2 = null;
    }
    else {
      if (initObj.hasOwnProperty('k1')) {
        this.k1 = initObj.k1
      }
      else {
        this.k1 = 0.0;
      }
      if (initObj.hasOwnProperty('k2')) {
        this.k2 = initObj.k2
      }
      else {
        this.k2 = 0.0;
      }
      if (initObj.hasOwnProperty('k3')) {
        this.k3 = initObj.k3
      }
      else {
        this.k3 = 0.0;
      }
      if (initObj.hasOwnProperty('p1')) {
        this.p1 = initObj.p1
      }
      else {
        this.p1 = 0.0;
      }
      if (initObj.hasOwnProperty('p2')) {
        this.p2 = initObj.p2
      }
      else {
        this.p2 = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DistortionCoefficients
    // Serialize message field [k1]
    bufferOffset = _serializer.float32(obj.k1, buffer, bufferOffset);
    // Serialize message field [k2]
    bufferOffset = _serializer.float32(obj.k2, buffer, bufferOffset);
    // Serialize message field [k3]
    bufferOffset = _serializer.float32(obj.k3, buffer, bufferOffset);
    // Serialize message field [p1]
    bufferOffset = _serializer.float32(obj.p1, buffer, bufferOffset);
    // Serialize message field [p2]
    bufferOffset = _serializer.float32(obj.p2, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DistortionCoefficients
    let len;
    let data = new DistortionCoefficients(null);
    // Deserialize message field [k1]
    data.k1 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [k2]
    data.k2 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [k3]
    data.k3 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [p1]
    data.p1 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [p2]
    data.p2 = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 20;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/DistortionCoefficients';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3d597195629c570d527b87022489fa04';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    float32 k1
    float32 k2
    float32 k3
    float32 p1
    float32 p2
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new DistortionCoefficients(null);
    if (msg.k1 !== undefined) {
      resolved.k1 = msg.k1;
    }
    else {
      resolved.k1 = 0.0
    }

    if (msg.k2 !== undefined) {
      resolved.k2 = msg.k2;
    }
    else {
      resolved.k2 = 0.0
    }

    if (msg.k3 !== undefined) {
      resolved.k3 = msg.k3;
    }
    else {
      resolved.k3 = 0.0
    }

    if (msg.p1 !== undefined) {
      resolved.p1 = msg.p1;
    }
    else {
      resolved.p1 = 0.0
    }

    if (msg.p2 !== undefined) {
      resolved.p2 = msg.p2;
    }
    else {
      resolved.p2 = 0.0
    }

    return resolved;
    }
};

module.exports = DistortionCoefficients;
