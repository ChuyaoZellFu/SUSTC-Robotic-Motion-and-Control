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

class Admittance {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.admittance_mode = null;
    }
    else {
      if (initObj.hasOwnProperty('admittance_mode')) {
        this.admittance_mode = initObj.admittance_mode
      }
      else {
        this.admittance_mode = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Admittance
    // Serialize message field [admittance_mode]
    bufferOffset = _serializer.uint32(obj.admittance_mode, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Admittance
    let len;
    let data = new Admittance(null);
    // Deserialize message field [admittance_mode]
    data.admittance_mode = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/Admittance';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5f17929e02863c928199e8741332cc24';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 admittance_mode
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Admittance(null);
    if (msg.admittance_mode !== undefined) {
      resolved.admittance_mode = msg.admittance_mode;
    }
    else {
      resolved.admittance_mode = 0
    }

    return resolved;
    }
};

module.exports = Admittance;
