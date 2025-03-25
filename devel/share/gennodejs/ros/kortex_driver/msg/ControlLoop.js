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

class ControlLoop {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.control_loop = null;
    }
    else {
      if (initObj.hasOwnProperty('control_loop')) {
        this.control_loop = initObj.control_loop
      }
      else {
        this.control_loop = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ControlLoop
    // Serialize message field [control_loop]
    bufferOffset = _serializer.uint32(obj.control_loop, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ControlLoop
    let len;
    let data = new ControlLoop(null);
    // Deserialize message field [control_loop]
    data.control_loop = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/ControlLoop';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '26936d7a4ae5672d376ef90225be07d4';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 control_loop
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ControlLoop(null);
    if (msg.control_loop !== undefined) {
      resolved.control_loop = msg.control_loop;
    }
    else {
      resolved.control_loop = 0
    }

    return resolved;
    }
};

module.exports = ControlLoop;
