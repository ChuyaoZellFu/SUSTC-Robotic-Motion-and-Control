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

class RunMode {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.run_mode = null;
    }
    else {
      if (initObj.hasOwnProperty('run_mode')) {
        this.run_mode = initObj.run_mode
      }
      else {
        this.run_mode = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type RunMode
    // Serialize message field [run_mode]
    bufferOffset = _serializer.uint32(obj.run_mode, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RunMode
    let len;
    let data = new RunMode(null);
    // Deserialize message field [run_mode]
    data.run_mode = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/RunMode';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'abe2eb774be1b28bf7c30a06ea5e3691';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 run_mode
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new RunMode(null);
    if (msg.run_mode !== undefined) {
      resolved.run_mode = msg.run_mode;
    }
    else {
      resolved.run_mode = 0
    }

    return resolved;
    }
};

module.exports = RunMode;
