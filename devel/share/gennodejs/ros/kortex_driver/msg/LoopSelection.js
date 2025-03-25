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

class LoopSelection {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.loop_selection = null;
    }
    else {
      if (initObj.hasOwnProperty('loop_selection')) {
        this.loop_selection = initObj.loop_selection
      }
      else {
        this.loop_selection = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type LoopSelection
    // Serialize message field [loop_selection]
    bufferOffset = _serializer.uint32(obj.loop_selection, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type LoopSelection
    let len;
    let data = new LoopSelection(null);
    // Deserialize message field [loop_selection]
    data.loop_selection = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/LoopSelection';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '404b340f5699ef6e44d5690bdce228f3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 loop_selection
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new LoopSelection(null);
    if (msg.loop_selection !== undefined) {
      resolved.loop_selection = msg.loop_selection;
    }
    else {
      resolved.loop_selection = 0
    }

    return resolved;
    }
};

module.exports = LoopSelection;
