// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let ControllerHandle = require('./ControllerHandle.js');

//-----------------------------------------------------------

class ControllerList {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.handles = null;
    }
    else {
      if (initObj.hasOwnProperty('handles')) {
        this.handles = initObj.handles
      }
      else {
        this.handles = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ControllerList
    // Serialize message field [handles]
    // Serialize the length for message field [handles]
    bufferOffset = _serializer.uint32(obj.handles.length, buffer, bufferOffset);
    obj.handles.forEach((val) => {
      bufferOffset = ControllerHandle.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ControllerList
    let len;
    let data = new ControllerList(null);
    // Deserialize message field [handles]
    // Deserialize array length for message field [handles]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.handles = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.handles[i] = ControllerHandle.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.handles.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/ControllerList';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '952a49d983a336790fc984bb01df940e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    ControllerHandle[] handles
    ================================================================================
    MSG: kortex_driver/ControllerHandle
    
    uint32 type
    uint32 controller_identifier
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ControllerList(null);
    if (msg.handles !== undefined) {
      resolved.handles = new Array(msg.handles.length);
      for (let i = 0; i < resolved.handles.length; ++i) {
        resolved.handles[i] = ControllerHandle.Resolve(msg.handles[i]);
      }
    }
    else {
      resolved.handles = []
    }

    return resolved;
    }
};

module.exports = ControllerList;
