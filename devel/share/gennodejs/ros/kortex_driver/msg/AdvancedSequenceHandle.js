// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let SequenceHandle = require('./SequenceHandle.js');

//-----------------------------------------------------------

class AdvancedSequenceHandle {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.handle = null;
      this.in_loop = null;
    }
    else {
      if (initObj.hasOwnProperty('handle')) {
        this.handle = initObj.handle
      }
      else {
        this.handle = new SequenceHandle();
      }
      if (initObj.hasOwnProperty('in_loop')) {
        this.in_loop = initObj.in_loop
      }
      else {
        this.in_loop = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type AdvancedSequenceHandle
    // Serialize message field [handle]
    bufferOffset = SequenceHandle.serialize(obj.handle, buffer, bufferOffset);
    // Serialize message field [in_loop]
    bufferOffset = _serializer.bool(obj.in_loop, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type AdvancedSequenceHandle
    let len;
    let data = new AdvancedSequenceHandle(null);
    // Deserialize message field [handle]
    data.handle = SequenceHandle.deserialize(buffer, bufferOffset);
    // Deserialize message field [in_loop]
    data.in_loop = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 9;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/AdvancedSequenceHandle';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7661973c2e6febb748e9471014616c3e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    SequenceHandle handle
    bool in_loop
    ================================================================================
    MSG: kortex_driver/SequenceHandle
    
    uint32 identifier
    uint32 permission
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new AdvancedSequenceHandle(null);
    if (msg.handle !== undefined) {
      resolved.handle = SequenceHandle.Resolve(msg.handle)
    }
    else {
      resolved.handle = new SequenceHandle()
    }

    if (msg.in_loop !== undefined) {
      resolved.in_loop = msg.in_loop;
    }
    else {
      resolved.in_loop = false
    }

    return resolved;
    }
};

module.exports = AdvancedSequenceHandle;
