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

class SequenceTaskHandle {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.sequence_handle = null;
      this.task_index = null;
    }
    else {
      if (initObj.hasOwnProperty('sequence_handle')) {
        this.sequence_handle = initObj.sequence_handle
      }
      else {
        this.sequence_handle = new SequenceHandle();
      }
      if (initObj.hasOwnProperty('task_index')) {
        this.task_index = initObj.task_index
      }
      else {
        this.task_index = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SequenceTaskHandle
    // Serialize message field [sequence_handle]
    bufferOffset = SequenceHandle.serialize(obj.sequence_handle, buffer, bufferOffset);
    // Serialize message field [task_index]
    bufferOffset = _serializer.uint32(obj.task_index, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SequenceTaskHandle
    let len;
    let data = new SequenceTaskHandle(null);
    // Deserialize message field [sequence_handle]
    data.sequence_handle = SequenceHandle.deserialize(buffer, bufferOffset);
    // Deserialize message field [task_index]
    data.task_index = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/SequenceTaskHandle';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd86cef527dffeac4930486133afaaef1';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    SequenceHandle sequence_handle
    uint32 task_index
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
    const resolved = new SequenceTaskHandle(null);
    if (msg.sequence_handle !== undefined) {
      resolved.sequence_handle = SequenceHandle.Resolve(msg.sequence_handle)
    }
    else {
      resolved.sequence_handle = new SequenceHandle()
    }

    if (msg.task_index !== undefined) {
      resolved.task_index = msg.task_index;
    }
    else {
      resolved.task_index = 0
    }

    return resolved;
    }
};

module.exports = SequenceTaskHandle;
