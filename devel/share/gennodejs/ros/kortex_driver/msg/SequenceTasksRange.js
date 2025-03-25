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

class SequenceTasksRange {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.first_task_index = null;
      this.second_task_index = null;
    }
    else {
      if (initObj.hasOwnProperty('first_task_index')) {
        this.first_task_index = initObj.first_task_index
      }
      else {
        this.first_task_index = 0;
      }
      if (initObj.hasOwnProperty('second_task_index')) {
        this.second_task_index = initObj.second_task_index
      }
      else {
        this.second_task_index = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SequenceTasksRange
    // Serialize message field [first_task_index]
    bufferOffset = _serializer.uint32(obj.first_task_index, buffer, bufferOffset);
    // Serialize message field [second_task_index]
    bufferOffset = _serializer.uint32(obj.second_task_index, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SequenceTasksRange
    let len;
    let data = new SequenceTasksRange(null);
    // Deserialize message field [first_task_index]
    data.first_task_index = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [second_task_index]
    data.second_task_index = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/SequenceTasksRange';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f9dba2f68de4791eab54435a0c4b312c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 first_task_index
    uint32 second_task_index
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SequenceTasksRange(null);
    if (msg.first_task_index !== undefined) {
      resolved.first_task_index = msg.first_task_index;
    }
    else {
      resolved.first_task_index = 0
    }

    if (msg.second_task_index !== undefined) {
      resolved.second_task_index = msg.second_task_index;
    }
    else {
      resolved.second_task_index = 0
    }

    return resolved;
    }
};

module.exports = SequenceTasksRange;
