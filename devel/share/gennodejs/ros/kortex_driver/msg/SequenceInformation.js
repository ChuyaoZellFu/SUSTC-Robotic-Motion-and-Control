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

class SequenceInformation {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.event_identifier = null;
      this.task_index = null;
      this.task_identifier = null;
    }
    else {
      if (initObj.hasOwnProperty('event_identifier')) {
        this.event_identifier = initObj.event_identifier
      }
      else {
        this.event_identifier = 0;
      }
      if (initObj.hasOwnProperty('task_index')) {
        this.task_index = initObj.task_index
      }
      else {
        this.task_index = 0;
      }
      if (initObj.hasOwnProperty('task_identifier')) {
        this.task_identifier = initObj.task_identifier
      }
      else {
        this.task_identifier = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SequenceInformation
    // Serialize message field [event_identifier]
    bufferOffset = _serializer.uint32(obj.event_identifier, buffer, bufferOffset);
    // Serialize message field [task_index]
    bufferOffset = _serializer.uint32(obj.task_index, buffer, bufferOffset);
    // Serialize message field [task_identifier]
    bufferOffset = _serializer.uint32(obj.task_identifier, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SequenceInformation
    let len;
    let data = new SequenceInformation(null);
    // Deserialize message field [event_identifier]
    data.event_identifier = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [task_index]
    data.task_index = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [task_identifier]
    data.task_identifier = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/SequenceInformation';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a7eedb18a251848053b5142e4934ee01';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 event_identifier
    uint32 task_index
    uint32 task_identifier
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SequenceInformation(null);
    if (msg.event_identifier !== undefined) {
      resolved.event_identifier = msg.event_identifier;
    }
    else {
      resolved.event_identifier = 0
    }

    if (msg.task_index !== undefined) {
      resolved.task_index = msg.task_index;
    }
    else {
      resolved.task_index = 0
    }

    if (msg.task_identifier !== undefined) {
      resolved.task_identifier = msg.task_identifier;
    }
    else {
      resolved.task_identifier = 0
    }

    return resolved;
    }
};

module.exports = SequenceInformation;
