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

class EventIdSequenceInfoNotification {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type EventIdSequenceInfoNotification
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type EventIdSequenceInfoNotification
    let len;
    let data = new EventIdSequenceInfoNotification(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/EventIdSequenceInfoNotification';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '68fe86a38033c2b20696485a1800dc6a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 UNSPECIFIED_SEQUENCE_EVENT = 0
    
    uint32 SEQUENCE_COMPLETED = 1
    
    uint32 SEQUENCE_ABORTED = 2
    
    uint32 SEQUENCE_PAUSED = 3
    
    uint32 SEQUENCE_TASK_STARTED = 4
    
    uint32 SEQUENCE_TASK_COMPLETED = 5
    
    uint32 SEQUENCE_STARTED = 6
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new EventIdSequenceInfoNotification(null);
    return resolved;
    }
};

// Constants for message
EventIdSequenceInfoNotification.Constants = {
  UNSPECIFIED_SEQUENCE_EVENT: 0,
  SEQUENCE_COMPLETED: 1,
  SEQUENCE_ABORTED: 2,
  SEQUENCE_PAUSED: 3,
  SEQUENCE_TASK_STARTED: 4,
  SEQUENCE_TASK_COMPLETED: 5,
  SEQUENCE_STARTED: 6,
}

module.exports = EventIdSequenceInfoNotification;
