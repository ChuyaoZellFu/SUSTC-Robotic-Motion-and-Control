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

class NotificationType {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type NotificationType
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type NotificationType
    let len;
    let data = new NotificationType(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/NotificationType';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9d8153f0fe98641698596673829b2649';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 NOTIFICATION_TYPE_UNSPECIFIED = 0
    
    uint32 NOTIFICATION_TYPE_THRESHOLD = 1
    
    uint32 NOTIFICATION_TYPE_FIX_RATE = 2
    
    uint32 NOTIFICATION_TYPE_EVENT = 3
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new NotificationType(null);
    return resolved;
    }
};

// Constants for message
NotificationType.Constants = {
  NOTIFICATION_TYPE_UNSPECIFIED: 0,
  NOTIFICATION_TYPE_THRESHOLD: 1,
  NOTIFICATION_TYPE_FIX_RATE: 2,
  NOTIFICATION_TYPE_EVENT: 3,
}

module.exports = NotificationType;
