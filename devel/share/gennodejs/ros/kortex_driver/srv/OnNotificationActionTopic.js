// Auto-generated. Do not edit!

// (in-package kortex_driver.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let NotificationOptions = require('../msg/NotificationOptions.js');

//-----------------------------------------------------------

let NotificationHandle = require('../msg/NotificationHandle.js');

//-----------------------------------------------------------

class OnNotificationActionTopicRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.input = null;
    }
    else {
      if (initObj.hasOwnProperty('input')) {
        this.input = initObj.input
      }
      else {
        this.input = new NotificationOptions();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type OnNotificationActionTopicRequest
    // Serialize message field [input]
    bufferOffset = NotificationOptions.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type OnNotificationActionTopicRequest
    let len;
    let data = new OnNotificationActionTopicRequest(null);
    // Deserialize message field [input]
    data.input = NotificationOptions.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 12;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/OnNotificationActionTopicRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '942d7c40a40678d0347656d7bafd96e9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    NotificationOptions input
    
    ================================================================================
    MSG: kortex_driver/NotificationOptions
    
    uint32 type
    uint32 rate_m_sec
    float32 threshold_value
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new OnNotificationActionTopicRequest(null);
    if (msg.input !== undefined) {
      resolved.input = NotificationOptions.Resolve(msg.input)
    }
    else {
      resolved.input = new NotificationOptions()
    }

    return resolved;
    }
};

class OnNotificationActionTopicResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.output = null;
    }
    else {
      if (initObj.hasOwnProperty('output')) {
        this.output = initObj.output
      }
      else {
        this.output = new NotificationHandle();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type OnNotificationActionTopicResponse
    // Serialize message field [output]
    bufferOffset = NotificationHandle.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type OnNotificationActionTopicResponse
    let len;
    let data = new OnNotificationActionTopicResponse(null);
    // Deserialize message field [output]
    data.output = NotificationHandle.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/OnNotificationActionTopicResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '29ff9348c5c8343d487a90668267a29e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    NotificationHandle output
    
    ================================================================================
    MSG: kortex_driver/NotificationHandle
    
    uint32 identifier
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new OnNotificationActionTopicResponse(null);
    if (msg.output !== undefined) {
      resolved.output = NotificationHandle.Resolve(msg.output)
    }
    else {
      resolved.output = new NotificationHandle()
    }

    return resolved;
    }
};

module.exports = {
  Request: OnNotificationActionTopicRequest,
  Response: OnNotificationActionTopicResponse,
  md5sum() { return '6fefdd07c6cb63a94f7b48e7e07e815b'; },
  datatype() { return 'kortex_driver/OnNotificationActionTopic'; }
};
