// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let SafetyInformation = require('./SafetyInformation.js');

//-----------------------------------------------------------

class SafetyInformationList {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.information = null;
    }
    else {
      if (initObj.hasOwnProperty('information')) {
        this.information = initObj.information
      }
      else {
        this.information = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SafetyInformationList
    // Serialize message field [information]
    // Serialize the length for message field [information]
    bufferOffset = _serializer.uint32(obj.information.length, buffer, bufferOffset);
    obj.information.forEach((val) => {
      bufferOffset = SafetyInformation.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SafetyInformationList
    let len;
    let data = new SafetyInformationList(null);
    // Deserialize message field [information]
    // Deserialize array length for message field [information]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.information = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.information[i] = SafetyInformation.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 35 * object.information.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/SafetyInformationList';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c4049b5c5ed2e1815e6545bc46f6cf34';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    SafetyInformation[] information
    ================================================================================
    MSG: kortex_driver/SafetyInformation
    
    SafetyHandle handle
    bool can_change_safety_state
    bool has_warning_threshold
    bool has_error_threshold
    uint32 limit_type
    float32 default_warning_threshold
    float32 default_error_threshold
    float32 upper_hard_limit
    float32 lower_hard_limit
    uint32 status
    uint32 unit
    ================================================================================
    MSG: kortex_driver/SafetyHandle
    
    uint32 identifier
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SafetyInformationList(null);
    if (msg.information !== undefined) {
      resolved.information = new Array(msg.information.length);
      for (let i = 0; i < resolved.information.length; ++i) {
        resolved.information[i] = SafetyInformation.Resolve(msg.information[i]);
      }
    }
    else {
      resolved.information = []
    }

    return resolved;
    }
};

module.exports = SafetyInformationList;
