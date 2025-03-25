// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let SafetyHandle = require('./SafetyHandle.js');

//-----------------------------------------------------------

class SafetyEvent {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.safety_handle = null;
    }
    else {
      if (initObj.hasOwnProperty('safety_handle')) {
        this.safety_handle = initObj.safety_handle
      }
      else {
        this.safety_handle = new SafetyHandle();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SafetyEvent
    // Serialize message field [safety_handle]
    bufferOffset = SafetyHandle.serialize(obj.safety_handle, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SafetyEvent
    let len;
    let data = new SafetyEvent(null);
    // Deserialize message field [safety_handle]
    data.safety_handle = SafetyHandle.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/SafetyEvent';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '48c102b68919349f9a9bb31c7e295748';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    SafetyHandle safety_handle
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
    const resolved = new SafetyEvent(null);
    if (msg.safety_handle !== undefined) {
      resolved.safety_handle = SafetyHandle.Resolve(msg.safety_handle)
    }
    else {
      resolved.safety_handle = new SafetyHandle()
    }

    return resolved;
    }
};

module.exports = SafetyEvent;
