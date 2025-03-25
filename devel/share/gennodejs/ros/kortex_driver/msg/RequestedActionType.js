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

class RequestedActionType {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.action_type = null;
    }
    else {
      if (initObj.hasOwnProperty('action_type')) {
        this.action_type = initObj.action_type
      }
      else {
        this.action_type = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type RequestedActionType
    // Serialize message field [action_type]
    bufferOffset = _serializer.uint32(obj.action_type, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RequestedActionType
    let len;
    let data = new RequestedActionType(null);
    // Deserialize message field [action_type]
    data.action_type = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/RequestedActionType';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '38655324c3c40f2a5f27841a581c389b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 action_type
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new RequestedActionType(null);
    if (msg.action_type !== undefined) {
      resolved.action_type = msg.action_type;
    }
    else {
      resolved.action_type = 0
    }

    return resolved;
    }
};

module.exports = RequestedActionType;
