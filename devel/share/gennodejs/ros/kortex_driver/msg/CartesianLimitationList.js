// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let CartesianLimitation = require('./CartesianLimitation.js');

//-----------------------------------------------------------

class CartesianLimitationList {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.limitations = null;
    }
    else {
      if (initObj.hasOwnProperty('limitations')) {
        this.limitations = initObj.limitations
      }
      else {
        this.limitations = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CartesianLimitationList
    // Serialize message field [limitations]
    // Serialize the length for message field [limitations]
    bufferOffset = _serializer.uint32(obj.limitations.length, buffer, bufferOffset);
    obj.limitations.forEach((val) => {
      bufferOffset = CartesianLimitation.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CartesianLimitationList
    let len;
    let data = new CartesianLimitationList(null);
    // Deserialize message field [limitations]
    // Deserialize array length for message field [limitations]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.limitations = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.limitations[i] = CartesianLimitation.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 12 * object.limitations.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/CartesianLimitationList';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c96ad5571d257cbcdf15809b29c600bb';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    CartesianLimitation[] limitations
    ================================================================================
    MSG: kortex_driver/CartesianLimitation
    
    uint32 type
    float32 translation
    float32 orientation
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CartesianLimitationList(null);
    if (msg.limitations !== undefined) {
      resolved.limitations = new Array(msg.limitations.length);
      for (let i = 0; i < resolved.limitations.length; ++i) {
        resolved.limitations[i] = CartesianLimitation.Resolve(msg.limitations[i]);
      }
    }
    else {
      resolved.limitations = []
    }

    return resolved;
    }
};

module.exports = CartesianLimitationList;
